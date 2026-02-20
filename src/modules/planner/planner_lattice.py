# SteerPy author, 2026.

# planner.py
# planner(car, world_model) -> [(x, y, target_speed), ...]
#
# Available state:
#   car.x, car.y, car.angle, car.speed
#   car.length, car.width, car.wheelbase
#   car.min_speed, car.max_speed
#   car.min_accel, car.max_accel
#   car.min_steering_angle_deg, car.max_steering_angle_deg
#   car.min_steering_speed_deg_s, car.max_steering_speed_deg_s
#   car.steer_angle, car.vx, car.vy, car.model, car.control_mode
#
# Sensors (meters, -1 = clear):
#   car.sensors.front, car.sensors.rear, car.sensors.left, car.sensors.right
#   car.sensors.front_left, car.sensors.front_right
#   car.sensors.lidar  # N-beam list (N is runtime-configurable; index 0 = front, increasing CCW)
#
# World model:
#   world_model.frame_id, world_model.dt
#   world_model.road_data with:
#       width, sample_distance, waypoints, path
#   world_model.loop is True for loop tracks
#   world_model.obstacles list of:
#       {x, y, length, width, heading_deg}
#
# Helpers:
#   car.log(msg), print(msg), drawLine(line, width, color)

import math
from bisect import bisect_left

# Lattice planner for PATH ONLY (lateral offsets around a reference line).
# - Constant output speed = 10.0
# - Adds collision cost (soft)
# - Draws ALL candidate paths using:
#     drawLine(points, line_size, line_color)
#
# Fixes applied:
#  1) Force trajectory start point at ego (car.x, car.y)
#  2) Smooth Frenet->XY frame by interpolating yaw between segments (avoid normal jumps)
#  3) Densify reference polyline before building RefPath (reduce kinks / projection jitter)

CONST_SPEED = 10.0
CONST_LINE_WIDTH = 0.08

# Lattice and scoring defaults.
DEFAULT_DENSIFY_STEP = 0.5
LATERAL_STEP = 0.9
MAX_OFFSET_MULT = 5
HORIZON_S = 25.0
MIN_SAMPLE_DS = 0.6
MIN_STEPS = 12
MAX_LATERAL_COMPLETION_S = 18.0
COLLISION_MARGIN = 0.6
EDGE_CLEARANCE_TARGET = 0.6

W_CENTER = 1.0
W_CHANGE = 0.7
W_SMOOTH = 1.2
W_EDGE = 4.0
W_COLL = 200.0

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _hypot(x, y):
    return math.hypot(x, y)

def _safe_get_path(road_data):
    path = road_data.get("path") or road_data.get("waypoints")
    if not path or len(path) < 2:
        return None
    out = []
    for p in path:
        try:
            if isinstance(p, dict):
                out.append((float(p.get("x", 0.0)), float(p.get("y", 0.0))))
            else:
                out.append((float(p[0]), float(p[1])))
        except Exception:
            continue
    if len(out) < 2:
        return None
    return out


def _densify_polyline(pts, step=0.5):
    if not pts or len(pts) < 2:
        return pts
    step = max(1e-3, float(step))
    out = [pts[0]]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            continue

        # insert interior points so segment pieces are <= step (approximately)
        n = int(seg_len / step)
        for k in range(1, n + 1):
            t = k / (n + 1)
            out.append((x0 + dx * t, y0 + dy * t))

        out.append((x1, y1))
    return out

class _RefPath:
    def __init__(self, pts, loop=False):
        self.loop = bool(loop)
        self.pts = pts[:]
        self.n = len(self.pts)

        self.s = [0.0]
        for i in range(1, self.n):
            x0, y0 = self.pts[i - 1]
            x1, y1 = self.pts[i]
            self.s.append(self.s[-1] + _hypot(x1 - x0, y1 - y0))
        self.length = self.s[-1]

        self.seg_yaw = []
        for i in range(self.n - 1):
            x0, y0 = self.pts[i]
            x1, y1 = self.pts[i + 1]
            self.seg_yaw.append(math.atan2(y1 - y0, x1 - x0))
        self.seg_yaw.append(self.seg_yaw[-1] if self.seg_yaw else 0.0)

    def _wrap_s(self, sval):
        if not self.loop or self.length <= 1e-6:
            return _clamp(sval, 0.0, self.length)
        return sval % self.length

    def sample(self, sval):
        """Return (x, y, yaw, nx, ny) at arc-length s."""
        s = self._wrap_s(sval)

        if s >= self.s[-1]:
            i = self.n - 2
        else:
            i = max(0, min(self.n - 2, bisect_left(self.s, s) - 1))

        s0 = self.s[i]
        s1 = self.s[i + 1]
        denom = (s1 - s0) if (s1 - s0) > 1e-9 else 1e-9
        t = (s - s0) / denom

        x0, y0 = self.pts[i]
        x1, y1 = self.pts[i + 1]
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t

        yaw0 = self.seg_yaw[i]
        yaw1 = self.seg_yaw[i + 1] if (i + 1) < len(self.seg_yaw) else yaw0
        # unwrap yaw1 around yaw0 to avoid 2*pi jump
        dyaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
        yaw = yaw0 + t * dyaw

        nx = -math.sin(yaw)  # left normal
        ny =  math.cos(yaw)
        return x, y, yaw, nx, ny

    def project_sd(self, x, y):
        """Project (x,y) to polyline. Return (s_proj, d_proj, yaw). d positive to the left."""
        best_d2 = float("inf")
        best_s = 0.0
        best_d = 0.0
        best_yaw = 0.0

        for i in range(self.n - 1):
            x0, y0 = self.pts[i]
            x1, y1 = self.pts[i + 1]
            vx = x1 - x0
            vy = y1 - y0
            seg_len2 = vx * vx + vy * vy
            if seg_len2 < 1e-9:
                continue

            wx = x - x0
            wy = y - y0
            t = (wx * vx + wy * vy) / seg_len2
            t = _clamp(t, 0.0, 1.0)

            px = x0 + t * vx
            py = y0 + t * vy

            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                seg_len = math.sqrt(seg_len2)
                best_s = self.s[i] + t * seg_len

                yaw = math.atan2(vy, vx)
                nx = -math.sin(yaw)
                ny =  math.cos(yaw)
                best_d = dx * nx + dy * ny
                best_yaw = yaw

        return self._wrap_s(best_s), best_d, best_yaw

class _Quintic1D:
    # d(sigma) from d0->d1 over [0,L], with d'(0)=d''(0)=d'(L)=d''(L)=0
    def __init__(self, d0, d1, L):
        self.L = max(L, 1e-3)
        L = self.L
        D = (d1 - d0)
        self.a0 = d0
        self.a3 =  10.0 * D / (L**3)
        self.a4 = -15.0 * D / (L**4)
        self.a5 =   6.0 * D / (L**5)

    def eval(self, sigma):
        s = _clamp(sigma, 0.0, self.L)
        return self.a0 + self.a3*s**3 + self.a4*s**4 + self.a5*s**5

def _inflate_obs_radius(obs):
    L = float(obs.get("length", 0.0))
    W = float(obs.get("width", 0.0))
    return 0.5 * math.hypot(L, W)

def _car_radius(car):
    return 0.5 * math.hypot(float(car.length), float(car.width))

def _collision_cost(points_xy, obstacles, car_r, margin=0.6):
    # quadratic penetration cost
    if not obstacles:
        return 0.0
    cost = 0.0
    for (x, y) in points_xy:
        for obs in obstacles:
            ox = float(obs.get("x", 0.0))
            oy = float(obs.get("y", 0.0))
            r_total = _inflate_obs_radius(obs) + car_r + margin
            d = math.hypot(x - ox, y - oy)
            pen = r_total - d
            if pen > 0.0:
                cost += pen * pen
    return cost


def _extract_world(world_model):
    scenario = getattr(world_model, "scenario", None)
    if not isinstance(scenario, dict):
        scenario = {}

    road_data = scenario.get("road_data", {}) or {}
    obstacles = scenario.get("obstacles", []) or []
    loop_flag = bool(scenario.get("loop", False))

    # Compatibility path for direct world_model fields.
    if not road_data:
        road_data = getattr(world_model, "road_data", {}) or {}
    if not obstacles:
        obstacles = getattr(world_model, "obstacles", []) or []
    if not loop_flag:
        loop_flag = bool(getattr(world_model, "loop", False))

    return road_data, obstacles, loop_flag


def _draw_candidate(points, color):
    try:
        drawLine(points, CONST_LINE_WIDTH, color)
    except Exception:
        pass

def planner(car, world_model):
    road_data, obstacles, loop_flag = _extract_world(world_model)
    pts = _safe_get_path(road_data)
    if not pts:
        return [(float(car.x), float(car.y), CONST_SPEED)]

    # FIX 3: densify the reference polyline before building RefPath
    pts = _densify_polyline(pts, step=float(road_data.get("densify_step", DEFAULT_DENSIFY_STEP)))

    ref = _RefPath(pts, loop=loop_flag)

    dt = float(getattr(world_model, "dt", 0.1) or 0.1)
    dt = _clamp(dt, 0.05, 0.2)

    # Road bounds
    road_w = float(road_data.get("width", 8.0))
    d_max = max(0.0, 0.5 * road_w - 0.3)

    # Ego in Frenet
    s0, d0, _ = ref.project_sd(float(car.x), float(car.y))
    d0 = _clamp(d0, -d_max, d_max)

    # FIX 1: force the path start exactly at ego pose
    ego_x = float(car.x)
    ego_y = float(car.y)

    # Lattice terminal offsets
    step = LATERAL_STEP
    offsets = [0.0]
    for i in range(1, MAX_OFFSET_MULT + 1):
        offsets.extend([i * step, -i * step])
    d_targets = [_clamp(o, -d_max, d_max) for o in offsets]
    d_targets = sorted(set(round(d, 3) for d in d_targets))

    # Horizon sampling along s
    horizon_s = HORIZON_S
    ds = max(MIN_SAMPLE_DS, CONST_SPEED * dt)
    n_steps = max(MIN_STEPS, int(horizon_s / ds))

    # Lateral shift completion distance
    L_lat = min(MAX_LATERAL_COMPLETION_S, horizon_s)

    car_r = _car_radius(car)

    best_xy = None
    best_cost = float("inf")

    for d_end in d_targets:
        poly = _Quintic1D(d0, d_end, L_lat)

        xy = [(ego_x, ego_y)]
        road_violation = False
        curv_proxy = 0.0
        last_dx = None
        last_dy = None

        for i in range(1, n_steps):
            sigma = i * ds
            s = s0 + sigma
            d = poly.eval(sigma) if sigma <= L_lat else d_end

            if abs(d) > d_max + 1e-6:
                road_violation = True
                break

            rx, ry, _yaw, nx, ny = ref.sample(s)
            x = rx + d * nx
            y = ry + d * ny
            xy.append((x, y))

            if len(xy) >= 2:
                dx = xy[-1][0] - xy[-2][0]
                dy = xy[-1][1] - xy[-2][1]
                if last_dx is not None:
                    a1 = math.atan2(last_dy, last_dx)
                    a2 = math.atan2(dy, dx)
                    da = abs((a2 - a1 + math.pi) % (2*math.pi) - math.pi)
                    curv_proxy += da
                last_dx, last_dy = dx, dy

        if road_violation or len(xy) < 2:
            continue

        coll = _collision_cost(xy, obstacles, car_r, margin=COLLISION_MARGIN)

        center_cost = abs(d_end)
        change_cost = abs(d_end - d0)
        smooth_cost = curv_proxy

        edge_slack = d_max - abs(d_end)
        edge_cost = 0.0 if edge_slack > EDGE_CLEARANCE_TARGET else (EDGE_CLEARANCE_TARGET - edge_slack) ** 2

        cost = (W_CENTER * center_cost +
                W_CHANGE * change_cost +
                W_SMOOTH * smooth_cost +
                W_EDGE   * edge_cost +
                W_COLL   * coll)

        # ---- draw this candidate ----
        color = "green" if coll <= 1e-9 else "red"
        _draw_candidate(xy, color)

        if cost < best_cost:
            best_cost = cost
            best_xy = xy

    if best_xy is None:
        # fallback: draw and follow centerline
        best_xy = [(ego_x, ego_y)]
        for i in range(1, n_steps):
            s = s0 + i * ds
            rx, ry, _yaw, _nx, _ny = ref.sample(s)
            best_xy.append((rx, ry))
        _draw_candidate(best_xy, "blue")
    else:
        # highlight chosen path
        _draw_candidate(best_xy, "cyan")

    return [(x, y, CONST_SPEED) for (x, y) in best_xy]
