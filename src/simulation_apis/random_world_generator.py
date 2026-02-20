# SteerPy author, 2026.

import math
import random


def _as_num(v, default):
    try:
        n = float(v)
        if math.isfinite(n):
            return n
    except Exception:
        pass
    return float(default)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _rand_range(a, b):
    if b <= a:
        return float(a)
    return random.uniform(a, b)


def _rand_int(a, b):
    lo = int(math.ceil(a))
    hi = int(math.floor(b))
    if hi <= lo:
        return lo
    return random.randint(lo, hi)


def _catmull_rom_chain(pts, closed, segs_per_span=12):
    n = len(pts)
    if n < 2:
        return list(pts)

    def get_p(i):
        if closed:
            return pts[i % n]
        return pts[max(0, min(n - 1, i))]

    spans = n if closed else n - 1
    out = []
    for s in range(spans):
        p0 = get_p(s - 1)
        p1 = get_p(s)
        p2 = get_p(s + 1)
        p3 = get_p(s + 2)
        for k in range(segs_per_span):
            t = k / segs_per_span
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * (
                (2 * p1[0])
                + (-p0[0] + p2[0]) * t
                + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                (2 * p1[1])
                + (-p0[1] + p2[1]) * t
                + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
            )
            out.append((x, y))
    if not closed:
        out.append(pts[-1])
    return out


cfg = _rand_cfg if isinstance(_rand_cfg, dict) else {}

obs_min = _clamp(int(round(_as_num(cfg.get("obsMin"), 4))), 0, 60)
obs_max = _clamp(int(round(_as_num(cfg.get("obsMax"), 10))), 0, 60)
if obs_min > obs_max:
    obs_min, obs_max = obs_max, obs_min

obs_size_min = _clamp(_as_num(cfg.get("obsSizeMin"), 3.0), 1.0, 20.0)
obs_size_max = _clamp(_as_num(cfg.get("obsSizeMax"), 6.0), 1.0, 20.0)
if obs_size_min > obs_size_max:
    obs_size_min, obs_size_max = obs_size_max, obs_size_min

road_len_min = _clamp(_as_num(cfg.get("roadLenMin"), 70.0), 20.0, 400.0)
road_len_max = _clamp(_as_num(cfg.get("roadLenMax"), 140.0), 20.0, 400.0)
if road_len_min > road_len_max:
    road_len_min, road_len_max = road_len_max, road_len_min

road_wid_min = _clamp(_as_num(cfg.get("roadWidthMin"), 7.0), 4.0, 30.0)
road_wid_max = _clamp(_as_num(cfg.get("roadWidthMax"), 12.0), 4.0, 30.0)
if road_wid_min > road_wid_max:
    road_wid_min, road_wid_max = road_wid_max, road_wid_min

road_length = _rand_range(road_len_min, road_len_max)
road_width = _rand_range(road_wid_min, road_wid_max)
sample_distance = 1.0

half_x = max(12.0, road_length * _rand_range(0.30, 0.42))
half_y = max(8.0, road_length * _rand_range(0.18, 0.30))
jitter_x = max(1.0, road_length * 0.05)
jitter_y = max(0.8, road_length * 0.04)

waypoints = [
    (-half_x + _rand_range(-jitter_x, jitter_x), -half_y + _rand_range(-jitter_y, jitter_y)),
    (-(half_x * 0.35) + _rand_range(-jitter_x, jitter_x), -half_y + _rand_range(-jitter_y, jitter_y)),
    (half_x + _rand_range(-jitter_x, jitter_x), -(half_y * 0.35) + _rand_range(-jitter_y, jitter_y)),
    (half_x + _rand_range(-jitter_x, jitter_x), half_y + _rand_range(-jitter_y, jitter_y)),
    (half_x * 0.2 + _rand_range(-jitter_x, jitter_x), half_y + _rand_range(-jitter_y, jitter_y)),
    (-half_x + _rand_range(-jitter_x, jitter_x), half_y * 0.25 + _rand_range(-jitter_y, jitter_y)),
]

closed = True
path = _catmull_rom_chain(waypoints, closed, 12)
target_count = _rand_int(obs_min, obs_max)
obstacles = []

if len(path) >= 10 and target_count > 0:
    n = len(path)
    lane_half = road_width * 0.5
    start_idx = _rand_int(0, n - 1)
    step = max(1.0, n / float(target_count))

    # Stratified placement: one obstacle per longitudinal slot with small jitter.
    # This keeps distribution visually uniform while preserving randomness.
    slot_indices = [int((start_idx + i * step) % n) for i in range(target_count)]

    for i, slot_idx in enumerate(slot_indices):
        placed = False
        for _try in range(36):
            spread = step * min(0.45, 0.15 + 0.08 * _try)
            idx = int((slot_idx + _rand_range(-spread, spread)) % n)

            a = path[(idx - 1 + n) % n]
            b = path[(idx + 1) % n]
            base = path[idx]
            h_rad = math.atan2(b[1] - a[1], b[0] - a[0])
            nx = -math.sin(h_rad)
            ny = math.cos(h_rad)

            side = -1.0 if (i % 2 == 0) else 1.0
            if random.random() < 0.25:
                side *= -1.0
            lateral_mag = _rand_range(road_width * 0.10, road_width * 0.30)
            lateral_jitter = _rand_range(-road_width * 0.03, road_width * 0.03)
            lateral = _clamp(side * lateral_mag + lateral_jitter, -lane_half * 0.85, lane_half * 0.85)

            x = base[0] + nx * lateral
            y = base[1] + ny * lateral
            obs_size = _rand_range(obs_size_min, obs_size_max)
            length = _rand_range(max(1.8, obs_size * 0.8), max(2.2, obs_size * 1.4))
            width = _rand_range(max(1.0, obs_size * 0.4), max(1.3, obs_size * 0.85))
            heading = h_rad * 180.0 / math.pi + _rand_range(-35.0, 35.0)

            if math.hypot(x - waypoints[0][0], y - waypoints[0][1]) < 9.0:
                continue

            min_sep = max(2.0, length * 0.7)
            too_close = False
            for o in obstacles:
                if math.hypot(x - o[0], y - o[1]) < (min_sep + o[2] * 0.5):
                    too_close = True
                    break
            if too_close:
                continue

            obstacles.append((x, y, length, width, heading))
            placed = True
            break
        if not placed:
            continue

_rand_world = {
    "waypoints": waypoints,
    "obstacles": obstacles,
    "road_width": road_width,
    "loop": closed,
    "sample_distance": sample_distance,
    "road_length": road_length,
}
