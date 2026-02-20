# SteerPy author, 2026.

class WorldModel:
    """
    Runtime world object passed to simulate_one_step(car, world_model).

    frame_id: int
    dt: float (seconds)
    road_data:
      width, sample_distance, waypoints, path
    loop:
      bool loop/open flag from world config
    obstacles:
      list of dicts: x, y, length, width, heading_deg

    Methods:
      set_runtime(...), set_road_data(...), set_obstacles(...), set_scenario(...), snapshot()
    """

    def __init__(self):
        self.frame_id = 0
        self.dt = 0.0
        self.loop = True
        self.road_data = {
            "width": 10.0,
            "sample_distance": 1.0,
            "waypoints": [],
            "path": [],
        }
        self.obstacles = []
        self.scenario = {
            "road_data": self.road_data,
            "loop": self.loop,
            "obstacles": self.obstacles,
        }

    @staticmethod
    def _to_xy_tuples(values):
        points = []
        for value in values or []:
            try:
                if len(value) >= 2:
                    points.append((float(value[0]), float(value[1])))
            except Exception:
                continue
        return points

    @staticmethod
    def _catmull_rom_point(p0, p1, p2, p3, t):
        t2 = t * t
        t3 = t2 * t
        x = 0.5 * (
            (2.0 * p1[0])
            + (-p0[0] + p2[0]) * t
            + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
            + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
        )
        y = 0.5 * (
            (2.0 * p1[1])
            + (-p0[1] + p2[1]) * t
            + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
            + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
        )
        return (x, y)

    def _build_spline_path(self, waypoints, sample_distance):
        pts = list(waypoints or [])
        n = len(pts)
        if n < 2:
            return pts

        closed = bool(self.loop)

        def get_p(i):
            if closed:
                return pts[i % n]
            return pts[max(0, min(n - 1, i))]

        spans = n if closed else n - 1
        out = []
        sample_distance = max(1e-3, float(sample_distance))

        for s in range(spans):
            p0 = get_p(s - 1)
            p1 = get_p(s)
            p2 = get_p(s + 1)
            p3 = get_p(s + 2)

            chord = max(1e-3, ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5)
            segs = max(4, int(round(chord / sample_distance)))

            for k in range(segs):
                t = k / float(segs)
                out.append(self._catmull_rom_point(p0, p1, p2, p3, t))

        if not closed:
            out.append(pts[-1])
        return out

    @staticmethod
    def _to_obstacle_dicts(obstacles):
        if not isinstance(obstacles, list):
            try:
                obstacles = list(obstacles)
            except Exception:
                obstacles = []

        out = []
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                try:
                    obstacle = dict(obstacle)
                except Exception:
                    continue
            out.append(
                {
                    "x": float(obstacle.get("x", 0.0)),
                    "y": float(obstacle.get("y", 0.0)),
                    "length": float(obstacle.get("length", 0.0)),
                    "width": float(obstacle.get("width", 0.0)),
                    "heading_deg": float(obstacle.get("heading_deg", 0.0)),
                }
            )
        return out

    def _refresh_scenario(self):
        self.scenario["road_data"] = self.road_data
        self.scenario["loop"] = self.loop
        self.scenario["obstacles"] = self.obstacles

    def set_runtime(self, frame_id, dt, road_width=None, road_path=None, loop_flag=None):
        self.frame_id = int(frame_id)
        self.dt = float(dt)
        if loop_flag is not None:
            self.loop = bool(loop_flag)
        if road_width is not None:
            self.road_data["width"] = float(road_width)
        if road_path is not None:
            self.road_data["path"] = self._to_xy_tuples(road_path)
        self._refresh_scenario()

    def set_road_data(self, road_data):
        if not isinstance(road_data, dict):
            try:
                road_data = dict(road_data)
            except Exception:
                road_data = {}

        width = float(road_data.get("width", self.road_data.get("width", 10.0)))
        sample_distance = max(
            1e-3,
            float(road_data.get("sample_distance", self.road_data.get("sample_distance", 1.0))),
        )
        waypoints = self._to_xy_tuples(road_data.get("waypoints", []))
        path = self._build_spline_path(waypoints, sample_distance)

        self.road_data = {
            "width": width,
            "sample_distance": sample_distance,
            "waypoints": waypoints,
            "path": path,
        }
        self._refresh_scenario()

    def set_obstacles(self, obstacles):
        self.obstacles = self._to_obstacle_dicts(obstacles)
        self._refresh_scenario()

    def set_scenario(self, road_data, obstacles, loop_flag=True):
        self.loop = bool(loop_flag)
        self.set_road_data(road_data)
        self.set_obstacles(obstacles)
        self._refresh_scenario()

    def snapshot(self):
        return {
            "frame_id": self.frame_id,
            "dt": self.dt,
            "scenario": {
                "road_data": dict(self.road_data),
                "loop": self.loop,
                "obstacles": [dict(o) for o in self.obstacles],
            },
        }
