# SteerPy author, 2026.

class _PointNormalizer:
    @staticmethod
    def coerce_point(value):
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return [float(value[0]), float(value[1])]
            except Exception:
                return None

        if isinstance(value, dict):
            if "x" in value and "y" in value:
                try:
                    return [float(value["x"]), float(value["y"])]
                except Exception:
                    return None
            if "xy" in value and isinstance(value["xy"], (list, tuple)) and len(value["xy"]) >= 2:
                try:
                    return [float(value["xy"][0]), float(value["xy"][1])]
                except Exception:
                    return None

        return None

    @classmethod
    def normalize_line(cls, line):
        points = []

        if isinstance(line, (list, tuple)):
            if len(line) == 4:
                try:
                    return [
                        [float(line[0]), float(line[1])],
                        [float(line[2]), float(line[3])],
                    ]
                except Exception:
                    pass

            for point in line:
                parsed = cls.coerce_point(point)
                if parsed is not None:
                    points.append(parsed)
            return points

        if isinstance(line, dict):
            if all(k in line for k in ("x1", "y1", "x2", "y2")):
                try:
                    return [
                        [float(line["x1"]), float(line["y1"])],
                        [float(line["x2"]), float(line["y2"])],
                    ]
                except Exception:
                    return []

            line_points = line.get("points", line.get("line"))
            if isinstance(line_points, (list, tuple)):
                for point in line_points:
                    parsed = cls.coerce_point(point)
                    if parsed is not None:
                        points.append(parsed)
                return points

        return []


class RuntimeBindings:
    def __init__(self, draw_line_callback, car=None, sensors=None, world_model=None):
        self._draw_line_callback = draw_line_callback
        self.car = car or Car()
        if sensors is not None:
            self.sensors = sensors
            self.car.sensors = sensors
        else:
            self.sensors = getattr(self.car, "sensors", None)
            if self.sensors is None:
                self.sensors = Sensors()
                self.car.sensors = self.sensors
        self.world_model = world_model or WorldModel()

    def draw_line(self, line, width=0.12, color="rgba(125,249,255,.95)"):
        points = _PointNormalizer.normalize_line(line)
        if len(points) < 2:
            return

        try:
            width_value = float(width)
        except Exception:
            width_value = 0.12

        self._draw_line_callback(
            {"points": points, "width": width_value, "color": str(color)}
        )

    def set_world_runtime(self, frame_id, dt, road_width, road_path=None, loop_flag=True):
        self.world_model.set_runtime(
            frame_id=frame_id,
            dt=dt,
            road_width=road_width,
            road_path=road_path,
            loop_flag=loop_flag,
        )

    def set_world_scenario(self, road_data, obstacles, loop_flag=True):
        self.world_model.set_scenario(
            road_data=road_data,
            obstacles=obstacles,
            loop_flag=loop_flag,
        )


_runtime_bindings = RuntimeBindings(_cb_draw_line)
_car_obj = _runtime_bindings.car
_sensors_obj = _runtime_bindings.sensors
_world_model_obj = _runtime_bindings.world_model


def drawLine(line, width=0.12, color="rgba(125,249,255,.95)"):
    _runtime_bindings.draw_line(line, width, color)


def _set_world_runtime(frame_id, dt, road_width, road_path=None, loop_flag=True):
    _runtime_bindings.set_world_runtime(frame_id, dt, road_width, road_path, loop_flag)


def _set_world_scenario(road_data, obstacles, loop_flag=True):
    _runtime_bindings.set_world_scenario(road_data, obstacles, loop_flag)
