# SteerPy author, 2026.

class Car:
    """
    Vehicle state object passed to simulate_one_step(car, world_model).

    Read-only (synced from simulator each frame):
        x, y           -- position in meters
        angle          -- heading in degrees (0 = +x/right, +CCW, y up)
        speed          -- scalar speed in m/s
        min_speed      -- speed floor in m/s
        max_speed      -- speed cap in m/s
        length         -- vehicle body length (m)
        width          -- vehicle body width (m)
        wheelbase      -- axle-to-axle distance (m)
        min_accel      -- minimum accel setting (m/s^2)
        max_accel      -- maximum accel setting (m/s^2)
        min_steering_angle_deg -- steering angle lower limit (deg)
        max_steering_angle_deg -- steering angle upper limit (deg)
        min_steering_speed_deg_s -- steering rate lower limit (deg/s)
        max_steering_speed_deg_s -- steering rate upper limit (deg/s)
        steer_angle    -- actual front-wheel angle in degrees (bicycle/ackermann/drift)
        vx, vy         -- velocity components in m/s (drift model)
        model          -- name of active physics model (str)
        control_mode   -- "auto" | "manual"

    Write (set once per frame, reset to 0 before each call):
        throttle       -- acceleration demand  [0, 1]
        brake          -- braking demand       [0, 1]
        steer          -- steering demand      [-1, 1]  (-1=full left, +1=full right)

    Debug draw helpers:
        drawLine(line, width=0.08, color='rgba(...)')
            line: [(x1,y1),(x2,y2),...] or [x1,y1,x2,y2] in meters

    Composed objects:
        sensors  -- Sensors instance (same object as the runtime `sensors` argument)
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.min_speed = -12.0
        self.max_speed = 36.0
        self.length = 4.5
        self.width = 2.0
        self.wheelbase = 2.8
        self.min_accel = 0.0
        self.max_accel = 12.0
        self.min_steering_angle_deg = -40.0
        self.max_steering_angle_deg = 40.0
        self.min_steering_speed_deg_s = -220.0
        self.max_steering_speed_deg_s = 220.0
        self.steer_angle = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.model = "kinematic"
        self.control_mode = "auto"
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.dt = 1.0 / 60.0
        self.accel_force = 12.0
        self.brake_force = 18.0
        self.friction = 0.16
        self._did_simulate_step = False
        self._model_step_fn = None
        self._model_state = None
        sensor_cls = globals().get("Sensors")
        self.sensors = sensor_cls() if callable(sensor_cls) else None

    @staticmethod
    def _clamp(value, lo, hi):
        if value < lo:
            return lo
        if value > hi:
            return hi
        return value

    def apply_control(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle = self._clamp(float(throttle), 0.0, 1.0)
        self.brake = self._clamp(float(brake), 0.0, 1.0)
        self.steer = self._clamp(float(steer), -1.0, 1.0)

    def load_model(self, step_fn, state_obj=None, model_name=None):
        if step_fn is None or not callable(step_fn):
            raise ValueError("step_fn must be callable: step(state, dt)")
        if state_obj is None:
            class _State:
                pass
            state_obj = _State()
        self._model_step_fn = step_fn
        self._model_state = state_obj
        if model_name:
            self.model = str(model_name)

    def clear_model(self):
        self._model_step_fn = None
        self._model_state = None

    def has_model(self):
        return self._model_step_fn is not None and self._model_state is not None

    def export_car_state(self, dt, accel_force=12.0, brake_force=18.0, friction=0.16):
        s = self._model_state
        s.x = self.x
        s.y = self.y
        s.heading_deg = self.angle
        s.speed = self.speed
        s.vx = self.vx
        s.vy = self.vy
        s.length = self.length
        s.width = self.width
        s.wheelbase = self.wheelbase
        s.min_speed = self.min_speed
        s.max_speed = self.max_speed
        s.min_accel = self.min_accel
        s.max_accel = self.max_accel
        s.min_steering_angle_deg = self.min_steering_angle_deg
        s.max_steering_angle_deg = self.max_steering_angle_deg
        s.min_steer_angle_deg = self.min_steering_angle_deg
        s.max_steer_angle_deg = self.max_steering_angle_deg
        s.min_steering_speed_deg_s = self.min_steering_speed_deg_s
        s.max_steering_speed_deg_s = self.max_steering_speed_deg_s
        s.min_steer_speed_deg_s = self.min_steering_speed_deg_s
        s.max_steer_speed_deg_s = self.max_steering_speed_deg_s
        s.steer_angle_deg = self.steer_angle
        s.throttle_input = self.throttle
        s.brake_input = self.brake
        s.steer_input = self.steer
        s.accel_force = float(accel_force)
        s.brake_force = float(brake_force)
        s.friction = float(friction)
        s.dt = float(dt)

    def import_car_state(self):
        s = self._model_state
        for attr in ("x", "y", "heading_deg", "speed", "vx", "vy", "steer_angle_deg"):
            if not hasattr(s, attr):
                continue
            val = getattr(s, attr)
            if attr == "heading_deg":
                self.angle = float(val)
            elif attr == "steer_angle_deg":
                self.steer_angle = float(val)
            else:
                setattr(self, attr, float(val))
        if hasattr(s, "length"):
            self.length = max(0.5, float(s.length))
        if hasattr(s, "width"):
            self.width = max(0.5, float(s.width))
        if hasattr(s, "wheelbase"):
            self.wheelbase = max(0.2, float(s.wheelbase))
        if hasattr(s, "min_speed"):
            self.min_speed = float(s.min_speed)
        if hasattr(s, "max_speed"):
            self.max_speed = float(s.max_speed)
        if self.min_speed > self.max_speed:
            self.min_speed, self.max_speed = self.max_speed, self.min_speed
        self.speed = self._clamp(self.speed, self.min_speed, self.max_speed)

    def simulate_one_step(self, dt=None, accel_force=None, brake_force=None, friction=None):
        if not self.has_model():
            raise RuntimeError("No model loaded. Call load_model(step_fn, state_obj) first.")
        if dt is None:
            dt = self.dt
        dt = float(dt)
        if accel_force is None:
            accel_force = self.accel_force
        if brake_force is None:
            brake_force = self.brake_force
        if friction is None:
            friction = self.friction
        self.export_car_state(dt, accel_force, brake_force, friction)
        self._model_step_fn(self._model_state, dt)
        self.import_car_state()
        self.dt = dt
        self.accel_force = float(accel_force)
        self.brake_force = float(brake_force)
        self.friction = float(friction)
        self._did_simulate_step = True

    def log(self, msg):
        _cb_log(str(msg))

    def drawLine(self, line, width=0.08, color="rgba(125,249,255,.9)"):
        drawLine(line, width, color)
