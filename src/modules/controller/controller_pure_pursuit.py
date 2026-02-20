# SteerPy author, 2026.

# controller.py
# controller(car, trajectory) -> [throttle, steer]
# steer convention: -1 left, +1 right
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
# Write controls in simulate_one_step.py:
#   car.throttle = 0..1
#   car.brake    = 0..1
#   car.steer    = -1..1
#
# Helpers:
#   car.log(msg), print(msg), drawLine(line, width, color)

import math

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def controller(car, trajectory):
    if not trajectory:
        return [0.0, 0.0]

    lookahead = clamp(2.0 + 0.35 * car.speed, 2.0, 10.0)
    target = trajectory[-1]
    for p in trajectory:
        if math.hypot(p[0] - car.x, p[1] - car.y) >= lookahead:
            target = p
            break

    tx, ty, target_speed = target
    dx = tx - car.x
    dy = ty - car.y
    hdg = math.radians(car.angle)

    x_local = math.cos(hdg) * dx + math.sin(hdg) * dy
    y_local = -math.sin(hdg) * dx + math.cos(hdg) * dy

    L = max(0.2, car.wheelbase)
    alpha = math.atan2(y_local, max(0.05, x_local))
    delta_left = math.atan2(2.0 * L * math.sin(alpha), max(0.5, lookahead))

    steer_limit = max(1.0, abs(car.min_steering_angle_deg), abs(car.max_steering_angle_deg))
    steer_cmd = clamp(-math.degrees(delta_left) / steer_limit, -1.0, 1.0)

    speed_err = target_speed - car.speed
    throttle = clamp(speed_err / 4.0, 0.0, 1.0)
    return [throttle, steer_cmd]
