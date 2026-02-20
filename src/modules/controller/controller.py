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

def controller(car, trajectory):
    if not trajectory:
        return [0.0, 0.0]

    throttle_cmd = 0.0
    steer_cmd = 0.0
    return [throttle_cmd, steer_cmd]
