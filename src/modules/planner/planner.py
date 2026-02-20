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

def planner(car, world_model=None):
    return  []
