# SteerPy author, 2026.

# simulate_one_step.py
# simulate_one_step(car, world_model) — called every frame
# Requires planner(...) from planner.py and controller(...) from controller.py
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
# Control API (write once per frame):
#   car.throttle = 0..1
#   car.brake    = 0..1
#   car.steer    = -1..1
#   car.simulate_one_step()  # optional: run model step immediately in Python
#
# Helpers:
#   car.log(msg), print(msg), drawLine(line, width, color)

def simulate_one_step(car, world_model):
    trajectory = planner(car, world_model)
    drawLine(trajectory, 0.08, "rgba(125,249,255,.9)")

    throttle, steer = controller(car, trajectory)
    car.throttle = throttle
    car.steer = steer
    car.simulate_one_step()

    if world_model.frame_id % 120 == 0 and trajectory:
        car.log(f"planner pts={len(trajectory)} throttle={throttle:.2f} steer={steer:.2f}")
