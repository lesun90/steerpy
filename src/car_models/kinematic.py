# SteerPy author, 2026.

# Convention: steer_input = -1 left, +1 right.

import math

class State:
    def __init__(self):
        # Runtime fields are injected from simulator each frame.
        pass

def step(state, dt):
    target = (state.max_steering_angle_deg * max(0.0, state.steer_input)
              + state.min_steering_angle_deg * max(0.0, -state.steer_input))
    if target >= state.steer_angle_deg:
        state.steer_angle_deg = min(target, state.steer_angle_deg + state.max_steering_speed_deg_s * dt)
    else:
        state.steer_angle_deg = max(target, state.steer_angle_deg + state.min_steering_speed_deg_s * dt)

    accel = max(state.min_accel, min(state.max_accel, state.accel_force))
    state.speed += state.throttle_input * accel * dt
    state.speed -= state.brake_input * state.brake_force * dt
    state.speed *= (1.0 - state.friction * dt)
    state.speed = max(state.min_speed, min(state.max_speed, state.speed))

    yaw_rate = (state.speed / max(0.2, state.wheelbase)) * math.tan(-math.radians(state.steer_angle_deg))
    state.heading_deg += math.degrees(yaw_rate * dt)

    rad = math.radians(state.heading_deg)
    state.vx = math.cos(rad) * state.speed
    state.vy = math.sin(rad) * state.speed
    state.x += state.vx * dt
    state.y += state.vy * dt
