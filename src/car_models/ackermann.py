# SteerPy author, 2026.


import math

class State:
    def __init__(self):
        # Runtime fields are injected from simulator each frame.
        pass

def step(state, dt):
    TRACK_HALF = 0.8

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

    if abs(state.steer_angle_deg) > 0.1 and abs(state.speed) > 0.5:
        sa = math.radians(state.steer_angle_deg)
        tan_sa = math.tan(sa)
        tan_sa = math.copysign(max(abs(tan_sa), 1e-6), tan_sa)

        L = max(0.2, state.wheelbase)
        r_in = L / tan_sa - TRACK_HALF
        r_out = L / tan_sa + TRACK_HALF
        r_ctr = 0.5 * (r_in + r_out)

        d_heading = (state.speed / max(abs(r_ctr), 0.1))
        d_heading *= (1.0 if r_ctr >= 0 else -1.0)
        d_heading *= dt * (180.0 / math.pi)
        state.heading_deg -= d_heading

    rad = math.radians(state.heading_deg)
    state.vx = math.cos(rad) * state.speed
    state.vy = math.sin(rad) * state.speed
    state.x += state.vx * dt
    state.y += state.vy * dt
