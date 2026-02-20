# SteerPy author, 2026.


import math

class State:
    def __init__(self):
        # Runtime fields are injected from simulator each frame.
        pass

def step(state, dt):
    CF = 1.8
    CR = 1.3

    target = (state.max_steering_angle_deg * max(0.0, state.steer_input)
              + state.min_steering_angle_deg * max(0.0, -state.steer_input))
    if target >= state.steer_angle_deg:
        state.steer_angle_deg = min(target, state.steer_angle_deg + state.max_steering_speed_deg_s * dt)
    else:
        state.steer_angle_deg = max(target, state.steer_angle_deg + state.min_steering_speed_deg_s * dt)

    head = math.radians(state.heading_deg)
    ch = math.cos(head)
    sh = math.sin(head)
    v_long = ch * state.vx + sh * state.vy
    v_lat = -sh * state.vx + ch * state.vy

    accel = max(state.min_accel, min(state.max_accel, state.accel_force))
    drive = state.throttle_input * accel
    brake = state.brake_input * state.brake_force
    slip = math.atan2(v_lat, abs(v_long) + 0.05)
    lat_a = -CR * slip * max(2.0, abs(v_long))
    ax_body = (drive - brake)
    ay_body = lat_a
    ax = ch * ax_body - sh * ay_body
    ay = sh * ax_body + ch * ay_body

    sa = math.radians(state.steer_angle_deg)
    yaw_gain = min(1.0, abs(v_long) / 1.5)
    state.heading_deg -= (v_long / (state.wheelbase + 0.01)) * math.tan(sa) * CF * yaw_gain * dt * (180.0 / math.pi)

    state.vx += ax * dt
    state.vy += ay * dt
    damp = max(0.05, min(0.95, state.friction + 0.35))
    state.vx *= (1.0 - damp * dt)
    state.vy *= (1.0 - damp * dt)

    head_now = math.radians(state.heading_deg)
    v_long_now = math.cos(head_now) * state.vx + math.sin(head_now) * state.vy
    speed_mag = math.hypot(state.vx, state.vy)
    signed_speed = 0.0 if speed_mag < 1e-6 else math.copysign(speed_mag, v_long_now)
    state.speed = max(state.min_speed, min(state.max_speed, signed_speed))
    state.x += state.vx * dt
    state.y += state.vy * dt

    if abs(state.speed) < 0.8:
        r = math.radians(state.heading_deg)
        state.vx = math.cos(r) * state.speed
        state.vy = math.sin(r) * state.speed
