# SteerPy 1.0 Simulator

SteerPy is a browser-based autonomous driving playground where you can write code, hit run, and watch your car make decisions in real time.

It provides:
- Python behavior editing (`planner.py`, `controller.py`, `simulate_one_step.py`)
- Configurable vehicle models (`car_models.py`)
- Vehicle limits (`car_config.py`)
- Scenario editing and random generation (`world_config.py`)
- Real-time visualization, sensors (including LiDAR), and console logs

## Motivation

Learning robotics can feel like a side quest in installation: ROS setup, environments, dependencies, and tooling before your first real experiment.

SteerPy skips that friction. Open the browser, write Python, and see the behavior immediately. No heavy robotics stack required.  
The mission is simple: make autonomous driving learning fun and practical, with a focus on planners and controllers.

## Source Layout

- `src/simulation_apis/`: simulation APIs (`simulate_one_step.py`, `random_world_generator.py`)
- `src/modules/`: behavior modules (`planner/planner.py`, `controller/controller.py`)
- `src/car_models/`: physics model scripts (`kinematic.py`, `bicycle.py`, `ackermann.py`, `drift.py`, `custom.py`)
- `src/sample_config/`: world/config samples (`world_config.py`, `car_config.py`)
- `src/core/`: shared simulator Python data types (`Car`, `Sensors`, `WorldModel`)
- `src/runtime/`: Pyodide runtime helper scripts loaded by `steerpy.html`

## Requirements

- A modern browser (Chrome/Edge/Firefox)
- Internet access (Pyodide and editor assets are loaded from CDN)
- A local static web server (recommended)

## Quick Start

1. Open a terminal in this project folder.
2. Start a local server:

```bash
python3 -m http.server 8000
```

3. Open:

```text
http://localhost:8000/steerpy.html
```

4. Wait for `Pyodide Python 3.11 ready` in the in-app console.
5. Start tweaking `planner.py` and `controller.py`, then watch the car respond.

## Python UI (Desktop)

You can run a similar UI in pure Python (`tkinter`):

```bash
python3 steerpy_py_ui.py
```

Features:
- No built-in code editor (edit files in your IDE).
- Auto-reload on save for:
  - `src/simulation_apis/simulate_one_step.py`
  - `src/modules/planner/planner.py`
  - `src/modules/controller/controller.py`
  - selected `src/car_models/*.py`
  - `src/sample_config/car_config.py`
  - `src/sample_config/world_config.py`
- Simulation panel with `Pause/Resume`, `1 Step`, `Restart`, and `Auto/Manual`.

## Main Workflow

1. Edit code in one of the Python tabs.
2. Apply changes:
   - `Run` in the active tab, or
   - `Ctrl+S` to save and apply active tab.
3. Watch the simulation and logs in the right panel.
4. Repeat fast: tweak, run, observe, improve.

## Tabs and What They Do

- `simulate_one_step.py`: main per-frame entry point (`simulate_one_step(car, world_model)`).
- `planner.py`: path/trajectory generation.
- `controller.py`: steering/throttle from trajectory.
- `car_models.py`: vehicle dynamics model (`step(state, dt)`), with presets.
- `car_config.py`: shared vehicle limits and dimensions.
- `world_config.py`: road, obstacles, and optional car initial state.

## Run / Apply Shortcuts

- `Ctrl+Enter`: apply `simulate_one_step.py` / `planner.py` / `controller.py`
- `Ctrl+Shift+Enter`: apply `car_models.py`
- `Ctrl+Alt+Shift+Enter`: apply `car_config.py`
- `Ctrl+Alt+Enter`: apply `world_config.py`
- `Ctrl+S` or `Cmd+S`: save + apply current active pane

## Controls (Simulation)

- `Space`: pause/resume
- `1 Step` button: when paused, advances exactly one simulation frame
- `Mode: Auto/Manual` button:
  - `Auto`: Python `simulate_one_step()` drives
  - `Manual`: arrow keys drive car
- `View: Free/Follow` button:
  - `Free`: camera can pan
  - `Follow`: camera tracks car
- Mouse wheel or `+` / `-`: zoom
- Hold left mouse button and drag: pan (in Free view)

## Exports

- `simulate_one_step.py` tab: `💾 Export simulate_one_step.py`
- `planner.py` / `controller.py` tabs: `💾 Export P+C`
- Exports download timestamped `.py` files.

## Console and Logging

- `car.log("message")` prints to simulator console.
- `print("message")` also goes to simulator console.
- Use the `Clear` button to clear logs.

## Python Data Available

### `car`

Common state and limits, including:
- pose/speed: `x`, `y`, `angle`, `speed`
- geometry: `length`, `width`, `wheelbase`
- limits: speed/accel/steering angle/steering speed bounds
- model state: `steer_angle`, `vx`, `vy`, `model`, `control_mode`

Control outputs to write each frame:
- `car.throttle` in `[0,1]`
- `car.brake` in `[0,1]`
- `car.steer` in `[-1,1]`

### `car.sensors`

- Directional distances (meters, `-1` means clear):
  - `front`, `rear`, `left`, `right`, `front_left`, `front_right`
- `lidar`: LiDAR beam list (`N` beams; runtime-configurable in World Settings via `LiDAR Range` and `LiDAR Beams`)

### `world_model`

- `frame_id`, `dt`
- `road_data`:
  - `width`, `waypoints`, `path`
- `loop`:
  - `True` for loop tracks
- `obstacles`:
  - list of obstacle dicts with `x`, `y`, `length`, `width`, `heading_deg`
- methods:
  - `set_runtime(...)`, `set_road_data(...)`, `set_obstacles(...)`, `set_scenario(...)`, `snapshot()`

## `world_config.py` Format

Example:

```python
waypoints = [
    (-15, -10),
    (15, -10),
    (15, 10),
    (-15, 10),
]
road_width = 9.0
loop = True

obstacles = [
    (6, 0, 6, 2.5, 25),   # (x, y, length, width, heading_deg)
]

car_init = [-15.0, -10.0, 0.0, 0.0, 0.0]  # optional
```

## Troubleshooting

- App stuck on loading:
  - Ensure internet is available (CDN scripts must load).
- `print()`/logs not visible:
  - Check the bottom console panel.
  - Ensure your `print` outputs include newline (normal `print(...)` does).
- Car not moving:
  - Ensure simulation is not paused.
  - Ensure control mode is `Auto` when testing Python behavior.
  - Click `Run` after editing code.
