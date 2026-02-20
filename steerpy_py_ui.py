#!/usr/bin/env python3
# SteerPy author, 2026.

"""SteerPy Python UI (runtime viewer, no editors).

- Uses source files under src/ directly.
- Auto-reloads on file save (behavior/model/config/world).
- Provides simulation controls and 2D visualization.
"""

from __future__ import annotations

import importlib.util
import inspect
import math
import pathlib
import sys
import time
from dataclasses import dataclass

try:
    import tkinter as tk
    from tkinter import filedialog, ttk
except ModuleNotFoundError as exc:
    if exc.name == "tkinter":
        sys.stderr.write(
            "Error: tkinter is not installed for this Python interpreter.\n"
            "Install it, then rerun:\n"
            "  Ubuntu/Debian: sudo apt-get install python3-tk\n"
            "  Fedora:        sudo dnf install python3-tkinter\n"
            "  Arch:          sudo pacman -S tk\n"
            "  macOS (brew):  brew install python-tk\n"
        )
        raise SystemExit(1)
    raise

ROOT = pathlib.Path(__file__).resolve().parent

PATHS = {
    "simulate": ROOT / "src" / "simulation_apis" / "simulate_one_step.py",
    "planner": ROOT / "src" / "modules" / "planner" / "planner.py",
    "controller": ROOT / "src" / "modules" / "controller" / "controller.py",
    "config": ROOT / "src" / "sample_config" / "car_config.py",
    "world": ROOT / "src" / "sample_config" / "world_config.py",
}

MODEL_PATHS = {
    "kinematic": ROOT / "src" / "car_models" / "kinematic.py",
    "bicycle": ROOT / "src" / "car_models" / "bicycle.py",
    "ackermann": ROOT / "src" / "car_models" / "ackermann.py",
    "drift": ROOT / "src" / "car_models" / "drift.py",
    "custom": ROOT / "src" / "car_models" / "custom.py",
}


class UIError(Exception):
    pass


def load_module_from_path(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise UIError(f"Cannot import module from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sensors_mod = load_module_from_path("sl_sensors", ROOT / "src" / "core" / "sensors.py")
car_mod = load_module_from_path("sl_car", ROOT / "src" / "core" / "car.py")
wm_mod = load_module_from_path("sl_world", ROOT / "src" / "core" / "world_model.py")
setattr(car_mod, "Sensors", sensors_mod.Sensors)


@dataclass
class CarState:
    accel_force: float = 12.0
    brake_force: float = 18.0
    friction: float = 0.16


class SteerPyPyUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SteerPy 1.0 (Python Runtime UI)")
        self.root.geometry("1300x860")

        self.car = car_mod.Car()
        self.world_model = wm_mod.WorldModel()
        self.car_state = CarState()

        self.frame_id = 0
        self.last_tick = time.perf_counter()
        self.paused = False
        self.step_once = False
        self.time_scale = 1.0
        self.control_mode = "auto"
        self.manual_keys = {"up": False, "down": False, "left": False, "right": False}

        self.logs: list[str] = []
        self.draw_lines: list[dict] = []
        self.track_path: list[tuple[float, float]] = []

        self.model_name = "kinematic"
        self.simulate_fn = None
        self.behavior_ns = {}
        self.behavior_overrides: dict[str, str | None] = {
            "planner": None,
            "controller": None,
            "simulate": None,
        }
        self.model_overrides: dict[str, str | None] = {name: None for name in MODEL_PATHS}
        self.config_override: str | None = None
        self.world_override: str | None = None

        self.file_mtimes: dict[pathlib.Path, int] = {}
        self.reload_cooldown = 0.0

        self.status_var = tk.StringVar(value="Watching source files...")

        self._build_ui()
        self._bind_keys()
        self._install_car_callbacks()
        self._force_reload_all()
        self._tick()

    def _build_ui(self):
        root_frame = ttk.Frame(self.root)
        root_frame.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(root_frame)
        controls.pack(fill=tk.X, padx=8, pady=8)

        ttk.Button(controls, text="Pause/Resume", command=self._toggle_pause).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(controls, text="1 Step", command=self._step_once).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(controls, text="Restart", command=self._restart).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(controls, text="Mode Auto/Manual", command=self._toggle_mode).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(controls, text="Reload Now", command=self._force_reload_all).grid(row=0, column=4, padx=4, pady=4)
        ttk.Button(controls, text="Load Planner", command=lambda: self._load_behavior_override("planner")).grid(row=0, column=5, padx=4, pady=4)
        ttk.Button(controls, text="Load Controller", command=lambda: self._load_behavior_override("controller")).grid(row=0, column=6, padx=4, pady=4)
        ttk.Button(controls, text="Load SimStep", command=lambda: self._load_behavior_override("simulate")).grid(row=0, column=7, padx=4, pady=4)
        ttk.Button(controls, text="Load Car Model", command=self._load_model_override).grid(row=0, column=8, padx=4, pady=4)
        ttk.Button(controls, text="Load Car Config", command=self._load_config_override).grid(row=0, column=9, padx=4, pady=4)
        ttk.Button(controls, text="Load World Config", command=self._load_world_override).grid(row=0, column=10, padx=4, pady=4)

        ttk.Label(controls, text="Model").grid(row=1, column=0, sticky="w", padx=4)
        self.model_var = tk.StringVar(value=self.model_name)
        model_combo = ttk.Combobox(controls, width=14, state="readonly", textvariable=self.model_var)
        model_combo["values"] = tuple(MODEL_PATHS.keys())
        model_combo.grid(row=1, column=1, sticky="w", padx=4)
        model_combo.bind("<<ComboboxSelected>>", self._on_model_change)

        ttk.Label(controls, text="Time Scale").grid(row=1, column=2, sticky="w", padx=4)
        self.time_scale_var = tk.DoubleVar(value=1.0)
        ttk.Scale(controls, from_=0.2, to=2.5, variable=self.time_scale_var, command=self._on_time_scale).grid(
            row=1, column=3, columnspan=2, sticky="ew", padx=4
        )

        controls.columnconfigure(10, weight=1)

        self.canvas = tk.Canvas(root_frame, bg="#101419", highlightthickness=1, highlightbackground="#2b3642")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.log_box = tk.Text(root_frame, height=10, wrap=tk.WORD)
        self.log_box.pack(fill=tk.X, padx=8, pady=(0, 8))

        status = ttk.Label(root_frame, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, padx=8, pady=(0, 8))

    def _bind_keys(self):
        self.root.bind("<KeyPress-Up>", lambda _: self._set_key("up", True))
        self.root.bind("<KeyRelease-Up>", lambda _: self._set_key("up", False))
        self.root.bind("<KeyPress-Down>", lambda _: self._set_key("down", True))
        self.root.bind("<KeyRelease-Down>", lambda _: self._set_key("down", False))
        self.root.bind("<KeyPress-Left>", lambda _: self._set_key("left", True))
        self.root.bind("<KeyRelease-Left>", lambda _: self._set_key("left", False))
        self.root.bind("<KeyPress-Right>", lambda _: self._set_key("right", True))
        self.root.bind("<KeyRelease-Right>", lambda _: self._set_key("right", False))

    def _install_car_callbacks(self):
        def _cb_log(msg):
            self.log(f"[py] {msg}")

        def _draw_line(line, width=0.08, color="rgba(125,249,255,.9)"):
            self._append_draw_line(line, width, color)

        setattr(car_mod, "_cb_log", _cb_log)
        setattr(car_mod, "drawLine", _draw_line)

    def _set_key(self, key: str, val: bool):
        self.manual_keys[key] = val

    def _on_time_scale(self, _=None):
        self.time_scale = float(self.time_scale_var.get())

    def _toggle_pause(self):
        self.paused = not self.paused

    def _step_once(self):
        self.step_once = True
        self.paused = True

    def _toggle_mode(self):
        self.control_mode = "manual" if self.control_mode == "auto" else "auto"
        self.log(f"mode={self.control_mode}")

    def _on_model_change(self, _=None):
        self.model_name = self.model_var.get()
        self._reload_model(force=True)

    def _load_model_override(self):
        path = filedialog.askopenfilename(
            title=f"Load car model override for {self.model_name}",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt"), ("All files", "*")],
        )
        if not path:
            return
        text = pathlib.Path(path).read_text(encoding="utf-8")
        self.model_overrides[self.model_name] = text
        self._reload_model(force=True)
        self.log(f"override loaded for model={self.model_name}: {path}")

    def _load_config_override(self):
        path = filedialog.askopenfilename(
            title="Load car_config override",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt"), ("All files", "*")],
        )
        if not path:
            return
        self.config_override = pathlib.Path(path).read_text(encoding="utf-8")
        self._reload_config(force=True)
        self.log(f"override loaded for car_config: {path}")

    def _load_world_override(self):
        path = filedialog.askopenfilename(
            title="Load world_config override",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt"), ("All files", "*")],
        )
        if not path:
            return
        self.world_override = pathlib.Path(path).read_text(encoding="utf-8")
        self._reload_world(force=True)
        self.log(f"override loaded for world_config: {path}")

    def _restart(self):
        self.frame_id = 0
        self.car.x = 0.0
        self.car.y = 0.0
        self.car.angle = 0.0
        self.car.speed = 0.0
        self.car.vx = 0.0
        self.car.vy = 0.0
        self.car.steer_angle = 0.0

    def _read_file(self, path: pathlib.Path) -> str:
        return path.read_text(encoding="utf-8")

    def _load_behavior_override(self, key: str):
        path = filedialog.askopenfilename(
            title=f"Load {key}.py override",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt"), ("All files", "*")],
        )
        if not path:
            return
        text = pathlib.Path(path).read_text(encoding="utf-8")
        self.behavior_overrides[key] = text
        self._reload_behavior(force=True)
        self.log(f"override loaded for {key}: {path}")

    def _get_mtime_ns(self, path: pathlib.Path) -> int:
        return path.stat().st_mtime_ns

    def _was_changed(self, path: pathlib.Path) -> bool:
        m = self._get_mtime_ns(path)
        old = self.file_mtimes.get(path)
        if old is None or m != old:
            self.file_mtimes[path] = m
            return True
        return False

    def _force_reload_all(self):
        self._reload_behavior(force=True)
        self._reload_config(force=True)
        self._reload_world(force=True)
        self._reload_model(force=True)

    def _reload_if_needed(self):
        now = time.perf_counter()
        if now < self.reload_cooldown:
            return

        try:
            changed = False
            if self._reload_behavior(force=False):
                changed = True
            if self._reload_config(force=False):
                changed = True
            if self._reload_world(force=False):
                changed = True
            if self._reload_model(force=False):
                changed = True
            if changed:
                self.status_var.set("Auto-reloaded source files")
        except Exception as exc:
            self.log(f"reload error: {exc}")
            self.status_var.set("Reload error")
            self.reload_cooldown = time.perf_counter() + 0.5

    def _reload_behavior(self, force: bool) -> bool:
        p_planner = PATHS["planner"]
        p_controller = PATHS["controller"]
        p_sim = PATHS["simulate"]

        changed = force
        if self.behavior_overrides["planner"] is None:
            changed = changed or self._was_changed(p_planner)
        if self.behavior_overrides["controller"] is None:
            changed = changed or self._was_changed(p_controller)
        if self.behavior_overrides["simulate"] is None:
            changed = changed or self._was_changed(p_sim)
        if not changed:
            return False

        ns = {"drawLine": self._behavior_draw_line}
        planner_src = self.behavior_overrides["planner"] if self.behavior_overrides["planner"] is not None else self._read_file(p_planner)
        controller_src = self.behavior_overrides["controller"] if self.behavior_overrides["controller"] is not None else self._read_file(p_controller)
        sim_src = self.behavior_overrides["simulate"] if self.behavior_overrides["simulate"] is not None else self._read_file(p_sim)
        exec(planner_src, ns, ns)
        exec(controller_src, ns, ns)
        exec(sim_src, ns, ns)

        fn = ns.get("simulate_one_step")
        if not callable(fn):
            raise UIError("simulate_one_step(...) not found")

        self.behavior_ns = ns
        self.simulate_fn = fn
        self.log("reloaded behavior")
        return True

    def _reload_config(self, force: bool) -> bool:
        p = PATHS["config"]
        if self.config_override is None and not (force or self._was_changed(p)):
            return False

        ns = {}
        src = self.config_override if self.config_override is not None else self._read_file(p)
        exec(src, ns, ns)
        cfg = ns.get("car_config")
        if not isinstance(cfg, dict):
            raise UIError("car_config dict not found")

        for k in (
            "length", "width", "wheelbase", "min_speed", "max_speed",
            "min_accel", "max_accel", "min_steering_angle_deg", "max_steering_angle_deg",
            "min_steering_speed_deg_s", "max_steering_speed_deg_s",
        ):
            if k in cfg:
                setattr(self.car, k, float(cfg[k]))

        self.log("reloaded car_config")
        return True

    def _reload_world(self, force: bool) -> bool:
        p = PATHS["world"]
        if self.world_override is None and not (force or self._was_changed(p)):
            return False

        ns = {}
        src = self.world_override if self.world_override is not None else self._read_file(p)
        exec(src, ns, ns)

        waypoints = ns.get("waypoints", [])
        obstacles = ns.get("obstacles", [])
        road_width = float(ns.get("road_width", 9.0))
        sample_distance = float(ns.get("sample_distance", 1.0))
        loop_flag = bool(ns.get("loop", True))

        road_data = {
            "width": road_width,
            "sample_distance": max(1e-3, sample_distance),
            "waypoints": waypoints,
            "path": waypoints,
        }
        self.world_model.set_scenario(road_data, self._normalize_obstacles(obstacles), loop_flag)
        self.track_path = self.world_model.road_data.get("path", [])

        car_init = ns.get("car_init")
        if isinstance(car_init, (list, tuple)) and len(car_init) >= 3:
            self.car.x = float(car_init[0])
            self.car.y = float(car_init[1])
            self.car.angle = float(car_init[2])
            if len(car_init) >= 4:
                self.car.speed = float(car_init[3])
            if len(car_init) >= 5:
                self.car.steer_angle = float(car_init[4])

        self.log("reloaded world_config")
        return True

    def _reload_model(self, force: bool) -> bool:
        p = MODEL_PATHS[self.model_name]
        override_src = self.model_overrides.get(self.model_name)
        if override_src is None and not (force or self._was_changed(p)):
            return False

        ns = {}
        src = override_src if override_src is not None else self._read_file(p)
        exec(src, ns, ns)
        step_fn = ns.get("step")
        if not callable(step_fn):
            raise UIError("model step(state, dt) not found")
        state_cls = ns.get("State")
        state_obj = state_cls() if callable(state_cls) else type("State", (), {})()
        self.car.load_model(step_fn, state_obj, self.model_name)
        self.log(f"reloaded model={self.model_name}")
        return True

    def _behavior_draw_line(self, line, width=0.08, color="rgba(225,249,255,.9)"):
        self._append_draw_line(line, width, color)

    def _append_draw_line(self, line, width=0.08, color="rgba(125,249,255,.9)"):
        points = self._normalize_line_points(line)
        if len(points) < 2:
            return
        try:
            canvas_width = max(1, int(round(float(width) * 8.0)))
        except Exception:
            canvas_width = 1
        self.draw_lines.append(
            {
                "points": points,
                "width": canvas_width,
                "color": self._normalize_color(color),
            }
        )

    @staticmethod
    def _normalize_line_points(line):
        def coerce_point(v):
            if isinstance(v, (list, tuple)) and len(v) >= 2:
                try:
                    return (float(v[0]), float(v[1]))
                except Exception:
                    return None
            if isinstance(v, dict):
                if "x" in v and "y" in v:
                    try:
                        return (float(v["x"]), float(v["y"]))
                    except Exception:
                        return None
                if "xy" in v and isinstance(v["xy"], (list, tuple)) and len(v["xy"]) >= 2:
                    try:
                        return (float(v["xy"][0]), float(v["xy"][1]))
                    except Exception:
                        return None
            return None

        if isinstance(line, (list, tuple)) and len(line) == 4 and all(isinstance(x, (int, float)) for x in line):
            return [(float(line[0]), float(line[1])), (float(line[2]), float(line[3]))]

        if isinstance(line, dict):
            if all(k in line for k in ("x1", "y1", "x2", "y2")):
                try:
                    return [
                        (float(line["x1"]), float(line["y1"])),
                        (float(line["x2"]), float(line["y2"])),
                    ]
                except Exception:
                    return []
            line = line.get("points", line.get("line", []))

        out = []
        for p in (line or []):
            cp = coerce_point(p)
            if cp is not None:
                out.append(cp)
        return out

    @staticmethod
    def _normalize_color(color):
        c = str(color or "").strip()
        if not c:
            return "#66d9ef"
        if c.startswith("rgba(") and c.endswith(")"):
            vals = [v.strip() for v in c[5:-1].split(",")]
            if len(vals) >= 3:
                try:
                    r = max(0, min(255, int(float(vals[0]))))
                    g = max(0, min(255, int(float(vals[1]))))
                    b = max(0, min(255, int(float(vals[2]))))
                    return f"#{r:02x}{g:02x}{b:02x}"
                except Exception:
                    pass
        return c

    @staticmethod
    def _normalize_obstacles(obstacles):
        out = []
        for ob in obstacles or []:
            if isinstance(ob, dict):
                out.append(ob)
            elif isinstance(ob, (list, tuple)) and len(ob) >= 5:
                out.append({
                    "x": float(ob[0]),
                    "y": float(ob[1]),
                    "length": float(ob[2]),
                    "width": float(ob[3]),
                    "heading_deg": float(ob[4]),
                })
        return out

    def _tick(self):
        now = time.perf_counter()
        dt = min(0.05, now - self.last_tick)
        self.last_tick = now

        self._reload_if_needed()

        should_step = (not self.paused) or self.step_once
        if should_step:
            self.step_once = False
            self._sim_step(dt * self.time_scale)

        self._draw_canvas()
        self.root.after(16, self._tick)

    def _sim_step(self, dt: float):
        self.frame_id += 1
        self.world_model.set_runtime(
            self.frame_id,
            dt,
            road_width=self.world_model.road_data.get("width", 9.0),
            road_path=self.track_path,
            loop_flag=self.world_model.loop,
        )

        self._update_sensors_stub()
        self.draw_lines.clear()

        if self.control_mode == "manual":
            self.car.throttle = 1.0 if self.manual_keys["up"] and not self.manual_keys["down"] else 0.0
            self.car.brake = 1.0 if self.manual_keys["down"] else 0.0
            steer = 0.0
            if self.manual_keys["left"]:
                steer -= 1.0
            if self.manual_keys["right"]:
                steer += 1.0
            self.car.steer = steer
        else:
            self.car.throttle = 0.0
            self.car.brake = 0.0
            self.car.steer = 0.0
            self._run_behavior()

        if self.control_mode == "manual" or not self.car._did_simulate_step:
            self.car.simulate_one_step(dt, self.car_state.accel_force, self.car_state.brake_force, self.car_state.friction)
        self.car._did_simulate_step = False

    def _run_behavior(self):
        fn = self.simulate_fn
        if not callable(fn):
            return

        try:
            params = [
                p
                for p in inspect.signature(fn).parameters.values()
                if p.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
            ]
            if len(params) == 2:
                fn(self.car, self.world_model)
            elif len(params) == 3:
                fn(self.car, self.car.sensors, self.world_model)
            else:
                raise UIError("simulate_one_step signature must be (car, world_model) or (car, sensors, world_model)")
        except Exception as exc:
            self.log(f"simulate_one_step() error: {exc}")

    def _update_sensors_stub(self):
        s = self.car.sensors
        if s is None:
            return
        s.front = s.rear = s.left = s.right = -1.0
        s.front_left = s.front_right = -1.0
        s.lidar = [-1.0] * 36

    def _draw_canvas(self):
        c = self.canvas
        c.delete("all")

        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        scale = 7.0

        def to_px(x: float, y: float):
            return w * 0.5 + x * scale, h * 0.5 - y * scale

        def build_road_edges(path_pts, road_width):
            if len(path_pts) < 2:
                return [], []
            hw = max(0.1, float(road_width) * 0.5)
            left = []
            right = []
            n = len(path_pts)
            for i in range(n):
                x, y = path_pts[i]
                if i == 0:
                    x0, y0 = path_pts[i]
                    x1, y1 = path_pts[i + 1]
                elif i == n - 1:
                    x0, y0 = path_pts[i - 1]
                    x1, y1 = path_pts[i]
                else:
                    x0, y0 = path_pts[i - 1]
                    x1, y1 = path_pts[i + 1]
                dx = x1 - x0
                dy = y1 - y0
                norm = math.hypot(dx, dy)
                if norm < 1e-9:
                    nx, ny = 0.0, 1.0
                else:
                    nx, ny = -dy / norm, dx / norm
                left.append((x + nx * hw, y + ny * hw))
                right.append((x - nx * hw, y - ny * hw))
            return left, right

        road_width = float(self.world_model.road_data.get("width", 9.0))
        left_edge, right_edge = build_road_edges(self.track_path, road_width)

        # Road boundaries
        if len(left_edge) >= 2:
            for i in range(len(left_edge) - 1):
                p1 = to_px(*left_edge[i])
                p2 = to_px(*left_edge[i + 1])
                c.create_line(*p1, *p2, fill="#4b5563", width=2)
        if len(right_edge) >= 2:
            for i in range(len(right_edge) - 1):
                p1 = to_px(*right_edge[i])
                p2 = to_px(*right_edge[i + 1])
                c.create_line(*p1, *p2, fill="#4b5563", width=2)

        # Center line
        for i in range(len(self.track_path) - 1):
            x1, y1 = self.track_path[i]
            x2, y2 = self.track_path[i + 1]
            p1 = to_px(x1, y1)
            p2 = to_px(x2, y2)
            c.create_line(*p1, *p2, fill="#7c8793", width=1, dash=(4, 4))

        for ob in self.world_model.obstacles:
            x = float(ob.get("x", 0.0))
            y = float(ob.get("y", 0.0))
            l = float(ob.get("length", 2.0))
            ww = float(ob.get("width", 1.0))
            p1 = to_px(x - l * 0.5, y - ww * 0.5)
            p2 = to_px(x + l * 0.5, y + ww * 0.5)
            c.create_rectangle(*p1, *p2, outline="#eab308", fill="")

        for line in self.draw_lines:
            pts_src = line.get("points", [])
            if len(pts_src) < 2:
                continue
            pts = []
            for x, y in pts_src:
                pts.extend(to_px(x, y))
            c.create_line(*pts, fill=line.get("color", "#66d9ef"), width=line.get("width", 1))

        cx, cy = to_px(self.car.x, self.car.y)
        ang = math.radians(self.car.angle)
        dx = math.cos(ang) * self.car.length * 0.5 * scale
        dy = -math.sin(ang) * self.car.length * 0.5 * scale
        wx = math.sin(ang) * self.car.width * 0.5 * scale
        wy = math.cos(ang) * self.car.width * 0.5 * scale

        p1 = (cx - dx - wx, cy - dy - wy)
        p2 = (cx + dx - wx, cy + dy - wy)
        p3 = (cx + dx + wx, cy + dy + wy)
        p4 = (cx - dx + wx, cy - dy + wy)
        c.create_polygon(*p1, *p2, *p3, *p4, outline="#34d399", fill="", width=2)

        def draw_wheel(center_x, center_y, heading_rad, wheel_len, wheel_wid, color="#e5e7eb"):
            lx = math.cos(heading_rad) * wheel_len * 0.5
            ly = -math.sin(heading_rad) * wheel_len * 0.5
            wxv = math.sin(heading_rad) * wheel_wid * 0.5
            wyv = math.cos(heading_rad) * wheel_wid * 0.5
            q1 = (center_x - lx - wxv, center_y - ly - wyv)
            q2 = (center_x + lx - wxv, center_y + ly - wyv)
            q3 = (center_x + lx + wxv, center_y + ly + wyv)
            q4 = (center_x - lx + wxv, center_y - ly + wyv)
            c.create_polygon(*q1, *q2, *q3, *q4, outline=color, fill="", width=2)

        # Wheel visualization
        half_track_px = self.car.width * 0.35 * scale
        axle_half_px = self.car.wheelbase * 0.5 * scale
        wheel_len_px = max(8.0, self.car.length * 0.22 * scale)
        wheel_wid_px = max(4.0, self.car.width * 0.12 * scale)
        # unit vectors for body frame in canvas coords
        fx, fy = math.cos(ang), -math.sin(ang)  # forward
        lx, ly = math.sin(ang), math.cos(ang)   # left

        rear_axle_x = cx - fx * axle_half_px
        rear_axle_y = cy - fy * axle_half_px
        front_axle_x = cx + fx * axle_half_px
        front_axle_y = cy + fy * axle_half_px

        rl_x = rear_axle_x + lx * half_track_px
        rl_y = rear_axle_y + ly * half_track_px
        rr_x = rear_axle_x - lx * half_track_px
        rr_y = rear_axle_y - ly * half_track_px
        fl_x = front_axle_x + lx * half_track_px
        fl_y = front_axle_y + ly * half_track_px
        fr_x = front_axle_x - lx * half_track_px
        fr_y = front_axle_y - ly * half_track_px

        steer_heading = math.radians(self.car.angle - self.car.steer_angle)
        draw_wheel(rl_x, rl_y, ang, wheel_len_px, wheel_wid_px, color="#94a3b8")
        draw_wheel(rr_x, rr_y, ang, wheel_len_px, wheel_wid_px, color="#94a3b8")
        draw_wheel(fl_x, fl_y, steer_heading, wheel_len_px, wheel_wid_px, color="#fbbf24")
        draw_wheel(fr_x, fr_y, steer_heading, wheel_len_px, wheel_wid_px, color="#fbbf24")

        # Heading indicator (green)
        hx = cx + math.cos(ang) * self.car.length * 0.8 * scale
        hy = cy - math.sin(ang) * self.car.length * 0.8 * scale
        c.create_line(cx, cy, hx, hy, fill="#34d399", width=2)

        # Steering visualization: front axle direction (amber)
        front_cx = cx + math.cos(ang) * self.car.wheelbase * 0.5 * scale
        front_cy = cy - math.sin(ang) * self.car.wheelbase * 0.5 * scale
        steer_rad = math.radians(self.car.angle - self.car.steer_angle)
        sx = front_cx + math.cos(steer_rad) * self.car.length * 0.45 * scale
        sy = front_cy - math.sin(steer_rad) * self.car.length * 0.45 * scale
        c.create_line(front_cx, front_cy, sx, sy, fill="#f59e0b", width=3)
        c.create_oval(front_cx - 2, front_cy - 2, front_cx + 2, front_cy + 2, outline="#f59e0b")

        c.create_text(
            12,
            12,
            text=f"mode={self.control_mode} paused={self.paused} frame={self.frame_id} model={self.model_name}",
            anchor="nw",
            fill="#e5e7eb",
        )
        c.create_text(
            12,
            30,
            text=f"speed={self.car.speed:.2f} m/s steer_angle={self.car.steer_angle:.1f} deg road_w={road_width:.1f} m",
            anchor="nw",
            fill="#e5e7eb",
        )

    def log(self, msg: str):
        line = str(msg)
        self.logs.append(line)
        if len(self.logs) > 400:
            self.logs = self.logs[-400:]
        self.log_box.delete("1.0", tk.END)
        self.log_box.insert("1.0", "\n".join(reversed(self.logs[-120:])))


def main():
    app = tk.Tk()
    SteerPyPyUI(app)
    app.mainloop()


if __name__ == "__main__":
    main()
