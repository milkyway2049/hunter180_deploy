#!/usr/bin/env python3
"""Tk slider utility for JointState command topic debugging."""

import math
import os
import shlex
import sys
import threading
from dataclasses import dataclass, field
from pathlib import Path
from time import monotonic
from typing import Any, Dict, List, Optional, Sequence, Tuple


DEFAULT_LIMIT_MIN = -3.14
DEFAULT_LIMIT_MAX = 3.14
PUBLISH_PERIOD_SEC = 0.02
UI_POLL_PERIOD_MS = 100
FEEDBACK_TIMEOUT_SEC = 0.5

MODE_HOLD = 2
MODE_JOINT = 4
MODE_QUERY = 6

DEFAULT_FSM_SERVICE_TOPIC = "/ec_slave_arm/fsm_service"
JOINT_STATE_TOPIC_TYPES = {"sensor_msgs/msg/JointState", "sensor_msgs/JointState"}

ROS_ENV_BOOTSTRAP_FLAG = "JOINT_HARDWARE_JOINT_SLIDER_ROS_ENV_BOOTSTRAPPED"


class StartupError(RuntimeError):
    pass


def find_workspace_setup(start_path: Optional[Path] = None) -> Optional[Path]:
    path = Path(start_path or __file__).resolve()
    if path.is_file():
        path = path.parent

    for directory in (path, *path.parents):
        setup_path = directory / "install" / "setup.bash"
        if setup_path.is_file():
            return setup_path
    return None


def build_missing_ros_dependency_message(module_name: str) -> str:
    setup_path = find_workspace_setup()
    script_path = Path(__file__).resolve()
    lines = [
        f"Missing ROS Python module: {module_name}",
        "This script must run in a shell with the robot_ws install overlay sourced.",
    ]

    if setup_path is not None:
        lines.extend(
            [
                "Try:",
                f"  source {setup_path}",
                f"  python3 {script_path}",
            ]
        )
    else:
        lines.extend(
            [
                "From the robot_ws root, build and source the workspace first:",
                "  colcon build --packages-up-to robot_interfaces ec_slave_arm_controller ec_joint_hardware",
                "  source install/setup.bash",
                f"  python3 {script_path}",
            ]
        )

    return "\n".join(lines)


def maybe_reexec_with_workspace_setup() -> bool:
    if os.environ.get(ROS_ENV_BOOTSTRAP_FLAG) == "1":
        return False

    setup_path = find_workspace_setup()
    if setup_path is None:
        return False

    env = os.environ.copy()
    env[ROS_ENV_BOOTSTRAP_FLAG] = "1"
    script_path = Path(__file__).resolve()
    python_args = [sys.executable, str(script_path), *sys.argv[1:]]
    command = "source " + shlex.quote(str(setup_path)) + " && exec "
    command += " ".join(shlex.quote(arg) for arg in python_args)

    try:
        os.execvpe("bash", ["bash", "-lc", command], env)
    except OSError:
        return False
    return True


def import_rclpy_module() -> Any:
    try:
        import rclpy
    except ModuleNotFoundError as exc:
        maybe_reexec_with_workspace_setup()
        module_name = exc.name or "rclpy"
        raise StartupError(build_missing_ros_dependency_message(module_name)) from exc
    return rclpy


def is_joint_state_topic_type(topic_types: Sequence[str]) -> bool:
    return any(topic_type in JOINT_STATE_TOPIC_TYPES for topic_type in topic_types)


def clamp_value(value: float, minimum: float, maximum: float) -> float:
    return min(max(float(value), minimum), maximum)


def format_float(value: float) -> str:
    return f"{value:.4f}"


def format_limit(value: float) -> str:
    text = f"{value:.12g}"
    return "0" if text == "-0" else text


def parse_limit_pair(min_text: str, max_text: str) -> Tuple[float, float]:
    try:
        minimum = float(min_text)
        maximum = float(max_text)
    except ValueError as exc:
        raise ValueError("limits must be numeric") from exc

    if not math.isfinite(minimum) or not math.isfinite(maximum):
        raise ValueError("limits must be finite")
    if minimum >= maximum:
        raise ValueError("min must be less than max")
    return minimum, maximum


def _finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def extract_valid_positions(
    positions: Sequence[float],
    names: Sequence[str],
    expected_names: Sequence[str],
) -> Optional[List[float]]:
    position_list = list(positions)
    name_list = list(names)
    expected = tuple(expected_names)
    if not name_list or len(name_list) != len(position_list) or not expected:
        return None

    position_by_name = dict(zip(name_list, position_list))
    ordered_positions = []
    for joint_name in expected:
        if joint_name not in position_by_name:
            return None
        value = _finite_float(position_by_name[joint_name])
        if value is None:
            return None
        ordered_positions.append(value)
    return ordered_positions


@dataclass
class JointTopicRuntimeState:
    label: str
    state_topic: str
    command_topic: str
    joint_names: Tuple[str, ...] = ()
    feedback: List[float] = field(default_factory=list)
    target: List[float] = field(default_factory=list)
    target_overridden: List[bool] = field(default_factory=list)
    limit_min: List[float] = field(default_factory=list)
    limit_max: List[float] = field(default_factory=list)
    feedback_ready: bool = False
    target_initialized: bool = False
    last_feedback_monotonic: Optional[float] = None
    target_version: int = 0
    limit_version: int = 0

    def __post_init__(self) -> None:
        initial_joint_names = tuple(self.joint_names)
        self.joint_names = ()
        if initial_joint_names:
            self.configure_joints(initial_joint_names)

    def configure_joints(self, joint_names: Sequence[str]) -> bool:
        names = tuple(joint_names)
        if not names:
            return False
        if names == self.joint_names:
            return False

        self.joint_names = names
        count = len(names)
        self.feedback = [math.nan] * count
        self.target = []
        self.target_overridden = [False] * count
        self.limit_min = [DEFAULT_LIMIT_MIN] * count
        self.limit_max = [DEFAULT_LIMIT_MAX] * count
        self.feedback_ready = False
        self.target_initialized = False
        self.last_feedback_monotonic = None
        self.target_version += 1
        self.limit_version += 1
        return True

    def update_feedback(
        self,
        positions: Sequence[float],
        monotonic_time: Optional[float] = None,
    ) -> None:
        values = [_finite_float(value) for value in positions]
        if len(values) != len(self.joint_names) or any(value is None for value in values):
            self.invalidate_feedback()
            return

        self.feedback = [float(value) for value in values if value is not None]
        self.feedback_ready = True
        self.last_feedback_monotonic = monotonic() if monotonic_time is None else monotonic_time

        self._expand_limits_to_include(self.feedback)
        if not self.target_initialized:
            self.target = list(self.feedback)
            self.target_overridden = [False] * len(self.joint_names)
            self.target_initialized = True
            self.target_version += 1
            return

        refreshed_target = list(self.target)
        for index, feedback in enumerate(self.feedback):
            if not self.target_overridden[index]:
                refreshed_target[index] = feedback
        if refreshed_target != self.target:
            self.target = refreshed_target
            self.target_version += 1

    def invalidate_feedback(self) -> None:
        count = len(self.joint_names)
        self.feedback = [math.nan] * count
        self.feedback_ready = False
        self.last_feedback_monotonic = None
        self.target = []
        self.target_overridden = [False] * count
        if self.target_initialized:
            self.target_version += 1
        self.target_initialized = False

    def expire_stale_feedback(self, monotonic_time: Optional[float] = None) -> bool:
        if not self.feedback_ready or self.last_feedback_monotonic is None:
            return False

        current_time = monotonic() if monotonic_time is None else monotonic_time
        if current_time - self.last_feedback_monotonic < FEEDBACK_TIMEOUT_SEC:
            return False

        self.invalidate_feedback()
        return True

    def set_target(self, index: int, value: float) -> None:
        if not self.feedback_ready or not self.target_initialized:
            return
        target = clamp_value(value, self.limit_min[index], self.limit_max[index])
        if self.target[index] != target or not self.target_overridden[index]:
            self.target[index] = target
            self.target_overridden[index] = True
            self.target_version += 1

    def reset_target_to_feedback(self) -> bool:
        if not self.feedback_ready:
            return False
        self._expand_limits_to_include(self.feedback)
        self.target = list(self.feedback)
        self.target_overridden = [False] * len(self.joint_names)
        self.target_initialized = True
        self.target_version += 1
        return True

    def build_command_target(self, monotonic_time: Optional[float] = None) -> Optional[List[float]]:
        self.expire_stale_feedback(monotonic_time)
        if not self.feedback_ready or not self.target_initialized or len(self.feedback) != len(self.joint_names):
            return None
        return [
            self.target[index] if self.target_overridden[index] else self.feedback[index]
            for index in range(len(self.joint_names))
        ]

    def apply_limits(self, minimums: Sequence[float], maximums: Sequence[float]) -> None:
        self.limit_min = list(minimums)
        self.limit_max = list(maximums)
        self.limit_version += 1

        if self.target_initialized:
            clamped = [
                clamp_value(value, self.limit_min[index], self.limit_max[index])
                for index, value in enumerate(self.target)
            ]
            if clamped != self.target:
                self.target = clamped
                self.target_version += 1

    def snapshot(self) -> dict:
        return {
            "label": self.label,
            "state_topic": self.state_topic,
            "command_topic": self.command_topic,
            "joint_names": tuple(self.joint_names),
            "feedback": list(self.feedback),
            "target": list(self.target),
            "limit_min": list(self.limit_min),
            "limit_max": list(self.limit_max),
            "feedback_ready": self.feedback_ready,
            "target_initialized": self.target_initialized,
            "target_version": self.target_version,
            "limit_version": self.limit_version,
        }

    def _expand_limits_to_include(self, values: Sequence[float]) -> None:
        changed = False
        for index, value in enumerate(values):
            if value < self.limit_min[index]:
                self.limit_min[index] = value
                changed = True
            if value > self.limit_max[index]:
                self.limit_max[index] = value
                changed = True
        if changed:
            self.limit_version += 1


@dataclass
class SharedRuntimeState:
    topic_pairs: List[JointTopicRuntimeState] = field(default_factory=list)
    fsm_service_topic: str = DEFAULT_FSM_SERVICE_TOPIC
    fsm_mode: int = 0
    fsm_mode_name: str = "unknown"
    publishing_enabled: bool = False
    status_message: str = ""
    shutdown_requested: bool = False
    lock: threading.RLock = field(default_factory=threading.RLock)


@dataclass
class TopicPairWidgets:
    frame: Any
    joint_names: Tuple[str, ...] = ()
    feedback_vars: List[Any] = field(default_factory=list)
    target_vars: List[Any] = field(default_factory=list)
    limit_min_vars: List[Any] = field(default_factory=list)
    limit_max_vars: List[Any] = field(default_factory=list)
    scales: List[Any] = field(default_factory=list)
    last_target_version: int = -1
    last_limit_version: int = -1


class JointModeSliderApp:
    def __init__(self, root: Any, shared_state: SharedRuntimeState, ros_node: Any) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.shared_state = shared_state
        self.ros_node = ros_node
        self._closed = False
        self._updating_widgets = False
        self._pair_widgets: Dict[int, TopicPairWidgets] = {}
        self._next_pair_row = 0

        self.mode_var = tk.StringVar(value="FSM: unknown")
        self.publish_var = tk.StringVar(value="Publish: stopped")
        self.status_var = tk.StringVar(value="")
        self.fsm_service_var = tk.StringVar(value=shared_state.fsm_service_topic)
        self.state_topic_var = tk.StringVar(value="")
        self.command_topic_var = tk.StringVar(value="")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh_topics()
        self.root.after(500, self.query_mode)
        self._poll_state()

    def _build_ui(self) -> None:
        self.root.title("ec_joint_hardware JointState Slider")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        top = self.ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(5, weight=1)

        self.ttk.Label(top, textvariable=self.mode_var, width=18).grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.ttk.Label(top, textvariable=self.publish_var, width=18).grid(row=0, column=1, sticky="w", padx=(0, 8))
        self.ttk.Label(top, text="FSM").grid(row=0, column=2, sticky="e", padx=(0, 4))
        self.ttk.Entry(top, textvariable=self.fsm_service_var, width=34).grid(row=0, column=3, sticky="ew", padx=(0, 8))

        buttons = self.ttk.Frame(top)
        buttons.grid(row=0, column=4, sticky="e")
        self.ttk.Button(buttons, text="Query Mode", command=self.query_mode).grid(row=0, column=0, padx=2)
        self.ttk.Button(buttons, text="Switch JOINT", command=self.switch_joint).grid(row=0, column=1, padx=2)
        self.ttk.Button(buttons, text="HOLD", command=self.hold).grid(row=0, column=2, padx=2)
        self.ttk.Button(buttons, text="Reset to Current", command=self.reset_to_current).grid(row=0, column=3, padx=2)
        self.ttk.Button(buttons, text="Apply Limits", command=self.apply_limits).grid(row=0, column=4, padx=2)

        topics = self.ttk.Frame(self.root, padding=(8, 0, 8, 8))
        topics.grid(row=1, column=0, sticky="ew")
        topics.columnconfigure(1, weight=1)
        topics.columnconfigure(3, weight=1)
        self.ttk.Label(topics, text="State topic").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.state_topic_combo = self.ttk.Combobox(topics, textvariable=self.state_topic_var)
        self.state_topic_combo.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.ttk.Label(topics, text="Command topic").grid(row=0, column=2, sticky="w", padx=(0, 4))
        self.command_topic_combo = self.ttk.Combobox(topics, textvariable=self.command_topic_var)
        self.command_topic_combo.grid(row=0, column=3, sticky="ew", padx=(0, 8))
        self.ttk.Button(topics, text="Refresh Topics", command=self.refresh_topics).grid(row=0, column=4, padx=2)
        self.ttk.Button(topics, text="Add Pair", command=self.add_pair_from_selection).grid(row=0, column=5, padx=2)

        body = self.ttk.Frame(self.root, padding=(8, 0, 8, 8))
        body.grid(row=2, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        self.pairs_frame = self.ttk.Frame(body)
        self.pairs_frame.grid(row=0, column=0, sticky="nsew")
        self.pairs_frame.columnconfigure(0, weight=1)

        status = self.ttk.Label(self.root, textvariable=self.status_var, anchor="w", padding=(8, 4))
        status.grid(row=3, column=0, sticky="ew")

    def refresh_topics(self) -> None:
        topics = self.ros_node.discover_joint_state_topics()
        self.state_topic_combo.configure(values=topics)
        self.command_topic_combo.configure(values=topics)
        if topics and not self.state_topic_var.get():
            self.state_topic_var.set(topics[0])
        if topics and not self.command_topic_var.get():
            self.command_topic_var.set(topics[0])
        with self.shared_state.lock:
            self.shared_state.status_message = f"Discovered {len(topics)} JointState topics."
        self._poll_state(schedule_next=False)

    def add_pair_from_selection(self) -> None:
        state_topic = self.state_topic_var.get().strip()
        command_topic = self.command_topic_var.get().strip()
        try:
            self._update_fsm_service_topic()
            pair = self.ros_node.add_topic_pair(state_topic, command_topic)
        except ValueError as exc:
            with self.shared_state.lock:
                self.shared_state.status_message = str(exc)
            self._poll_state(schedule_next=False)
            return
        self._ensure_pair_group(pair)
        self._poll_state(schedule_next=False)

    def _ensure_pair_group(self, pair: JointTopicRuntimeState) -> TopicPairWidgets:
        pair_id = id(pair)
        if pair_id in self._pair_widgets:
            return self._pair_widgets[pair_id]

        frame = self.ttk.LabelFrame(
            self.pairs_frame,
            text=f"{pair.label}: {pair.state_topic} -> {pair.command_topic}",
            padding=8,
        )
        frame.grid(row=self._next_pair_row, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(2, weight=1)
        self._next_pair_row += 1

        widgets = TopicPairWidgets(frame=frame)
        self._pair_widgets[pair_id] = widgets
        self._rebuild_pair_rows(pair, widgets, tuple(pair.joint_names))
        return widgets

    def _rebuild_pair_rows(
        self,
        pair: JointTopicRuntimeState,
        widgets: TopicPairWidgets,
        joint_names: Tuple[str, ...],
    ) -> None:
        for child in widgets.frame.winfo_children():
            child.destroy()

        widgets.joint_names = joint_names
        widgets.feedback_vars = []
        widgets.target_vars = []
        widgets.limit_min_vars = []
        widgets.limit_max_vars = []
        widgets.scales = []
        widgets.last_target_version = -1
        widgets.last_limit_version = -1

        headers = ("Joint", "State", "Command")
        for header_column, text in enumerate(headers):
            self.ttk.Label(widgets.frame, text=text).grid(row=0, column=header_column, sticky="w", padx=3)

        if not joint_names:
            self.ttk.Label(widgets.frame, text="Waiting for named JointState feedback.").grid(
                row=1,
                column=0,
                columnspan=3,
                sticky="w",
                padx=3,
                pady=4,
            )
            return

        for index, joint_name in enumerate(joint_names):
            row = index + 1
            feedback_var = self.tk.StringVar(value="--")
            target_var = self.tk.StringVar(value="--")
            limit_min_var = self.tk.StringVar(value=format_limit(DEFAULT_LIMIT_MIN))
            limit_max_var = self.tk.StringVar(value=format_limit(DEFAULT_LIMIT_MAX))

            self.ttk.Label(widgets.frame, text=joint_name, width=32).grid(row=row, column=0, sticky="w", padx=3, pady=2)
            self.ttk.Label(widgets.frame, textvariable=feedback_var, width=10, anchor="e").grid(
                row=row,
                column=1,
                sticky="e",
                padx=3,
            )

            command_cell = self.ttk.Frame(widgets.frame)
            command_cell.grid(row=row, column=2, sticky="ew", padx=3)
            command_cell.columnconfigure(1, weight=1)
            self.ttk.Label(command_cell, textvariable=target_var, width=10, anchor="e").grid(
                row=0,
                column=0,
                sticky="e",
                padx=(0, 4),
            )
            scale = self.tk.Scale(
                command_cell,
                from_=DEFAULT_LIMIT_MIN,
                to=DEFAULT_LIMIT_MAX,
                resolution=0.001,
                orient=self.tk.HORIZONTAL,
                showvalue=False,
                state=self.tk.DISABLED,
                length=320,
                command=lambda value, pair=pair, index=index: self._on_slider(pair, index, value),
            )
            scale.grid(row=0, column=1, sticky="ew", padx=(0, 4))
            self.ttk.Entry(command_cell, textvariable=limit_min_var, width=8).grid(row=0, column=2, padx=(0, 2))
            self.ttk.Entry(command_cell, textvariable=limit_max_var, width=8).grid(row=0, column=3, padx=(2, 0))

            widgets.feedback_vars.append(feedback_var)
            widgets.target_vars.append(target_var)
            widgets.limit_min_vars.append(limit_min_var)
            widgets.limit_max_vars.append(limit_max_var)
            widgets.scales.append(scale)

    def _on_slider(self, pair: JointTopicRuntimeState, index: int, value_text: str) -> None:
        if self._updating_widgets:
            return
        value = _finite_float(value_text)
        if value is None:
            return

        with self.shared_state.lock:
            if pair not in self.shared_state.topic_pairs or not pair.feedback_ready:
                return
            pair.set_target(index, value)

    def _update_fsm_service_topic(self) -> None:
        self.ros_node.configure_fsm_service_topic(self.fsm_service_var.get().strip())

    def query_mode(self) -> None:
        self._update_fsm_service_topic()
        self.ros_node.request_mode(MODE_QUERY)

    def switch_joint(self) -> None:
        self._update_fsm_service_topic()
        self.ros_node.request_mode(MODE_JOINT)

    def hold(self) -> None:
        self._update_fsm_service_topic()
        self.ros_node.request_mode(MODE_HOLD)

    def reset_to_current(self) -> None:
        with self.shared_state.lock:
            changed = False
            for pair in self.shared_state.topic_pairs:
                changed = pair.reset_target_to_feedback() or changed
            if changed:
                self.shared_state.status_message = "Targets reset from latest valid feedback."
            else:
                self.shared_state.status_message = "No valid feedback available to reset targets."
        self._poll_state(schedule_next=False)

    def apply_limits(self) -> None:
        with self.shared_state.lock:
            pairs_by_id = {id(pair): pair for pair in self.shared_state.topic_pairs}

        parsed_limits = {}
        for pair_id, widgets in self._pair_widgets.items():
            pair = pairs_by_id.get(pair_id)
            if pair is None or not widgets.joint_names:
                continue

            minimums = []
            maximums = []
            for index, joint_name in enumerate(widgets.joint_names):
                try:
                    minimum, maximum = parse_limit_pair(
                        widgets.limit_min_vars[index].get(),
                        widgets.limit_max_vars[index].get(),
                    )
                except ValueError as exc:
                    with self.shared_state.lock:
                        self.shared_state.status_message = f"{pair.label} {joint_name}: {exc}"
                    self._poll_state(schedule_next=False)
                    return
                minimums.append(minimum)
                maximums.append(maximum)
            parsed_limits[pair_id] = (minimums, maximums)

        with self.shared_state.lock:
            for pair_id, (minimums, maximums) in parsed_limits.items():
                pair = pairs_by_id.get(pair_id)
                if pair is not None:
                    pair.apply_limits(minimums, maximums)
            self.shared_state.status_message = "Applied slider limits."
        self._poll_state(schedule_next=False)

    def close(self) -> None:
        self._closed = True
        with self.shared_state.lock:
            self.shared_state.publishing_enabled = False
            self.shared_state.shutdown_requested = True
            self.shared_state.status_message = "Shutting down."
        self.root.destroy()

    def _poll_state(self, schedule_next: bool = True) -> None:
        if self._closed:
            return

        with self.shared_state.lock:
            for pair in self.shared_state.topic_pairs:
                pair.expire_stale_feedback()
            pair_snapshots = [(pair, pair.snapshot()) for pair in self.shared_state.topic_pairs]
            snapshot = {
                "fsm_mode": self.shared_state.fsm_mode,
                "fsm_mode_name": self.shared_state.fsm_mode_name,
                "publishing_enabled": self.shared_state.publishing_enabled,
                "status_message": self.shared_state.status_message,
            }

        mode_name = snapshot["fsm_mode_name"] or "unknown"
        self.mode_var.set(f"FSM: {mode_name} ({snapshot['fsm_mode']})")
        self.publish_var.set("Publish: JOINT active" if snapshot["publishing_enabled"] else "Publish: stopped")
        self.status_var.set(snapshot["status_message"])

        for pair, pair_snapshot in pair_snapshots:
            widgets = self._ensure_pair_group(pair)
            self._render_pair(pair, widgets, pair_snapshot)

        if schedule_next:
            self.root.after(UI_POLL_PERIOD_MS, self._poll_state)

    def _render_pair(self, pair: JointTopicRuntimeState, widgets: TopicPairWidgets, snapshot: dict) -> None:
        joint_names = tuple(snapshot["joint_names"])
        if widgets.joint_names != joint_names:
            self._rebuild_pair_rows(pair, widgets, joint_names)
        if not joint_names:
            return

        scale_state = self.tk.NORMAL if snapshot["feedback_ready"] else self.tk.DISABLED
        self._updating_widgets = True
        try:
            if widgets.last_limit_version != snapshot["limit_version"]:
                for index, scale in enumerate(widgets.scales):
                    scale.configure(from_=snapshot["limit_min"][index], to=snapshot["limit_max"][index])
                    widgets.limit_min_vars[index].set(format_limit(snapshot["limit_min"][index]))
                    widgets.limit_max_vars[index].set(format_limit(snapshot["limit_max"][index]))
                widgets.last_limit_version = snapshot["limit_version"]

            for scale in widgets.scales:
                scale.configure(state=scale_state)

            for index in range(len(joint_names)):
                feedback = snapshot["feedback"][index] if index < len(snapshot["feedback"]) else math.nan
                widgets.feedback_vars[index].set(format_float(feedback))

                if snapshot["target_initialized"]:
                    widgets.target_vars[index].set(format_float(snapshot["target"][index]))
                else:
                    widgets.target_vars[index].set("--")

            if widgets.last_target_version != snapshot["target_version"] and snapshot["target_initialized"]:
                for index, scale in enumerate(widgets.scales):
                    scale.set(snapshot["target"][index])
                widgets.last_target_version = snapshot["target_version"]
        finally:
            self._updating_widgets = False


def create_ros_node(shared_state: SharedRuntimeState) -> Any:
    try:
        from rclpy.node import Node
        from robot_interfaces.srv import FsmState
        from sensor_msgs.msg import JointState
    except ModuleNotFoundError as exc:
        maybe_reexec_with_workspace_setup()
        module_name = exc.name or "ROS message/service package"
        raise StartupError(build_missing_ros_dependency_message(module_name)) from exc

    class JointModeSliderNode(Node):
        def __init__(self) -> None:
            super().__init__("ec_joint_hardware_joint_mode_slider")
            self.fsm_client = self.create_client(FsmState, shared_state.fsm_service_topic)
            self._publishers_by_pair: Dict[int, Any] = {}
            self._subscriptions_by_pair: Dict[int, Any] = {}
            self._mode_request_generation = 0
            self.create_timer(PUBLISH_PERIOD_SEC, self._publish_targets)

        def discover_joint_state_topics(self) -> List[str]:
            try:
                topics_and_types = self.get_topic_names_and_types()
            except Exception:
                return []

            topics = [
                topic_name
                for topic_name, topic_types in topics_and_types
                if is_joint_state_topic_type(topic_types)
            ]
            return sorted(set(topics))

        def configure_fsm_service_topic(self, topic: str) -> None:
            if not topic:
                raise ValueError("FSM service topic must not be empty")
            with shared_state.lock:
                if topic == shared_state.fsm_service_topic:
                    return
                shared_state.fsm_service_topic = topic
                shared_state.status_message = f"FSM service topic set to {topic}."
            self.fsm_client = self.create_client(FsmState, topic)

        def add_topic_pair(
            self,
            state_topic: str,
            command_topic: str,
            label: Optional[str] = None,
        ) -> JointTopicRuntimeState:
            state_topic = state_topic.strip()
            command_topic = command_topic.strip()
            if not state_topic:
                raise ValueError("State topic must not be empty")
            if not command_topic:
                raise ValueError("Command topic must not be empty")

            with shared_state.lock:
                for pair in shared_state.topic_pairs:
                    if pair.state_topic == state_topic and pair.command_topic == command_topic:
                        existing = pair
                        break
                else:
                    existing = JointTopicRuntimeState(
                        label or f"Pair {len(shared_state.topic_pairs) + 1}",
                        state_topic,
                        command_topic,
                    )
                    shared_state.topic_pairs.append(existing)
                    shared_state.status_message = f"Added topic pair {state_topic} -> {command_topic}."

            self._ensure_topic_pair_ros_handles(existing)
            return existing

        def _ensure_topic_pair_ros_handles(self, pair: JointTopicRuntimeState) -> None:
            pair_id = id(pair)
            if pair_id not in self._publishers_by_pair:
                self._publishers_by_pair[pair_id] = self.create_publisher(JointState, pair.command_topic, 10)
            if pair_id not in self._subscriptions_by_pair:
                self._subscriptions_by_pair[pair_id] = self.create_subscription(
                    JointState,
                    pair.state_topic,
                    lambda msg, pair=pair: self._feedback_callback(pair, msg),
                    10,
                )

        def request_mode(self, target_mode: int) -> None:
            with shared_state.lock:
                self._mode_request_generation += 1
                request_generation = self._mode_request_generation
                if target_mode == MODE_HOLD:
                    shared_state.publishing_enabled = False
                shared_state.status_message = f"Requesting FSM mode {target_mode}."
                service_topic = shared_state.fsm_service_topic

            if not self.fsm_client.wait_for_service(timeout_sec=0.0):
                with shared_state.lock:
                    if request_generation != self._mode_request_generation:
                        return
                    if target_mode == MODE_JOINT:
                        shared_state.publishing_enabled = False
                    shared_state.status_message = f"FSM service unavailable: {service_topic}"
                return

            request = FsmState.Request()
            request.target_mode = target_mode
            future = self.fsm_client.call_async(request)
            future.add_done_callback(
                lambda done_future: self._handle_fsm_response(
                    target_mode,
                    request_generation,
                    done_future,
                )
            )

        def _handle_fsm_response(self, requested_mode: int, request_generation: int, future: Any) -> None:
            with shared_state.lock:
                if request_generation != self._mode_request_generation:
                    return

            try:
                response = future.result()
            except Exception as exc:
                with shared_state.lock:
                    if request_generation != self._mode_request_generation:
                        return
                    if requested_mode in (MODE_HOLD, MODE_JOINT):
                        shared_state.publishing_enabled = False
                    shared_state.status_message = f"FSM request failed: {exc}"
                return

            mode_name = response.current_mode_name or "unknown"
            is_joint = response.success and (response.current_mode == MODE_JOINT or mode_name.upper() == "JOINT")
            with shared_state.lock:
                if request_generation != self._mode_request_generation:
                    return
                shared_state.fsm_mode = int(response.current_mode)
                shared_state.fsm_mode_name = mode_name
                if requested_mode == MODE_JOINT:
                    if is_joint:
                        for pair in shared_state.topic_pairs:
                            pair.reset_target_to_feedback()
                    shared_state.publishing_enabled = is_joint
                elif requested_mode == MODE_HOLD:
                    shared_state.publishing_enabled = False
                elif requested_mode == MODE_QUERY and not is_joint:
                    shared_state.publishing_enabled = False

                result = "OK" if response.success else "ERROR"
                publish_state = "publishing enabled" if shared_state.publishing_enabled else "publishing stopped"
                shared_state.status_message = f"{result}: {response.message}; mode={mode_name}; {publish_state}."

        def _feedback_callback(self, pair: JointTopicRuntimeState, msg: Any) -> None:
            message_names = tuple(str(name) for name in msg.name)
            with shared_state.lock:
                if not message_names:
                    pair.invalidate_feedback()
                    shared_state.status_message = f"{pair.state_topic}: JointState.name is empty."
                    return

                had_joint_names = bool(pair.joint_names)
                changed = pair.configure_joints(message_names)
                if changed and had_joint_names:
                    shared_state.status_message = (
                        f"{pair.state_topic}: joint names changed; waiting for next complete feedback."
                    )
                    return

                positions = extract_valid_positions(msg.position, msg.name, pair.joint_names)
                if positions is None:
                    pair.invalidate_feedback()
                    shared_state.status_message = f"{pair.state_topic}: invalid or incomplete JointState feedback."
                    return
                pair.update_feedback(positions)

        def _publish_targets(self) -> None:
            with shared_state.lock:
                for pair in shared_state.topic_pairs:
                    pair.expire_stale_feedback()
                if not shared_state.publishing_enabled or shared_state.shutdown_requested:
                    return

                now = self.get_clock().now().to_msg()
                for pair in shared_state.topic_pairs:
                    target = pair.build_command_target()
                    if target is None:
                        continue

                    publisher = self._publishers_by_pair.get(id(pair))
                    if publisher is None:
                        continue

                    msg = JointState()
                    msg.header.stamp = now
                    msg.name = list(pair.joint_names)
                    msg.position = target
                    publisher.publish(msg)

    return JointModeSliderNode()


def _spin_ros_node(rclpy_module: Any, node: Any) -> None:
    try:
        rclpy_module.spin(node)
    except Exception as exc:
        if rclpy_module.ok():
            print(f"ROS spin failed: {exc}", file=sys.stderr)


def main() -> int:
    try:
        rclpy = import_rclpy_module()
        import tkinter as tk
    except StartupError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    rclpy.init()
    shared_state = SharedRuntimeState()
    node = None
    spin_thread = None

    try:
        node = create_ros_node(shared_state)
        spin_thread = threading.Thread(target=_spin_ros_node, args=(rclpy, node), daemon=True)
        spin_thread.start()

        root = tk.Tk()
        JointModeSliderApp(root, shared_state, node)
        root.mainloop()
    except StartupError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        with shared_state.lock:
            shared_state.publishing_enabled = False
            shared_state.shutdown_requested = True
    finally:
        with shared_state.lock:
            shared_state.publishing_enabled = False
            shared_state.shutdown_requested = True

        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None and hasattr(node, "destroy_node"):
            node.destroy_node()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
