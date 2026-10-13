import importlib.util
import math
from pathlib import Path
import sys
import types
from types import SimpleNamespace

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "joint_mode_slider.py"
SPEC = importlib.util.spec_from_file_location("joint_mode_slider", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
joint_mode_slider = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = joint_mode_slider
SPEC.loader.exec_module(joint_mode_slider)


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeClient:
    def wait_for_service(self, timeout_sec: float) -> bool:
        del timeout_sec
        return False


class FakeJointState:
    def __init__(self) -> None:
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


class FakeNode:
    discovered_topics = []

    def __init__(self, name: str) -> None:
        self.name = name
        self.publishers = {}
        self.subscriptions = []
        self.timers = []
        self.clients = {}

    def create_publisher(self, message_type, topic: str, depth: int):
        del message_type, depth
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_client(self, service_type, topic: str):
        del service_type
        client = FakeClient()
        self.clients[topic] = client
        return client

    def create_subscription(self, message_type, topic: str, callback, depth: int) -> None:
        del message_type, depth
        self.subscriptions.append((topic, callback))

    def create_timer(self, period: float, callback) -> None:
        self.timers.append((period, callback))

    def get_topic_names_and_types(self):
        return list(self.discovered_topics)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))


class FakeFsmState:
    class Request:
        def __init__(self) -> None:
            self.target_mode = 0


def install_fake_ros_modules(monkeypatch) -> None:
    FakeNode.discovered_topics = []

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = FakeNode
    rclpy_module.node = rclpy_node_module

    robot_interfaces_module = types.ModuleType("robot_interfaces")
    robot_interfaces_srv_module = types.ModuleType("robot_interfaces.srv")
    robot_interfaces_srv_module.FsmState = FakeFsmState
    robot_interfaces_module.srv = robot_interfaces_srv_module

    sensor_msgs_module = types.ModuleType("sensor_msgs")
    sensor_msgs_msg_module = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg_module.JointState = FakeJointState
    sensor_msgs_module.msg = sensor_msgs_msg_module

    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node_module)
    monkeypatch.setitem(sys.modules, "robot_interfaces", robot_interfaces_module)
    monkeypatch.setitem(sys.modules, "robot_interfaces.srv", robot_interfaces_srv_module)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs_module)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs_msg_module)


def make_joint_state(names, positions) -> FakeJointState:
    message = FakeJointState()
    message.name = list(names)
    message.position = list(positions)
    return message


def test_unmodified_joints_follow_latest_feedback() -> None:
    state = joint_mode_slider.JointTopicRuntimeState(
        "Arm",
        "/arm/state",
        "/arm/command",
        ("joint_a", "joint_b", "joint_c"),
    )
    state.update_feedback([0.0, 0.1, 0.2])
    state.set_target(1, 0.75)

    latest_feedback = [0.01, 0.11, 0.21]
    state.update_feedback(latest_feedback)

    command = state.build_command_target()
    assert command is not None
    assert command == pytest.approx([0.01, 0.75, 0.21])


def test_command_target_requires_complete_valid_feedback() -> None:
    state = joint_mode_slider.JointTopicRuntimeState(
        "Arm",
        "/arm/state",
        "/arm/command",
        ("joint_a", "joint_b"),
    )
    assert state.build_command_target() is None

    positions = joint_mode_slider.extract_valid_positions(
        [0.1, float("nan")],
        ["joint_a", "joint_b"],
        state.joint_names,
    )
    assert positions is None


def test_missing_dependency_message_mentions_ec_joint_hardware_package(monkeypatch) -> None:
    monkeypatch.setattr(joint_mode_slider, "find_workspace_setup", lambda: None)

    message = joint_mode_slider.build_missing_ros_dependency_message("rclpy")

    assert "ec_joint_hardware" in message
    assert "ec_slave_arm_controller ec_slave_arm" not in message


def test_feedback_expires_at_half_second_and_clears_stale_command() -> None:
    state = joint_mode_slider.JointTopicRuntimeState(
        "Arm",
        "/arm/state",
        "/arm/command",
        ("joint_a", "joint_b"),
    )
    state.update_feedback([0.1, 0.2], monotonic_time=10.0)
    state.set_target(0, 0.9)

    assert state.build_command_target(monotonic_time=10.499) is not None
    assert state.build_command_target(monotonic_time=10.5) is None
    assert not state.feedback_ready
    assert all(math.isnan(value) for value in state.feedback)
    assert state.target == []
    assert state.target_overridden == [False, False]


def test_discovers_only_joint_state_topics(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    FakeNode.discovered_topics = [
        ("/arm/state", ["sensor_msgs/msg/JointState"]),
        ("/diagnostics", ["diagnostic_msgs/msg/DiagnosticArray"]),
        ("/arm/command", ["sensor_msgs/msg/JointState"]),
        ("/legacy", ["sensor_msgs/JointState"]),
    ]

    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)

    assert node.discover_joint_state_topics() == ["/arm/command", "/arm/state", "/legacy"]


def test_topic_pair_accepts_arbitrary_joint_count_and_publishes_after_joint_mode(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    monkeypatch.setattr(joint_mode_slider, "monotonic", lambda: 20.0)

    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    pair = node.add_topic_pair("/arm/state", "/arm/command", label="Arm")

    node._feedback_callback(pair, make_joint_state(["joint_a", "joint_b", "joint_c"], [0.1, 0.2, 0.3]))
    pair.set_target(1, 1.2)

    node._publish_targets()
    assert node.publishers["/arm/command"].messages == []

    shared_state.publishing_enabled = True
    node._publish_targets()

    assert len(node.publishers["/arm/command"].messages) == 1
    message = node.publishers["/arm/command"].messages[0]
    assert message.name == ["joint_a", "joint_b", "joint_c"]
    assert message.position == pytest.approx([0.1, 1.2, 0.3])


def test_invalid_feedback_stops_only_affected_pair(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    monkeypatch.setattr(joint_mode_slider, "monotonic", lambda: 30.0)

    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    left = node.add_topic_pair("/left/state", "/left/command", label="Left")
    right = node.add_topic_pair("/right/state", "/right/command", label="Right")

    node._feedback_callback(left, make_joint_state(["left_a", "left_b"], [0.1, 0.2]))
    node._feedback_callback(right, make_joint_state(["right_a", "right_b"], [0.3, 0.4]))
    node._feedback_callback(left, make_joint_state(["left_a", "left_b"], [float("nan"), 0.2]))

    shared_state.publishing_enabled = True
    node._publish_targets()

    assert node.publishers["/left/command"].messages == []
    assert len(node.publishers["/right/command"].messages) == 1
    assert node.publishers["/right/command"].messages[0].position == pytest.approx([0.3, 0.4])


def test_topic_pair_rebuilds_when_state_joint_names_change_and_requires_next_feedback(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    monkeypatch.setattr(joint_mode_slider, "monotonic", lambda: 40.0)

    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    pair = node.add_topic_pair("/arm/state", "/arm/command", label="Arm")

    node._feedback_callback(pair, make_joint_state(["joint_a", "joint_b"], [0.1, 0.2]))
    assert pair.feedback_ready
    assert pair.joint_names == ("joint_a", "joint_b")

    node._feedback_callback(pair, make_joint_state(["joint_a", "joint_c"], [0.3, 0.4]))
    assert pair.joint_names == ("joint_a", "joint_c")
    assert not pair.feedback_ready
    assert pair.target == []

    node._feedback_callback(pair, make_joint_state(["joint_a", "joint_c"], [0.5, 0.6]))
    assert pair.feedback_ready
    assert pair.build_command_target() == pytest.approx([0.5, 0.6])


def test_hold_request_stops_generic_topic_publishing(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    pair = node.add_topic_pair("/arm/state", "/arm/command", label="Arm")
    node._feedback_callback(pair, make_joint_state(["joint_a"], [0.1]))
    shared_state.publishing_enabled = True

    class ControlledFuture:
        def __init__(self) -> None:
            self.callback = None
            self.response = None

        def add_done_callback(self, callback) -> None:
            self.callback = callback

        def result(self):
            return self.response

        def complete(self, response) -> None:
            self.response = response
            self.callback(self)

    class ControlledClient:
        def __init__(self) -> None:
            self.futures = []

        def wait_for_service(self, timeout_sec: float) -> bool:
            del timeout_sec
            return True

        def call_async(self, request):
            future = ControlledFuture()
            future.target_mode = request.target_mode
            self.futures.append(future)
            return future

    client = ControlledClient()
    node.fsm_client = client

    node.request_mode(joint_mode_slider.MODE_HOLD)
    assert not shared_state.publishing_enabled
    node._publish_targets()
    assert node.publishers["/arm/command"].messages == []

    hold_response = SimpleNamespace(
        success=True,
        current_mode=joint_mode_slider.MODE_HOLD,
        current_mode_name="HOLD",
        message="holding",
    )
    client.futures[0].complete(hold_response)
    assert shared_state.fsm_mode == joint_mode_slider.MODE_HOLD
    assert shared_state.fsm_mode_name == "HOLD"


def test_stale_joint_mode_response_cannot_override_newer_hold(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    pair = node.add_topic_pair("/arm/state", "/arm/command", label="Arm")
    node._feedback_callback(pair, make_joint_state(["joint_a"], [0.1]))

    class ControlledFuture:
        def __init__(self) -> None:
            self.callback = None
            self.response = None

        def add_done_callback(self, callback) -> None:
            self.callback = callback

        def result(self):
            return self.response

        def complete(self, response) -> None:
            self.response = response
            self.callback(self)

    class ControlledClient:
        def __init__(self) -> None:
            self.futures = []

        def wait_for_service(self, timeout_sec: float) -> bool:
            del timeout_sec
            return True

        def call_async(self, request):
            future = ControlledFuture()
            future.target_mode = request.target_mode
            self.futures.append(future)
            return future

    client = ControlledClient()
    node.fsm_client = client

    node.request_mode(joint_mode_slider.MODE_JOINT)
    node.request_mode(joint_mode_slider.MODE_HOLD)
    assert [future.target_mode for future in client.futures] == [
        joint_mode_slider.MODE_JOINT,
        joint_mode_slider.MODE_HOLD,
    ]

    hold_response = SimpleNamespace(
        success=True,
        current_mode=joint_mode_slider.MODE_HOLD,
        current_mode_name="HOLD",
        message="holding",
    )
    client.futures[1].complete(hold_response)
    assert not shared_state.publishing_enabled
    assert shared_state.fsm_mode == joint_mode_slider.MODE_HOLD

    stale_joint_response = SimpleNamespace(
        success=True,
        current_mode=joint_mode_slider.MODE_JOINT,
        current_mode_name="JOINT",
        message="joint",
    )
    client.futures[0].complete(stale_joint_response)

    assert not shared_state.publishing_enabled
    assert shared_state.fsm_mode == joint_mode_slider.MODE_HOLD
    assert shared_state.fsm_mode_name == "HOLD"


def test_publish_is_atomic_with_feedback_expiration(monkeypatch) -> None:
    install_fake_ros_modules(monkeypatch)
    monkeypatch.setattr(joint_mode_slider, "monotonic", lambda: 50.0)

    shared_state = joint_mode_slider.SharedRuntimeState()
    node = joint_mode_slider.create_ros_node(shared_state)
    pair = node.add_topic_pair("/arm/state", "/arm/command", label="Arm")
    node._feedback_callback(pair, make_joint_state(["joint_a"], [0.1]))
    shared_state.publishing_enabled = True

    lock_owned_during_publish = []
    original_publish = node.publishers["/arm/command"].publish

    def record_lock_state(message) -> None:
        lock_owned_during_publish.append(shared_state.lock._is_owned())
        original_publish(message)

    node.publishers["/arm/command"].publish = record_lock_state
    node._publish_targets()

    assert lock_owned_during_publish == [True]


def test_feedback_is_reordered_by_joint_name() -> None:
    ordered = joint_mode_slider.extract_valid_positions(
        [2.0, 0.0, 1.0],
        ["joint_c", "joint_a", "joint_b"],
        ("joint_a", "joint_b", "joint_c"),
    )

    assert ordered == pytest.approx([0.0, 1.0, 2.0])
