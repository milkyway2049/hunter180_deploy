import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    model_path = get_package_share_directory("ec_description")

    xacro_file = os.path.join(model_path, "urdf", "hunter180.xacro")

    controller_path = get_package_share_directory("ec_controller")

    # Process xacro file
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    controllers_config_file = os.path.join(
        controller_path, "config", "controllers.yaml"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config_file],
        output="both",
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    default_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["default_controller", "--controller-manager", "controller_manager"],
        output="screen",
    )

    stand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "stand_controller",
            "--controller-manager",
            "controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    walk_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "walk_controller",
            "--controller-manager",
            "controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    controller_switcher_node = Node(
        package="ec_controller",
        executable="controller_switcher_node",
        name="controller_switcher",
        output="screen",
        parameters=[controllers_config_file],
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=control_node,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[default_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=default_controller_spawner,
                    on_exit=[stand_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=stand_controller_spawner,
                    on_exit=[walk_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=walk_controller_spawner,
                    on_exit=[controller_switcher_node],
                )
            ),
            control_node,
            node_robot_state_publisher,
        ]
    )
