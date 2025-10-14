import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("arduinobot_description"),
                    "urdf",
                    "arduinobot.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,},
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "config",
                "arduinobot_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Add a delay to controller manager to ensure robot_state_publisher is ready
    delayed_controller_manager = TimerAction(
        period=2.0,  # Wait 2 seconds
        actions=[controller_manager]
    )

    # Wait for controller manager to be ready before spawning controllers
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[arm_controller_spawner]
                )
            ]
        )
    )

    delayed_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[gripper_controller_spawner]
                )
            ]
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            delayed_controller_manager,
            delayed_joint_state_broadcaster_spawner,
            delayed_arm_controller_spawner,
            delayed_gripper_controller_spawner,
        ]
    )
