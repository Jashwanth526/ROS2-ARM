import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def cleaned_env():
        env = dict(os.environ)
        ld = env.get("LD_LIBRARY_PATH", "")
        if ld:
            parts = [p for p in ld.split(":") if "snap" not in p]
            env["LD_LIBRARY_PATH"] = ":".join(parts)
        return env
    arduinobot_description_dir = get_package_share_directory("arduinobot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        arduinobot_description_dir, "urdf", "arduinobot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    launch_rviz_arg = DeclareLaunchArgument(
        name="launch_rviz",
        default_value="True",
        description="Launch RViz2 (set to False if RViz fails due to Snap/GLIBC conflict)"
    )

    launch_jsp_gui_arg = DeclareLaunchArgument(
        name="launch_jsp_gui",
        default_value="True",
        description="Launch joint_state_publisher_gui"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        env=cleaned_env(),
        condition=IfCondition(LaunchConfiguration("launch_jsp_gui")),
    )

    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    world_markers_node = Node(
        package="arduinobot_utils",
        executable="world_markers_publisher",
        name="world_markers_publisher",
        output="screen",
        respawn=True
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(arduinobot_description_dir, "rviz", "display.rviz")],
        env=cleaned_env(),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        model_arg,
        launch_rviz_arg,
        launch_jsp_gui_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        static_world_to_base,
        world_markers_node,
        rviz_node
    ])
