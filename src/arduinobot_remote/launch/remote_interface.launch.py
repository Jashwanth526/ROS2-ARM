import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )
    
    use_simple_arg = DeclareLaunchArgument(
        "use_simple",
        default_value="True",
        description="Use simple task server (recommended for object detection)"
    )

    grasp_z_offset_arg = DeclareLaunchArgument(
        "grasp_z_offset",
        default_value="0.005",
        description="TCP height above object for grasp (m). Lower to go further down."
    )

    pregrasp_z_offset_arg = DeclareLaunchArgument(
        "pregrasp_z_offset",
        default_value="0.12",
        description="Hover height above object before grasp (m)."
    )

    enable_alexa_arg = DeclareLaunchArgument(
        "enable_alexa",
        default_value="False",
        description="Whether to launch the Flask-based Alexa interface from this launch file"
    )

    use_python = LaunchConfiguration("use_python")
    use_simple = LaunchConfiguration("use_simple")
    is_sim = LaunchConfiguration("is_sim")
    enable_alexa = LaunchConfiguration("enable_alexa")
    grasp_z_offset = LaunchConfiguration("grasp_z_offset")
    pregrasp_z_offset = LaunchConfiguration("pregrasp_z_offset")

    task_server_simple = Node(
        package="arduinobot_remote",
        executable="task_server_simple_node",
        condition=IfCondition(use_simple),
        parameters=[
            {"use_sim_time": is_sim},
            {"grasp_z_offset": grasp_z_offset},
            {"pregrasp_z_offset": pregrasp_z_offset},
        ],
        output='screen'
    )

    # Original complex task server
    task_server_node = Node(
        package="arduinobot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_simple),
        parameters=[{"use_sim_time": is_sim}]
    )

    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("arduinobot_description"),
            "urdf",
            "arduinobot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/arduinobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    task_server_node_py = Node(
        package="arduinobot_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": is_sim}]
    )

    alexa_interface_node = Node(
        package="arduinobot_remote",
        executable="alexa_interface.py",
        parameters=[{"use_sim_time": is_sim}],
        output='screen',
        condition=IfCondition(enable_alexa)
    )

    return LaunchDescription([
        use_python_arg,
        use_simple_arg,
        grasp_z_offset_arg,
        pregrasp_z_offset_arg,
        enable_alexa_arg,
        is_sim_arg,
        task_server_simple,
        task_server_node,
        task_server_node_py,
        alexa_interface_node,
    ])