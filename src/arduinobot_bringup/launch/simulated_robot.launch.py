import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )
    
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_moveit"),
            "launch",
            "moveit.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )
    
    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduinobot_remote"),
            "launch",
            "remote_interface.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )
    
    object_detector = Node(
        package='object_detection',
        executable='object_detector',
        name='object_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    world_markers = Node(
        package='arduinobot_utils',
        executable='world_markers_publisher.py',
        name='world_markers_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        respawn=True
    )
    
    ee_markers = Node(
        package='arduinobot_utils',
        executable='ee_markers_publisher',
        name='ee_markers_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        respawn=True
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface,
        object_detector,
        world_markers,
        ee_markers
    ])