import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include the main robot simulation
    arduinobot_description = get_package_share_directory("arduinobot_description")
    arduinobot_moveit = get_package_share_directory("arduinobot_moveit") 
    arduinobot_controller = get_package_share_directory("arduinobot_controller")

    # Launch Gazebo with the robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduinobot_description, "launch"), "/gazebo.launch.py"
        ])
    )
    
    # Launch MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduinobot_moveit, "launch"), "/moveit.launch.py"
        ])
    )
    
    # Launch controllers
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arduinobot_controller, "launch"), "/controller.launch.py"
        ])
    )

    # Object detection node
    object_detector_node = Node(
        package="object_detection",
        executable="object_detector",
        name="object_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )

    # Enhanced task server (replaces the original task server)
    enhanced_task_server_node = Node(
        package="object_detection", 
        executable="enhanced_task_server",
        name="enhanced_task_server",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        controller_launch,
        object_detector_node,
        enhanced_task_server_node,
    ])