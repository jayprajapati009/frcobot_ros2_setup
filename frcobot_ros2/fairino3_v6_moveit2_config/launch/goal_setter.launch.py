# Import necessary modules for ROS 2 launch description
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Generate a launch description to include multiple launch files for a MoveIt configuration.

    This function sets up paths for MoveIt and robot description packages,
    includes their respective launch files, and spawns controllers.
    """
    # Get the package paths using ament_index
    moveit_config_path = get_package_share_directory('fairino3_v6_moveit2_config')
    description_config_path = get_package_share_directory('fairino_description')

    # Include the robot description launch file (URDF and SRDF)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, 'launch', 'rsp.launch.py')  # Path to rsp.launch.py
        )
    )

    # Include MoveIt demo launch file
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, 'launch', 'demo.launch.py')  # Path to demo.launch.py
        )
    )

    # Include controllers spawn launch file
    controllers_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_path, 'launch', 'spawn_controllers.launch.py')  # Path to spawn_controllers.launch.py
        )
    )

    # Return the LaunchDescription object with all included launches
    return LaunchDescription([
        rsp_launch,        # Robot description launch
        moveit_demo,       # MoveIt demo launch
        controllers_spawn  # Controllers spawn launch
    ])
