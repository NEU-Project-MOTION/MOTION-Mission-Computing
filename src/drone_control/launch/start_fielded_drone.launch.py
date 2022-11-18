import os
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 
# File to launch mavros for a fielded drone
#


namespace = "drone"

""" Main launch function that ROS2 will run when we use `ros2 launch ...` """
def generate_launch_description():
    config = os.path.join(get_package_share_directory('drone_control'),
        'config',
        'fielded_params.yaml'
    )

    # Create a launch description to start our nodes
    launch_description = LaunchDescription([
        # Launch the MAVROS node
        Node(
            package="mavros", 
            executable="mavros_node",
            output="screen",
            namespace=f"{namespace}/mavros",
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[config],
            respawn=True
            ),
        # Launch realsense T265 node
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            output="screen",
            namespace=f"{namespace}/T265",
#            arguments=['--ros-args', '--log-level', 'DEBUG'],
            parameters=[config],
            respawn=True
            ),
        # Launch realsense D435i node
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            output="screen",
            namespace=f"{namespace}/D435",
            parameters=[config],
            respawn=True
            ),

    ]) # end LaunchDescription

    # Return launch description
    return launch_description
