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
        'sitl_params.yaml'
    )

    # Create a launch description to start our nodes
    launch_description = LaunchDescription([
        # Launch vision_bridge node
        Node(
            package="drone_control",
            executable="vision_bridge",
            output="screen",
            namespace=f"{namespace}",
            parameters=[config]
            ),
        # Launch camera static transform publisher (camera pointing straight down)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "-1.5708", "0", "camera_pose_frame", "base_link"]
            ),
        # Launch odom static transform publisher (connecting odom and odom_frame)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "odom_frame"]
            ),
        # Launch map-odom link static transform publisher 
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "map"]
            ),
    ]) # end LaunchDescription

    # Return launch description
    return launch_description











