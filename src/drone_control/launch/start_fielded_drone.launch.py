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
        'params.yaml'
    )

    # Create a launch description to start our nodes
    launch_description = LaunchDescription([
        # Launch the MAVROS node
        Node(
            package="mavros", 
            executable="mavros_node",
            output="screen",
            namespace=f"{namespace}/mavros",
            parameters=[config]
            ),
        # Launch realsense node
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            output="screen",
            namespace=f"{namespace}/realsense",
            parameters=[config]
            ),
        # Launch static transform publisher (camera pointing straight down)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "1.5708", "0", "base_link", "camera_pose_frame"]
            ),

    ]) # end LaunchDescription

    # Return launch description
    return launch_description
