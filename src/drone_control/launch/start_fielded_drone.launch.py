from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

# 
# File to launch mavros for a fielded drone
#


namespace = "drone"

""" Main launch function that ROS2 will run when we use `ros2 launch ...` """
def generate_launch_description():
    # Create a launch description to start our nodes
    launch_description = LaunchDescription([
        # Launch the MAVROS node
        Node(
                package="mavros", 
                executable="mavros_node",
                output="screen",
                namespace=f"{namespace}/mavros",
                parameters=[{
                        "fcu_url": "/dev/ttyTHS2:921600",
                        "gcs_url": "udp://@localhost",
                        "target_system_id": 1,
                        "target_component_id": 1,
                        "fcu_protocol": "v2.0",
                    }]
            ),
          
    ]) # end LaunchDescription

    # Return launch description
    return launch_description
