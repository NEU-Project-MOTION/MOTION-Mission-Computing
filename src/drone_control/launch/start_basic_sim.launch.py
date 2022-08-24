import os
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

# 
# File to launch a basic PX4 sim with gazebo and default world/drone
#

# https://github.com/ros2/launch_ros/tree/master/launch_ros/launch_ros/actions
# https://github.com/ros2/launch/tree/master/launch/launch/actions

namespace = "drone"

""" Code to launch the sim (SITL) """
def launch_sitl(context, *args, **kwargs):
    # TODO make faster by not building: https://docs.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html
    # os.system(f'xterm -e "$HOME/MOTION-Mission-Computing/PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh -m iris -n 1"')
    
    # Run PX4 in xterm, and pause (read) after its done incase there were errors
    os.system(f'xterm -e "cd $HOME/MOTION-Mission-Computing/src/PX4-Autopilot ; make px4_sitl gazebo ; read" & ')

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
                        "fcu_url": "udp://:14540@127.0.0.1:14557",
                        "gcs_url": "",
                        "target_system_id": 1,
                        "target_component_id": 1,
                        "fcu_protocol": "v2.0",
                    }]
            ),

            # Launch px4 sitl (includes a function vs a node i think?)
            OpaqueFunction(
                function = launch_sitl
            ), 

    ]) # end LaunchDescription

    # Return launch description
    return launch_description
