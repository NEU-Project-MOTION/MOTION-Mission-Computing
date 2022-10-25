import os
import sys
import jinja2
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration




# 
# File to launch a single frog on franklin field
#

# https://github.com/ros2/launch_ros/tree/master/launch_ros/launch_ros/actions
# https://github.com/ros2/launch/tree/master/launch/launch/actions

namespace = "drone"

""" Code to launch the sim (SITL) """
def launch_sitl(context, *args, **kwargs):
    build_path = "$HOME/MOTION-Mission-Computing/src/PX4-Autopilot/build/px4_sitl_default"
    log_path = build_path + "/froglog"

    # Create a log folder for logging if it doesn't already exist
    os.system(f'[ ! -d "{log_path}" ] && mkdir -p "{log_path}"')

    # Set PX4 environment variables
    os.environ["PX4_SIM_MODEL"] = "quadrotor_x"
    os.environ["PX4_ESTIMATOR"] = "ekf2"

    # Start px4 simulator by itself, passing it stuff like where to log stuff
    os.system(f'xterm -e "{build_path}/bin/px4 {build_path}/etc -i 0 -w {log_path} -s {build_path}/etc/init.d-posix/rcS ; read" & ')


""" Build the drone model (jinja changes properties of our drone model file) """
def parse_jinja_model(name):
    file_path = f'{get_package_share_directory("crack_gazebo")}/models/robots/frog_v2/model.sdf.jinja'
    # Properties of the frog model to change
    mappings = {"namespace": name,
                "mavlink_tcp_port": 4560,
                "mavlink_udp_port": 14560,
                "serial_enabled": "0", # For HITL
                "serial_device": "", # For HITL
                "serial_baudrate": "", # For HITL
                "hil_mode": "0", # For HITL
                "include_camera": True,
                "camera_tilt": 0.0}

    tmp_path = f'/tmp/{name}.sdf'
    templateFilePath = jinja2.FileSystemLoader(os.path.dirname(file_path))
    jinja_env = jinja2.Environment(loader=templateFilePath)
    j_template = jinja_env.get_template(os.path.basename(file_path))
    output = j_template.render(mappings)
    with open(tmp_path, 'w') as sdf_file:
        sdf_file.write(output)
    return tmp_path




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

            # Launch px4 sitl (includes a function vs a node i think?)
            OpaqueFunction(
                function = launch_sitl
            ), 
            
            # Launch the Gazebo server (this is in charge of doing all the actual physics)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('gazebo_ros'),
                        'launch/gzserver.launch.py')
                    ),
                    launch_arguments={
                        'world': "franklin_park_cluttered.world",
                        'verbose': "true", 
                    }.items(),

            ),
            
            # Start the Gazebo client (the GUI you see which handles the rendering)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('gazebo_ros'),
                        'launch/gzclient.launch.py')
                    ),
            ),

            # Spawn the drone in gazebo
            Node(
                package="gazebo_ros", executable="spawn_entity.py",
                arguments=[
                    "-entity", namespace, "-file", parse_jinja_model(namespace),
                    "-robot_namespace", namespace,
                    "-spawn_service_timeout", "120.0",
                    "-x", "0", "-y", "0", "-z", "0.15",
                ],
                output="screen",
            )



    ]) # end LaunchDescription

    # Return launch description
    return launch_description
