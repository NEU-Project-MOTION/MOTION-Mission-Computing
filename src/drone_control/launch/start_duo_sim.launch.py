import os
import jinja2
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 
# File to launch two frogs on franklin field
#

# https://github.com/ros2/launch_ros/tree/master/launch_ros/launch_ros/actions
# https://github.com/ros2/launch/tree/master/launch/launch/actions


""" Build the drone model (jinja changes properties of our drone model file) """
def parse_jinja_model(name, i):
    file_path = f'{get_package_share_directory("crack_gazebo")}/models/robots/frog_v2/model.sdf.jinja'
    # Properties of the frog model to change
    mappings = {"namespace": name,
                "mavlink_tcp_port": 4560 + i,
                "mavlink_udp_port": 14560 + i,
                "serial_enabled": "0", # For HITL
                "serial_device": "", # For HITL
                "serial_baudrate": "", # For HITL
                "hil_mode": "0", # For HITL
                "include_camera": True,
                "camera_tilt": 0.0}

    tmp_path = f'/tmp/{name}{i}.sdf'
    templateFilePath = jinja2.FileSystemLoader(os.path.dirname(file_path))
    jinja_env = jinja2.Environment(loader=templateFilePath)
    j_template = jinja_env.get_template(os.path.basename(file_path))
    output = j_template.render(mappings)
    with open(tmp_path, 'w') as sdf_file:
        sdf_file.write(output)
    return tmp_path



""" Generates launch sequence for vehicle"""
def generate_vehicle(namespace, i):
    return [
            # Launch the MAVROS node
            Node(
                package="mavros", 
                executable="mavros_node",
                output="screen",
                namespace=f"{namespace}/mavros",
                parameters=[{
                        "fcu_url": f"udp://:{14540+i}@127.0.0.1:{14557+i}",
                        "gcs_url": "",
                        "target_system_id": 1+i,
                        "target_component_id": 1,
                        "fcu_protocol": "v2.0",
                    }]
            ),

            # Run PX4 SITL
            Node(
                package='drone_control', executable="sitl",
                output='screen',
                namespace=namespace,
                arguments=[
                    "--instance", str(i),
                ],
            ),

            # Spawn the drone in gazebo
            Node(
                package="gazebo_ros", executable="spawn_entity.py",
                arguments=[
                    "-entity", namespace, "-file", parse_jinja_model(namespace, i),
                    "-robot_namespace", namespace,
                    "-spawn_service_timeout", "120.0",
                    "-x", "0", "-y", f"{i*2}", "-z", "0.15",
                ],
                output="screen",
            ),
    ]





""" Main launch function that ROS2 will run when we use `ros2 launch ...` """
def generate_launch_description():
    # Create a launch description to start our nodes
    launch_description = [
            # Launch the Gazebo server (this is in charge of doing all the actual physics)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('gazebo_ros'),
                        'launch/gzserver.launch.py')
                    ),
                    launch_arguments={
                        'world': "franklin_park.world",
                        'verbose': "false", 
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


    ] # end LaunchDescription

    launch_description += generate_vehicle("drone_0", 0)
    launch_description += generate_vehicle("drone_1", 1)

    # Return launch description
    return LaunchDescription(launch_description)
