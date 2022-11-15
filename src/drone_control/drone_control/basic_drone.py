#
# Base class for an drone control, based off of NUAV's code. This is intended to provide functions that all missions will need .
#

import threading
import time
import math
from typing import Any
from scipy.spatial.transform import Rotation
# rclpy (Ros Client Library PYthon)
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
# ROS Messages
from rcl_interfaces.srv import SetParameters
from mavros_msgs.srv import CommandTOL, CommandBool, CommandLong
from mavros_msgs.msg import Altitude, State, PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion

# Base mission class which extends a ROS2 node
class BasicDrone(Node):

    def __init__(self, namespace="drone", node_name="drone_control_node"):
        """ Initialiser to be called in sub-classes

        Args:
            namespace (string, optional): The namespace of the ros node. Defaults to 'drone'
            node_name (string, optional): The name of the ros node. Defaults to 'drone_control_node'
        """
        super().__init__(node_name=node_name, namespace=namespace) # Init ros2 node

        ## Public vars
        self.altitude: Altitude = None
        self.current_pose: PoseStamped = None 
        self.mavros_state: State = None 

        ## Private vars
        self._local_target_pos: PositionTarget = None

        # ROS2 service clients (commands to mavros usually)
        self._arm_srv = self.create_client(CommandBool, "mavros/cmd/arming")
        self._takeoff_srv = self.create_client(CommandTOL, "mavros/cmd/takeoff")
        self._land_srv = self.create_client(CommandTOL, "mavros/cmd/land")
        self._command_srv = self.create_client(CommandLong, "mavros/cmd/command")
        self._param_set_srv = self.create_client(SetParameters, "mavros/param/set_parameters")

        # ROS2 publishers (data sent to the drone such as target position)
        self._local_pos_pub = self.create_publisher(PositionTarget, "mavros/setpoint_raw/local", 10)

        # ROS2 subscribers (data read from the drone such as altitude)
        self._altitude_sub = self.create_subscription(Altitude, "mavros/altitude", self.altitude_cb, qos_profile_sensor_data)
        self._altitude_sub = self.create_subscription(PoseStamped, "mavros/local_position/pose", self.pose_cb, qos_profile_sensor_data)
        self._altitude_sub = self.create_subscription(State, "mavros/state", self.mavstate_cb, qos_profile_sensor_data)

        # Start asyncronous stuff such as ROS spinner and offboard position publishing
        threading.Thread(target=self._spin_async, daemon=True).start()
        threading.Thread(target=self._publish_position, daemon=True).start()

        # Wait for MAVROS to start
        while(rclpy.ok() and self.mavros_state is None):
            self.get_logger().info("Waiting for MAVROS...", throttle_duration_sec=1, skip_first=True)

        while(rclpy.ok() and not self.mavros_state.connected):
            self.get_logger().info("Waiting for PX4...", throttle_duration_sec=1, skip_first=True)


    ##############
    # API functions
    ##############


    def set_param(self, param_name: str, param_value: Any, param_type:rclpy.parameter.Parameter.Type=rclpy.parameter.Parameter.Type.INTEGER):
        """Sets a parameter on the drone

        Args:
            param_name (string): The name of the parameter to set
            param_value (int): The value to set the parameter to
        """
        # check if the service is ready
        ready = self._param_set_srv.wait_for_service(timeout_sec=5.0)
        if not ready:
            self.get_logger().error(f'MAVROS param service call timed out ({param_name})')
            return

        # call the param service
        param = rclpy.parameter.Parameter(param_name, param_type, param_value)

        param_set = SetParameters.Request()
        param_set.parameters = [param.to_parameter_msg(), ]
        self._param_set_srv.call(param_set)


    # TODO do nothing if already flying
    def arm_takeoff(self, altitude=2.5, blocking=True):
        """Attempts to arm and takeoff (blocks until reaches target altitude)

        Args:
            altitude (float, optional): Target altitude in meters [2.5,inf). Defaults to 2.5m.
            blocking (bool, optional): Whether to block untill finished taking off. Defaults to True.

        Returns:
            boolean: Whether the action was successful
        """
        #Verify arguments
        if(altitude < 2.5):
            self.get_logger().error(f"Failed to takeoff (min altitude is 2.5m)")
            return False

        # fail if we cant read MAVROS altitude
        if(self.altitude is None):
            self.get_logger().error(f"Failed to takeoff (MAVROS altitude not set)")
            return False

        # request arm through MAVROS service
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_responce = self._arm_srv.call(arm_request)

        if not arm_responce.success:
            self.get_logger().error(f"Failed to arm before takeoff (MAV_RESULT={arm_responce.result})")
            return False # Failure

        # request takeoff
        takeoff_request = CommandTOL.Request()
        takeoff_request.altitude = float(altitude+self.altitude.amsl) # Set target altitude (convert to above sea level)
        takeoff_request.latitude = float("nan") # nan tells it to use current lat/lon/yaw
        takeoff_request.longitude = float("nan")
        takeoff_request.yaw = float("nan")
        takeoff_responce = self._takeoff_srv.call(takeoff_request)

        if not takeoff_responce.success:
            self.get_logger().error(f"Failed to takeoff, dissarming (MAV_RESULT={takeoff_responce.result})")
            arm_request.value = False # Attempt to dissarm if takeoff fails after arming was successful
            self._arm_srv.call(arm_request)
            return False # Failure

        # block while taking off
        while(rclpy.ok() and blocking and self.altitude.local < altitude-0.2):
            pass

        return True # Success


    # TODO fix land not proceeding if already landed
    def land(self):
        """Land at the current location (blocks until landed)

        Returns:
            boolean: Whether or not the land was successful
        """

        land_request = CommandTOL.Request()
        land_request.latitude = float("nan") # nan tells it to use current lat/lon/yaw
        land_request.longitude = float("nan")
        land_request.yaw = float("nan")
        land_responce = self._land_srv.call(land_request)

        if not land_responce.success:
            self.get_logger().error(f"Failed to land (MAV_RESULT={land_responce.result})")
            return False # Failure

        # Block until landed (and dissarmed)
        while rclpy.ok() and self.mavros_state.armed:
            pass

        return True # Success


    # TODO look into why mavros/set_mode doesn't work
    def enable_offboard(self, set_target_current_position=True):
        """Sets the drone into OFFBOARD mode such that it starts moving to the target position set via `set_local_target`
        """

        # set initial target
        if set_target_current_position:
            self.set_local_target(self.current_pose.pose.position.x,
                                  self.current_pose.pose.position.y,
                                  self.current_pose.pose.position.z)

        # force publish a setpoint now to allow offboard swap
        self._local_pos_pub.publish(self._local_target_pos)

        # change mode
        command_request = CommandLong.Request()
        command_request.command = int(176) #MAV_CMD_DO_SET_MODE
        command_request.param1 = float(209.0) #OFFBOARD, apparently
        command_request.param2 = float(6.0)

        self._command_srv.call(command_request)


    def set_local_target(self, target_east, target_north, target_up, target_yaw=0):
        """Sets the target local position in ENU relative to the home position (non-blocking)

        Args:
            target_east (float): Meters east 
            target_north (float): Meters north
            target_up (float): Meters up
            target_yaw (float): Rotation in Degrees
        """
        # first time setup
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NEDZ
        if self._local_target_pos is None:
            self._local_target_pos = PositionTarget()
            self._local_target_pos.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            self._local_target_pos.type_mask = 0b100111111000 # ignore all but pos and yaw

        # set target
        self._local_target_pos.position.x = float(target_east)
        self._local_target_pos.position.y = float(target_north)
        self._local_target_pos.position.z = float(target_up)

        self._local_target_pos.yaw = float(target_yaw)


    def go_to_position(self, east: float, north: float, up: float, yaw: float=0, print_distance=False, reached_radius:float=1):
        """Flys the drone to a NAU position, blocking until it gets there

        Args:
            east (float): Meters east 
            north (float): Meters north
            up (float): Meters up
            yaw (float): Rotation in Degrees
            reached_radius (float): Distance in m from target position to consider it reached
        """

        self.set_local_target(east, north, up, yaw)

        # block until we reach the target position
        while(rclpy.ok() and self.distance_to_target() > reached_radius):
            if print_distance:
                self.get_logger().info(f"Distance to target: {self.distance_to_target()}")
            time.sleep(0.1) # Check at 10Hz


    def distance_to_target(self):
        """Returns the distance to the target position in meters

        Returns:
            float: Distance to target in meters
        """
        return math.sqrt(
            (self._local_target_pos.position.x-self.current_pose.pose.position.x)**2 + 
            (self._local_target_pos.position.y-self.current_pose.pose.position.y)**2 + 
            (self._local_target_pos.position.z-self.current_pose.pose.position.z)**2)        


    def get_mode(self):
        """Gets the drone's mode

        Returns:
            string: The current mode (see mavros_msgs.msg.State). Valid modes: 
                        MANUAL, ACRO, ALTCTL, POSCTL, OFFBOARD, STABILIZED, 
                        RATTITUDE, AUTO.MISSION, AUTO.LOITER, AUTO.RTL, 
                        AUTO.LAND, AUTO.RTGS, AUTO.READY, AUTO.TAKEOFF
        """
        return self.mavros_state.mode


    ##############
    # ROS Callbacks
    ##############

    def altitude_cb(self, alt_msg):
        self.altitude = alt_msg

    def pose_cb(self, pose_msg):
        self.current_pose = pose_msg

    def mavstate_cb(self, state_msg):
        self.mavros_state = state_msg


    ##############
    # Misc
    ##############

    def _spin_async(self):
        """Private function to spin ROS (this lets it check for callbacks)"""
        rclpy.spin(self, executor=MultiThreadedExecutor())
  
    # TODO make this a timer callback (?)
    def _publish_position(self):
        """Publish the desired target position to the drone"""
        # Run at 30Hz
        r = self.create_rate(30)

        while(rclpy.ok()):
            r.sleep()
            if self._local_target_pos is not None:
                self._local_target_pos.header.stamp = self.get_clock().now().to_msg()
                self._local_pos_pub.publish(self._local_target_pos)
