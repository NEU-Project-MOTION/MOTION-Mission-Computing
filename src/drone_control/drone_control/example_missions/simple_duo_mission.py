#!/usr/bin/env python3

#
# Use with start_duo_sim.launch.py
#
import rclpy
import time
from drone_control.basic_drone import BasicDrone

def main():
    rclpy.init() # init ros

    drone_0 = BasicDrone(namespace="drone_0") # init drone (drone is a ros node)
    drone_1 = BasicDrone(namespace="drone_1") # init drone (drone is a ros node)
    time.sleep(1) # let drone init

    drone_0.get_logger().info("Takeoff to 5m...")
    drone_0.arm_takeoff(altitude=5) # Takeoff
    drone_1.arm_takeoff(altitude=5)

    drone_0.get_logger().info("Moving 4m east...")
    drone_0.set_local_target(target_east=4, target_north=0, target_up=5, target_yaw=90) # move 4 meters east 
    drone_0.enable_offboard() # enable "offboard" control mode (aka autonomous mode)
    drone_1.set_local_target(target_east=4, target_north=0, target_up=5, target_yaw=90) # move 4 meters east 
    drone_1.enable_offboard() # enable "offboard" control mode (aka autonomous mode)
    time.sleep(5)

    drone_0.get_logger().info("Moving back...")
    drone_0.set_local_target(target_east=0, target_north=0, target_up=5, target_yaw=0) # move back
    drone_1.set_local_target(target_east=0, target_north=0, target_up=5, target_yaw=0)
    time.sleep(5)

    drone_0.get_logger().info("Landing...")
    drone_0.land() # Land
    drone_1.land()

    drone_0.get_logger().info("Done!")

if __name__ == '__main__':
    main()