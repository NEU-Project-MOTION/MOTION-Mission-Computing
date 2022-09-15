#
# Use with start_single_sim.launch.py
#
import rclpy
import time
from drone_control.basic_drone import BasicDrone

def main():
    rclpy.init() # init ros

    drone = BasicDrone() # init drone (drone is a ros node)

    drone.get_logger().info("Takeoff to 2.5m...")
    drone.arm_takeoff(altitude=2.5) # Takeoff

    input("Press enter to continue...")

    drone.get_logger().info("Moving 1m east...")
    drone.set_local_target(target_east=1, target_north=0, target_up=2.5, target_yaw=0)
    drone.enable_offboard() # enable "offboard" control mode (aka autonomous mode)
    time.sleep(5)

    input("Press enter to continue...")

    drone.get_logger().info("Rotating 90...")
    drone.set_local_target(target_east=1, target_north=0, target_up=2.5, target_yaw=90)
    time.sleep(5)

    input("Press enter to continue...")

    drone.get_logger().info("Moving back...")
    drone.set_local_target(target_east=0, target_north=0, target_up=2.5, target_yaw=0) # move back
    time.sleep(5)

    input("Press enter to continue...")

    drone.get_logger().info("Landing...")
    drone.land() # Land

    drone.get_logger().info("Done!")

if __name__ == '__main__':
    main()