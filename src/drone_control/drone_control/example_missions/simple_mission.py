#
# Use with start_single_sim.launch.py
#
import rclpy
import time
from drone_control.basic_drone import BasicDrone

def main():
    rclpy.init() # init ros

    drone = BasicDrone() # init drone (drone is a ros node)
    time.sleep(1) # let drone init

    drone.get_logger().info("Takeoff to 5m...")
    drone.arm_takeoff(altitude=5) # Takeoff

    drone.get_logger().info("Moving 4m east...")
    drone.set_local_target(target_east=4, target_north=0, target_up=5, target_yaw=90) # move 4 meters east 
    drone.enable_offboard() # enable "offboard" control mode (aka autonomous mode)
    time.sleep(5)

    input("Press enter to continue...")

    drone.get_logger().info("Moving back...")
    drone.set_local_target(target_east=1, target_north=0, target_up=5, target_yaw=0) # move back
    time.sleep(5)

    drone.get_logger().info("Landing...")
    drone.land() # Land

    drone.get_logger().info("Done!")

if __name__ == '__main__':
    main()