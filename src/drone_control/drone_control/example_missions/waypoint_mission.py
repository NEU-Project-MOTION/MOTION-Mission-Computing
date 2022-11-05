#!/usr/bin/env python3

#
# Use with start_single_sim.launch.py
#
import rclpy
from drone_control.basic_drone import BasicDrone


LOOP = True
WAYPOINTS = [
    (10, 0, 2.5),
    (10, 10, 2.5),
    (0, 10, 2.5),
    (0, 0, 2.5),
    ]


def main():
    rclpy.init() # init ros

    drone = BasicDrone() # init drone (drone is a ros node)

    try:
        # Fly
        drone.get_logger().info(f"Takeoff to {max(WAYPOINTS[0][2], 2.5)}m...")
        drone.arm_takeoff(altitude=WAYPOINTS[0][2]) # Takeoff

        drone.enable_offboard()

        # Waypoints
        while True:
            for i, waypoint in enumerate(WAYPOINTS):
                drone.get_logger().info(f"Flying to waypoint {i}")
                drone.go_to_position(*waypoint)

            # break if we are not looping
            if not LOOP:
                break
        
        # Land
        drone.get_logger().info("Landing...")
        drone.land() 
        drone.get_logger().info("Done!")

    except KeyboardInterrupt:
        drone.get_logger().info("CTRL-C caught, Landing...")
        drone.land() # Land


if __name__ == '__main__':
        main()
