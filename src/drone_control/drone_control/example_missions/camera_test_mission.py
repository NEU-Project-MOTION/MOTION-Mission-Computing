#
# 
#
import time
import rclpy
from drone_control.basic_drone import BasicDrone
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

# convert the ros image to OpenCV and display it
bridge = CvBridge()
last_image_time = 0
def handle_image(img_msg):
  # convert ROS image to OpenCV
  try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  except CvBridgeError as e:
    print(e)

  # draw a circle because why not
  (rows, cols, channels) = cv_image.shape
  cv2.circle(cv_image, center=(cols//2,rows//2), radius=10, color=(0,0,255), thickness=3)

  # calculate and draw FPS
  global last_image_time
  curr_time = img_msg.header.stamp.sec*1e9+img_msg.header.stamp.nanosec
  if last_image_time != 0 and curr_time-last_image_time != 0:
    cv2.putText(cv_image, f"FPS: {1e9/(curr_time-last_image_time):.0f}", org=(0,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(0,0,255), thickness=2)
  last_image_time = curr_time

  # display the image
  cv2.imshow("Realsense Image", cv_image)
  cv2.waitKey(3)


def main():
    rclpy.init() # init ros

    drone = BasicDrone() # init drone (drone is a ros node)
    drone.create_subscription(Image, "/drone/realsense/color/image_raw", handle_image, 10) # subscride to the realsense camera

    time.sleep(1) # let drone init

    drone.get_logger().info("Takeoff to 2.5m...")
    drone.arm_takeoff(altitude=2.5) # Takeoff

    # wait for user to click enter in terminal
    input("Press enter to land...")

    drone.get_logger().info("Landing...")
    drone.land() # Land

    drone.get_logger().info("Done!")

if __name__ == '__main__':
    main()