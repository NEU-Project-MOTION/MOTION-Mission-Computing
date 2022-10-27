# MOTION-Mission-Computing
Repository for MOTION's companion computer

## Requirements
- PX4 Install: `bash ./src/PX4-Autopilot/Tools/setup/ubuntu.sh`
- ROS Install: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Colcon?: `sudo apt install python3-colcon-common-extensions`
- Gazebo: `curl -sSL http://get.gazebosim.org | sh`
- Gazebo ROS: `sudo apt install ros-foxy-gazebo-ros-pkgs`
- MavROS: `sudo apt install ros-foxy-mavros ros-foxy-mavros-extras`
- MavROS Fix: `sudo geographiclib-get-geoids egm96-5`
- XTerm: `sudo apt install xterm`

### Optional
- ZSH Autocomplete: `eval "$(register-python-argcomplete3 ros2)"`
- Realsense: `ros-foxy-realsense2-camera`
- Realsense models: `sudo apt-get install ros-foxy-realsense2-description`
- Cartographer: `sudo apt install ros-foxy-cartographer-ros`
- Camera calibration stuff: `sudo apt install ros-foxy-camera-calibration ros-foxy-camera-calibration-parsers ros-foxy-camera-info-manager ros-foxy-launch-testing-ament-cmake`
- PCL ROS: `sudo apt install ros-foxy-pcl-ros`

# Notes
- To clone run `git clone git@github.com:NEU-Project-MOTION/MOTION-Mission-Computing.git --recurse-submodules`
- To build run `./scripts/build.py -p` (omit the `-p` after first build)
- On a first build of PX4 run `param set NAV_RCL_ACT 0` 
- The Jetson Nano's IP over USB is `192.168.55.100`
- Jetson slow AF wifi fix: `sudo iw dev wlan0 set power_save off`
- `donatello@nuav`
- Run realsense publisher: `ros2 launch realsense2_camera rs_launch.py`
- To convert realsense to laser scan (2D): `ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args -r depth:=/camera/depth/image_rect_raw -r depth_camera_info:=/camera/depth/camera_info -r scan:=/camera/scan`
- Camera calibration: [Docs here](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- Checkerboard generator: [Website](https://calib.io/pages/camera-calibration-pattern-generator)
- Run T625: `ros2 launch realsense2_camera rs_launch.py tf_publish_rate:=20 pose_enabled:=true pose_fps:=200 publish_odom_tf:=true publish_tf:=true`

