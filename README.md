# MOTION-Mission-Computing
Repository for MOTION's companion computer

## Requirements
- PX4 Install: `bash ./src/PX4-Autopilot/Tools/setup/ubuntu.sh`
- Colcon?: `sudo apt install python3-colcon-common-extensions`
- ROS Install: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Gazebo: `curl -sSL http://get.gazebosim.org | sh`
- Gazebo ROS: `sudo apt install ros-foxy-gazebo-ros-pkgs`
- MavROS: `sudo apt install ros-foxy-mavros`
- XTerm: `sudo apt install xterm`
- MavROS Fix: `sudo geographiclib-get-geoids egm96-5`
- ZSH Autocomplete: `eval "$(register-python-argcomplete3 ros2)"`
- Cartographer: `sudo apt install ros-foxy-cartographer-ros`


# Notes
- The Jetson Nano's IP over USB is 192.168.55.100
- On a first build of PX4 run `param set NAV_RCL_ACT 0` 
- To clone run `git clone git@github.com:NEU-Project-MOTION/MOTION-Mission-Computing.git --recurse-submodules`
To build run `./scripts/build.py -p` (omit the `-p` after first build)