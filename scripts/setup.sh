#!/bin/bash
# Sets up terminal environment to recognize our code
# Also registers some custom commands (aliases)

CRACK_WS="$( cd -- "$(dirname "${BASH_SOURCE[0]-$0}")" >/dev/null 2>&1 ; cd .. ; pwd -P )" 

# Source ROS and our code if it exists
source "/opt/ros/foxy/setup.sh" 
if [ -f "$CRACK_WS/install/setup.sh" ]; then source "$CRACK_WS/install/setup.sh"; fi


# Tell Gazebo where all our custom models/worlds are
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$CRACK_WS/src/crack_gazebo/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$CRACK_WS/src/crack_gazebo/worlds
source $CRACK_WS/src/PX4-Autopilot/Tools/setup_gazebo.bash $CRACK_WS/src/PX4-Autopilot $CRACK_WS/src/PX4-Autopilot/build/px4_sitl_default > /dev/null


# Make ROS output pretty colors
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{function_name}():{line_number}]: {message}"


### Aliases (custom commands)
alias sim="ros2 launch drone_control start_single_sim.launch.py"
alias killros2='killall -9 gzserver gzclient _ros2_daemon px4 ros2'
