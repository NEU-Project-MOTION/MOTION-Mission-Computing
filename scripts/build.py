#!/usr/bin/python3
# Builds ros packages and px4

import os
import argparse

HEADER = ''
OKGREEN = ''
WARNING = ''
FAIL = ''
ENDC = ''

def print_green(str):
    print(f"\033[95m\033[92m{str}\033[0m")

def print_orange(str):
    print(f"\033[95m\033[93m{str}\033[0m")

def print_red(str):
    print(f"\033[95m\033[91m{str}\033[0m")

def parse_args():
    ''' Parse command line arguments '''
    parser = argparse.ArgumentParser(description="Builds all neccesary code. Only builds ROS by default")

    parser.add_argument('-p', "--px4", dest="build_px4", action="store_true", default=False, help="Build PX4")

    return parser.parse_args()


def main(args):
    # Get crack_ws location
    ws_location = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Build ros2 packages
    ret = os.system("colcon build --symlink-install --packages-skip px4")
    if ret != 0:
        print_red("ROS Failed to build")
    else:
        print_green("ROS build successfully!")

    # Build px4_sitl if building for sim
    if args.build_px4:
        print_green("Building PX4...")
        ret = os.system(f'cd {ws_location}/src/PX4-Autopilot; DONT_RUN=1 make px4_sitl gazebo')
        if ret != 0:
            print_red("PX4 Failed to build")
        else:
            print_green("PX4 build successfully!")
            print_green("Run \"source scripts/setup.sh\" if you added new modules")


if __name__ == "__main__":
    args = parse_args()
    main(args)
