#!/usr/bin/python3
# Cleans the repo, removing all compiled code

import os
import argparse
import shutil


def print_green(str):
    print(f"\033[95m\033[92m{str}\033[0m")


def parse_args():
    ''' Parse command line arguments '''
    parser = argparse.ArgumentParser(description="Builds all neccesary code. Only builds ROS by default")

    parser.add_argument('-p', "--px4", dest="clean_px4", action="store_true", default=False, help="Clean PX4")

    return parser.parse_args()


def main(args):
    # Get MOTION-Mission-Computing location
    ws_location = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Cleam ros2 packages
    shutil.rmtree(f'{ws_location}/build', ignore_errors=True)
    shutil.rmtree(f'{ws_location}/install', ignore_errors=True)
    shutil.rmtree(f'{ws_location}/log', ignore_errors=True)

    # Clean px4
    if args.clean_px4:
        os.system(f'cd {ws_location}/src/PX4-Autopilot; make clean; rm -rf build')

    print_green("Cleaned!")

if __name__ == "__main__":
    args = parse_args()
    main(args)
