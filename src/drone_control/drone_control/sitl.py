#!/usr/bin/env python
"""
File to start PX4 SITL with given arguments.
"""
import os
import textwrap
from argparse import ArgumentParser, RawDescriptionHelpFormatter


def get_arguments():
    parser = ArgumentParser(prog='sitl',
    formatter_class=RawDescriptionHelpFormatter,
    epilog=textwrap.dedent('''\
        Starts PX4 SITL.
        '''))
    parser.add_argument("-i", "--instance", default=0, type=int, help="Instance of vehicle.")
    parser.add_argument("-m", "--model", default="iris", type=str, help="Vehicle model to start.")
    parser.add_argument("-d", "--log-directory", default=f"{os.environ['HOME']}/.ros", type=str, help="Directory to store log files.")
    parser.add_argument("-b", "--build-path", default=f"$HOME/MOTION-Mission-Computing/src/PX4-Autopilot/build/px4_sitl_default", type=str, help="Path to PX4 build files.")
    parser.add_argument("-g", "--gui", default=True, type=bool, help="Show PX4 shell gui.")
    args, _ = parser.parse_known_args()
    return args


def main():
    # Arguments
    args = get_arguments()

    # Setups variables
    i = args.instance
    model = args.model
    log_directory = args.log_directory
    build_path = args.build_path

    # Create folder for storing vehicle data
    os.system(f'[ ! -d "{log_directory}" ] && mkdir -p "{log_directory}"')

    # Set PX4 environment variables
    os.environ["PX4_SIM_MODEL"] = model
    os.environ["PX4_SIM_SPEED_FACTOR"] = "1.0" # TODO: this sets COM_OF_LOSS_T to 0.5, setting it higher does not turnoff failsafe enabled problem
    os.environ["PX4_ESTIMATOR"] = "ekf2"

    # Launch PX4 SITL
    if args.gui:
        os.system(f'xterm -e "{build_path}/bin/px4 {build_path}/etc -i {i} -w {log_directory}/sitl_{model}_{i} -s {build_path}/etc/init.d-posix/rcS ; read" & ')
    else:
        os.system(f'{build_path}/bin/px4 -i {i} -d "{build_path}/etc" -w {log_directory}/sitl_{model}_{i} -s {build_path}/etc/init.d-posix/rcS >{log_directory}/sitl_out.log 2>{log_directory}/sitl_err.log')


if __name__=="__main__":
    main()
