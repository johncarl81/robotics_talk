#! /usr/bin/env python3
import argparse
import subprocess
import signal
import time

def run_simulation(args):
    processes = []
    # Start Gazebo
    processes.append(subprocess.Popen("/entrypoint.sh ros2 launch gazebo_ros gazebo.launch.py gui:={} world:=worlds/empty_sky.world".format("{}".format(args.gui).lower()), shell=True))
    time.sleep(3)
    i = 0
    model = "/workspace/gazebo/src/example_sim/models/iris_with_standoffs/model.sdf"
    processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run gazebo_ros spawn_entity.py -file {model} -entity drone{i + 1}", shell=True))
    processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run example_sim arducopter.sh {i} drone{i+1}{i+1}", shell=True))
    processes.append(subprocess.Popen(f"/entrypoint.sh ros2 launch example_sim apm.launch.xml name:=drone{i+1} fcu_url:=udp://0.0.0.0:{14551 + (i * 10)}@{14555 + (i * 10)} tgt_system:={i + 1}", shell=True))
    # processes.append(subprocess.Popen(f"/entrypoint.sh ros2 run example_sim takeoff_service"))
    def shutdown(signum, frame):
        print('Exiting...')
        for p in processes:
            p.kill()
        exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.pause()

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def get_args():
    parser = argparse.ArgumentParser(description='Gazebo simulation')

    parser.add_argument(
        '--gui',
        type=str2bool,
        nargs='?',
        const=True,
        default=True)
    args = parser.parse_args()
    return args

def main():
    args = get_args()
    run_simulation(args)

if __name__ == '__main__':
    main()
