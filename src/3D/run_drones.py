""" Runs multiple drones simultaneously in separate processes.
    Use --help for a description of the available command-line options.
"""

import sys
import subprocess
from multiprocessing import Process
import argparse
from common import *

def id_set(string):
    ids = set()
    for c in string:
        if c.isdigit() and int(c) in range(NUM_RXS + 1):
            ids.add(int(c))
        else:
            raise argparse.ArgumentTypeError(
                '{} is not a valid string of UAV IDs'.format(string))
    return ids


def main():
    parser = argparse.ArgumentParser(
        description='Runs a Tx and multiple Rx drones (or a subset of these) \
        in different processes.')
   
    parser.add_argument(
        '-u',
        '--uavs',
        type=id_set,
        default=set(range(NUM_RXS + 1)),
        help='IDs of the UAVs to run, e.g. 023 runs UAVs 0, 2 and 3')
    parser.add_argument(
        '-l',
        '--local',
        action='store_true',
        help='specify to use the local flight parameters file')
    args = parser.parse_args()

    processes = []

    if TX_ID in args.uavs:
        tx_args = []
        if args.local:
            tx_args.append('--local')
        processes.append(
            Process(target=subprocess.run,
                    args=(["python3", "tx.py"] + tx_args, )))

    for rx_id in range(1, NUM_RXS + 1):
        if rx_id in args.uavs:
            rx_args = [str(rx_id)]
            if args.local:
                rx_args.append('--local')
            processes.append(
                Process(target=subprocess.run,
                        args=(["python3", "rx.py"] + rx_args, )))

    for process in processes:
        process.start()

    # Run forever, or until interrupted.
    try:
        for process in processes:
            process.join()
    except KeyboardInterrupt:
        print("Interrupted, terminating processes...")
        for process in processes:
            process.terminate()
        for process in processes:
            process.join()
        sys.exit()

if __name__ == '__main__':
    main()
    sys.exit()
