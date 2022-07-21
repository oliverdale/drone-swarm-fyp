"""
filename: server_gps.py
author: Reka Norman
date: 16th October 2020
description: Reads output from file of GPS data and converts into Decimal Degrees output.
    GPS co-ordinates are then sent to another device running client_gps/mavros_offboard_postctl/rx using ZeroMQ
"""
import zmq
import time
import struct
import argparse
from common import get_parameters
from gps import GPSCoord

# File to read the target coordinates from.
FILENAME = 'test.txt'#"gps_storage.txt"

# Period for sending the coordinates.
LOOP_PERIOD_S = 0.5



def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-l',
        '--local',
        action='store_true',
        help='specify to use the local flight parameters file')

    args = parser.parse_args()

    first_send = True
    second_send = True
    third_send = True
    # Initialise socket
    flight_parameters = get_parameters(args.local)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:{}".format(flight_parameters['COMMON']['TARGET_POS_PORT'])) # Must match the one in mavros_offboard_posctl.py and rx.py.

    # Open file
    storage = open(FILENAME, "r")
    gps_data = storage.readlines()

    # Read each line and transmit to client
    for i, line in enumerate(gps_data):
        loop_start_time = time.time()
        time_sample, lat_str, lon_str, alt_str = line.strip().split(",")
        coord = GPSCoord(float(lat_str), float(lon_str), float(alt_str))

        message = struct.pack("ddd", coord.lat, coord.long, coord.alt)
        print("Sending: {}".format(coord))
        socket.send(message)

        time.sleep(loop_start_time + LOOP_PERIOD_S - time.time())

        if first_send:
            first_send = False
            input("\nStarting position sent. Start sending more positions?\n")

        elif second_send:
            second_send = False
            input("\nStarting position sent. Start sending more positions?\n")

        elif third_send:
            third_send = False
            input("\nStarting position sent. Start sending more positions?\n")

    storage.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted. Exiting...")
