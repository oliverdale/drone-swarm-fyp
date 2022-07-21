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
from gps import GPSCoord


# Must match the one in mavros_offboard_posctl.py and rx.py.
PORT = 5556

# File to read the target coordinates from.
FILENAME = "gps_storage.txt"

# Period for sending the coordinates.
LOOP_PERIOD_S = 1


def main():
    # Initialise socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:{}".format(PORT))

    # Open file
    storage = open(FILENAME, "r")
    gps_data = storage.readlines()

    # Read each line and transmit to client
    for line in gps_data:
        loop_start_time = time.time()
        time_sample, lat_str, lon_str = line.strip().split(",")
        coord = GPSCoord(float(lat_str), float(lon_str))

        message = struct.pack("dd", coord.lat, coord.long)
        print("Sending: {}".format(coord))
        socket.send(message)

        time.sleep(loop_start_time + LOOP_PERIOD_S - time.time())

    storage.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted. Exiting...")
