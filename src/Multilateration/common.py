"""
filename: common.py
author: Reka Norman
date: 16th October 2020
description:
    Common configuration constants and functions shared by both rx.py and tx.py.
"""

from gpsv2 import GPSCoord
import swarming_logic
import zmq
import struct


# The swarming logic current assumes there are exactly four UAVs, so this must
# hav-e the value 4.
NUM_RXS = 4

# IP address and port numbers of the machine running tx.py
#TX_IP_ADDRESS = "10.42.0.108"
TX_IP_ADDRESS = "localhost"
TX_STARTUP_PORT = 5550
TX_RECEIVE_PORT = 5551
TX_SEND_PORT = 5552

# IP address and port number of the server sending target coordinates (running
# server_data.py or server_gps.py).
#TARGET_POS_IP_ADDRESS ="10.42.0.108"
TARGET_POS_IP_ADDRESS = "localhost"
TARGET_POS_PORT = 5556

# The drone ID of the transmitter drone.
TX_ID = 0

# UAV and Target Altitudes
RX_ALTITUDE = 20
TX_ALTITUDE = 20
TARGET_ALTITUDE = 10


def move_into_formation(context, controller, uav_id):
    """ Receives the desired formation centre position via a socket, and moves
    to the appropritate position in the formaiton based on its UAV ID.
    """
    input("\nUAV {}: Move to formation start position?\n".format(uav_id))

    # How close to the desired formation position should we get?
    threshold_m = 1

    # Create socket to read the desired centre position
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://{}:{}".format(TARGET_POS_IP_ADDRESS, TARGET_POS_PORT))
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    desired_position = GPSCoord(0, 0)
    while controller.get_current_position().distance(desired_position) > threshold_m:
        # Receive a new centre position from the socket and calculate the UAV's
        # position in the formation based on its ID.
        print("Receiving desired centre position from server...\n")
        centre_position = GPSCoord(*struct.unpack("dd", socket.recv()))
        desired_position = swarming_logic.update_loc(centre_position, uav_id, None)
        assert(desired_position is not None)

        controller.reach_position(desired_position.lat, desired_position.long)

    socket.close()
    print("\nUAV {}: In formation".format(uav_id))
