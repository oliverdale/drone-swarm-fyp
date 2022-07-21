"""
filename: common.py
author: Reka Norman
date: 16th October 2020
description:
    Common configuration constants and functions shared by both rx.py and tx.py.
"""

from gps import GPSCoord
import zmq
import struct
import json5
import rospy

import os


PARAMETERS_FILE = "flight_parameters_local.json"

# The swarming logic current assumes there are exactly four UAVs, so this must
# have the value 4.
NUM_RXS = 4

# The drone ID of the transmitter drone.
TX_ID = 0

# Controller Outputs
cntrl_strs = ['NOTHING','IN_POSITION','MOVE_TO_START','START_TRACKING','STOP_DRONES']
NOTHING = 0             # Sent when the drones should just keep doing what they're doing
IN_POSITION = 1         # Sent when the drones are manually placed in the correct position
MOVE_TO_START = 2       # Sent when the drones are ready to move to their start positions
START_TRACKING   = 3    # Sent when the drones are ready to start tracking
STOP_DRONES = 4         # Tells all the drones to move to the hold state

# Drone States
state_strs = ['ON','CONNECTED','OFFBOARD_WAIT','OFFBOARD','TRACKING','HOLD','FAIL']
ON_STATE = 0
CONECTED_STATE = 1
OFFBOARD_WAIT_STATE = 2
OFFBOARD_STATE = 3
READY_STATE = 4
TRACKING_STATE = 5
HOLD_STATE = 6
FAIL_STATE = 7



def get_parameters():
    """
    Reads the flight_parameters file to a dict and returns it
    """
    os.chdir(os.path.dirname(__file__))
    f = open(PARAMETERS_FILE)
    data = json5.load(f)
    return data

def update_loc(target, drone_num, drone_pos=None):
    """ returns the desired GPSCoord position of the drone in formation, 
    given the target GPSCoord, and drone ID. If target is -1, indicating an
    error, returns the drones current location """

    params = get_parameters()
    offset = params["COMMON"]["DRONE_DISTANCE"]

    if target.lat == -1 or target.long == -1:
        pos = drone_pos
        rospy.logwarn("no lat or long")
    else:
        if drone_num == 0:
            pos = target.add_x_offset(0).add_y_offset(0).add_z_offset(params["0"]["DEF_ALTITUDE"])

        elif drone_num == 1:
            pos = target.add_x_offset(-offset).add_y_offset(offset).add_z_offset(params["1"]["DEF_ALTITUDE"])

        elif drone_num == 2:
            pos = target.add_x_offset(offset).add_y_offset(offset).add_z_offset(params["2"]["DEF_ALTITUDE"])

        elif drone_num == 3:
            pos = target.add_x_offset(-offset).add_y_offset(-offset).add_z_offset(params["3"]["DEF_ALTITUDE"])

        elif drone_num == 4:
            pos = target.add_x_offset(offset).add_y_offset(-offset).add_z_offset(params["4"]["DEF_ALTITUDE"])

    return pos

def move_into_formation(context, controller, uav_id):
    """ Receives the desired formation centre position via a socket, and moves
    to the appropritate position in the formaiton based on its UAV ID.
    """
    input("\nUAV {}: Move to formation start position?\n".format(uav_id))

    # How close to the desired formation position should we get?
    threshold_m = 1

    flight_parameters = get_parameters()

    # Create socket to read the desired centre position
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://{}:{}".format(flight_parameters['COMMON']['TARGET_POS_IP_ADDRESS'], flight_parameters['COMMON']['TARGET_POS_PORT']))
    socket.setsockopt(zmq.SUBSCRIBE, b"")


    desired_position = GPSCoord(0, 0, controller.default_altitude)
    while controller.get_current_position().distance(desired_position) > threshold_m:
        # Receive a new centre position from the socket and calculate the UAV's
        # position in the formation based on its ID.
        rospy.loginfo("Receiving desired centre position from server...\n")
        rospy.loginfo(str(desired_position))
        centre_position = GPSCoord(*struct.unpack("ddd", socket.recv()))
        desired_position = update_loc(centre_position, uav_id, None)
        rospy.loginfo(str(centre_position))
        
        assert(desired_position is not None)
        
        #rospy.loginfo("UAV {:1d} X Distance: {:.2f}".format(controller.ID, controller.get_current_position().x_distance(desired_position)))
        #rospy.loginfo("UAV {:1d} Y Distance: {:.2f}".format(controller.ID, controller.get_current_position().y_distance(desired_position)))

        controller.reach_position(desired_position.lat, desired_position.long, desired_position.alt)

    socket.close()
    rospy.loginfo("\nUAV {}: In formation".format(uav_id))
