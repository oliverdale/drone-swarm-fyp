"""
filename: common.py
author: Rowan Sinclair
date: 30th September 2021
description:
    Common configuration constants and functions shared by both rx.py and tx.py.
"""

from gps import GPSCoord
import json5
import rospy

import os

PARAMETERS_FILE_LOCAL = "flight_parameters_local.json"
PARAMETERS_FILE = "flight_parameters.json"

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
ON_STATE = 0            # When the drones first turn on
CONECTED_STATE = 1      # Move to this state one the connections between the drones are successfully established
OFFBOARD_WAIT_STATE = 2 # Waiting forthe pilot to switch to Offboard Mode
OFFBOARD_STATE = 3      # Moving to the correct starting position
READY_STATE = 4         # In the correct start position, ready to start tracking once commanded
TRACKING_STATE = 5      # Tracking the target
HOLD_STATE = 6          # Somthing has gone wrong. Hold position
FAIL_STATE = 7


def get_parameters(is_local):
    """
    Reads the flight_parameters json file to a dict and returns it
    """
    if is_local:
        filename = PARAMETERS_FILE_LOCAL
    else:
        filename = PARAMETERS_FILE

    os.chdir(os.path.dirname(__file__)) # set the working directory to this file's directory
    f = open(filename)
    data = json5.load(f) # return the json file as a dict
    return data

def update_loc(target, drone_num, is_local, drone_pos=None):
    """ returns the desired GPSCoord position of the drone in formation, 
    given the target GPSCoord, and drone ID. If target is -1, indicating an
    error, returns the drones current location if it is supplied esle returns None"""

    params = get_parameters(is_local)
    offset = params["COMMON"]["DRONE_DISTANCE"]

    if target.lat == -1 or target.long == -1:
        pos = drone_pos
        rospy.logwarn("no lat or long")
    else:
        if drone_num == 0: # TX
            pos = target.add_x_offset(0).add_y_offset(0).add_z_offset(params["0"]["DEF_ALTITUDE"])

        elif drone_num == 1: # RX1
            pos = target.add_x_offset(-offset).add_y_offset(offset).add_z_offset(params["1"]["DEF_ALTITUDE"])

        elif drone_num == 2: # RX2
            pos = target.add_x_offset(offset).add_y_offset(offset).add_z_offset(params["2"]["DEF_ALTITUDE"])

        elif drone_num == 3: # RX3
            pos = target.add_x_offset(-offset).add_y_offset(-offset).add_z_offset(params["3"]["DEF_ALTITUDE"])

        elif drone_num == 4: # RX4
            pos = target.add_x_offset(offset).add_y_offset(-offset).add_z_offset(params["4"]["DEF_ALTITUDE"])

    return pos
