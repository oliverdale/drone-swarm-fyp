"""
filename: logger.py
author: Connor O'Reilly
date: 30th September 2021
description: 
    Reads the drone 0 log file and plots the drones position while it is in the tracking state
    To use this file update the flight time variable to the date you wish to plot. The log needs to be in the logs folder
    in the same directory as this file. Only works for logs created after 09-28-2021_13-58 as the format is different before
    this date.
"""

import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
import csv
from gps import *
from multilateration import PosFilter

# Change the current working directory to this files location
os.chdir(os.path.dirname(__file__)) 

flight_time = "10-25-2021_14-39" # Date of the log file to be used. This script does not work for flight tests before 09-28-2021_13-58
flight_file_name = "logs/flight_" + flight_time + "/" # look for logs in the log folder
log_file_drone0_name = flight_file_name + "drone0_" + flight_time #+ ".txt"

def str_to_float_list(str_in):
    """
    Takes a string representation of a list of three floats and returns a list of three floats 
    """
    # If the string is None return nones
    if str_in == 'None' or str_in is None:
        list_of_floats = [None, None, None]
    else:
        list_of_strs = str_in.replace('[', '').replace(']', '').split(' ')
        list_of_floats = []
        # Go through the split list and convert them to floats
        for s in list_of_strs:
            if s == '':
                pass
            elif s != 'None':
                list_of_floats.append(float(s))
            else:
                list_of_floats.append(None)
                
    return list_of_floats

def state_str_to_list(str_in):
    """
    Takes a string of states as they are stored in the logs (e.g. [5, 5, 5, 5]5) and 
    returns the states as a python list (e.g. [5, 5, 5, 5, 5])
    """
    if str_in is not None:
        tx_state = [int(str_in[-1])] # Remove the final character of the string as it is the tx's state
        x = str_in[:-1]
        states = [int(i.strip(',')) for i in x.strip('[]').strip(',').split(' ')]
        return tx_state+states
    else:
        return None

def remove_nones(a, b):
    """
    Takes two lists which may contain None values at random positions. This function
    removes The None and the value with the same index from the other list.
    The returned lists do not contain Nones and are the same length.
    Only works on lists of lists which contain Nones
    TODO: make work on lists which contain Nones
    """
    longest = len(a) if len(a) > len(b) else len(b)
    a_out, b_out = [], []
    for i in range(0,longest):
        if i < len(a):
            if any(ap is not None for ap in a[i]):
                if i < len(b): 
                    if any(bp is not None for bp in b[i]):
                        a_out.append(a[i])
                        b_out.append(b[i])

    return a_out, b_out


def GPS_str_to_obj(GPS_str):
    """
    Takes a string representation of a GPSCoord (e.g. GPS(-43.5206, 172.5830, 0.0000, var=[-1.0,-1.0,-1.0]))
    and returns the GPSCoord object represented by that string
    """
 
    if GPS_str == 'None' or GPS_str == None:
        return None
    
    str_list = GPS_str.replace('GPS(', '').replace(')', '').replace('var=', '').split(',')
    vars = '{} {} {}'.format(str_list[3],str_list[4],str_list[5]) # The last three items in the list are the variances
    return_val = GPSCoord(
        float(str_list[0]),
        float(str_list[1]),
        float(str_list[2]),
        variances=str_to_float_list(vars)
    )
    return return_val


def main():

    # Initialise the lists which hold the log data
    target_filt_cart = []
    tx_filt_cart = []
    states_list = []
    tx_filt_gps = []
    tx_real_gps = []
    target_real_gps = []
    multilat_error = []
    times = []

    # Open the log file to read the data
    with open(log_file_drone0_name) as csv_file:
        next(csv_file) # Skip the first row as it does not contain column names
        csv_reader = csv.DictReader(csv_file, delimiter=';')
        # Go through each row, extract the relevant data and convert it to a useful form
        i = 1
        for row in csv_reader:
            # Check if its the last row, if so break
            if 'STOP' in row['TIME']:
                break
            else:
                target_filt_cart.append(str_to_float_list(row['TARGET_FILT_CART']))
                tx_filt_cart.append(str_to_float_list(row['TX_FILT_CART']))
                states_list.append(state_str_to_list(row['STATES_LIST']))
                tx_filt_gps.append(GPS_str_to_obj(row['TX_FILT_GPS']))
                tx_real_gps.append(GPS_str_to_obj(row['TX_REAL_GPS']))
                target_real_gps.append(GPS_str_to_obj(row['TARGET_REAL_GPS']))
                multilat_error.append(row['MULTILATERATION_ERROR'])
                times.append(datetime.strptime(row['TIME'],'%H:%M:%S:%f'))

    # This is a list of bool values which are True when all the drones are in the tracking state
    should_plot = [s == [5,5,5,5,5] for s in states_list]
    # Apply the mask to all the data. The filter isinitialised until the drones are in the tracking state so the
    # logged filter values before this are not usefull
    target_pos_mask = [i for (i, v) in zip(target_filt_cart, should_plot) if v]
    tx_filt_cart_mask = [i for (i, v) in zip(tx_filt_cart, should_plot) if v]
    tx_GPS_filt_mask = [i for (i, v) in zip(tx_filt_gps, should_plot) if v]
    tx_GPS_real_mask = [i for (i, v) in zip(tx_real_gps, should_plot) if v]
    target_GPS_real_mask = [i for (i, v) in zip(target_real_gps, should_plot) if v]
    times_mask = [i for (i, v) in zip(times, should_plot) if v]
    multilat_error_mask = np.array([i for (i, v) in zip(multilat_error, should_plot) if v]).astype(float)


    # Initialise the kalman filter object to calculate cartesian coords from lat lng (no filtering is done in the script)
    pos_filter = PosFilter(
            drones_i = [GPSCoord(0,0,0)]*5, # Other than the initial target position none of these values matter
            z_std_drone = 0,
            z_std_range = 0,
            initial_target = target_GPS_real_mask[0], # Use the initial position of the target as the origin
            initial_target_uncertanty = 0.1,
            is_local=True)

    cartesian_tx_pos = []
    cartesian_tx_pos_filt = []
    cartesian_target_pos = []


    # Convert the GPS objects to np arrays with cartesian coords so they can be plotted
    for a_gps, b_gps, c_gps in zip(target_GPS_real_mask,tx_GPS_real_mask, tx_GPS_filt_mask):
        cartesian_target_pos.append(pos_filter.GPS_to_cartesian(a_gps))
        cartesian_tx_pos.append(pos_filter.GPS_to_cartesian(b_gps))
        cartesian_tx_pos_filt.append(pos_filter.GPS_to_cartesian(c_gps))

    cartesian_target_pos = np.array(cartesian_target_pos)
    cartesian_tx_pos = np.array(cartesian_tx_pos)
    cartesian_tx_pos_filt = np.array(cartesian_tx_pos)
    tx_filt_cart_mask = np.array(tx_filt_cart_mask)

    target_pos_mask, tx_filt_cart_mask = remove_nones(target_pos_mask,tx_filt_cart_mask)
    target_pos_mask = np.array(target_pos_mask)
    tx_filt_cart_mask = np.array(tx_filt_cart_mask)

    time_since_tracking = [] # A list of times expressed as floats (time since the start of tracking)
    start_time = times_mask[0] # Set the starting time to when tracking starts
    for time_obj in times_mask:
        t = (time_obj - start_time).total_seconds()
        time_since_tracking.append(t)


    # Plot the real position of the target and the real position of the tx drone
    pos_plot = plt.figure(1)
    ax = pos_plot.add_subplot(121, projection='3d')
    # The real position of the target
    ax.plot(cartesian_target_pos[:, 0], cartesian_target_pos[:, 1], cartesian_target_pos[:, 2], 'magenta', label = 'Real target position')
    ax.plot(target_pos_mask[:, 0], target_pos_mask[:, 1], target_pos_mask[:, 2], 'b', label = 'Est. target position')

    # Real position of the TX drone
    ax.plot(cartesian_tx_pos[:, 0], cartesian_tx_pos[:, 1], cartesian_tx_pos[:, 2], 'g', label = 'Real drone position') 
    # Commanded position output from the kalman filter
    # ax.plot(tx_filt_cart_mask[:, 0], tx_filt_cart_mask[:, 1], tx_filt_cart_mask[:, 2], 'k--', label = 'Commanded drone position', alpha = 0.5) 
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    plt.legend()

    ax = pos_plot.add_subplot(122)
    ax.plot(time_since_tracking, multilat_error_mask, label = 'Error')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [m]')
    plt.show()


# If this file is being run, run main
if __name__ == "__main__":
    main()

