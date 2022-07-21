"""
filename: swarming_logic.py
author: Alex Scott
date: 16th September 2021
description: Functions to check positions and variance of tx and rx positions.
It then returns if there is an error or warning for the drone formation's
safety.
"""

import math
import numpy as np
from gps import GPSCoord
from common import *
import rospy


class Failsafes:

    def __init__(self, is_local) -> None:
        # Get the constants from the json
        self.fp = get_parameters(is_local)
        self.size = self.fp['COMMON']['DRONE_DISTANCE']*2 
        self.formation_error = self.fp['COMMON']['FORMATION_ERROR']
        self.var_def = self.fp['COMMON']['DEFAULT_VARIANCE']
        self.max_drone_movement = self.fp['COMMON']['MAX_DRONE_MOVEMENT']

        # TODO: At the moment the altitude of rx1 is assumed to be the altitude of all the rx drones
        # Redo base distances matrix so that the rx drones can all have differnt altitudes
        self.rx_alt = abs(self.fp['1']["DEF_ALTITUDE"]-self.fp['0']["DEF_ALTITUDE"]) # The difference between the rx and tx alt
        self.tx_rx = np.sqrt((self.size*np.sqrt(2)/2)**2+self.rx_alt**2)
        self.rx_rx_diag = self.size*np.sqrt(2)

        self.prev_pos = None # The last position of the drone

    def positions(self, tx_pos, rx_pos):
        """
        This function takes the realtime GPS positions of the drones and returns True if the formation 
        is good. It checks whether the variance spheres overlap and give a warning if the drones are
        getting too far from their ideal swarm positions
        """
        drones = [tx_pos] + rx_pos

        # The distances between the drones
        base_distances = np.array([
            [0,           self.tx_rx,       self.tx_rx,       self.tx_rx,       self.tx_rx       ],
            [self.tx_rx,  0,                self.size,        self.size,        self.rx_rx_diag  ],
            [self.tx_rx,  self.size,        0,                self.rx_rx_diag,  self.size        ],
            [self.tx_rx,  self.size,        self.rx_rx_diag,  0,                self.size        ],
            [self.tx_rx,  self.rx_rx_diag,  self.size,        self.size,        0               ]])

        return_val = True

        for i, d1 in enumerate(drones):
            for j, d2 in enumerate(drones):
                # We don't need to compare the distance between each drone twice or the distance between the same drone
                if j < i:
                    # Sum the maximum variances to find the 1sd distance sphere
                    # TODO: Model the actuall ellipse for differing lat, long variances instead of taking max.
                    

                    # If any of the GPS variances are undefined, use the default variance
                    if any([var==-1 for var in d1.var+d2.var]):
                        limit = self.var_def*2
                    else:
                        limit = max(d1.var[0:2]) + max(d2.var[0:2])
        
                    # If the drone's variance spheres overlap return false
                    if d1.distance3d(d2) < limit:
                        rospy.logfatal("ERROR: DRONES {} and {} ARE TOO CLOSE ({:.2f})".format(i,j,d1.distance3d(d2)))
                        return_val = False

                    # If the drones are within a specified range limit of eachother return false
                    if abs(d1.distance3d(d2)-base_distances[i,j])/base_distances[i,j] > self.formation_error:
                        rospy.logwarn("WARNING: DRONES {} and {} ARE OUT OF FORMATION ({:.2f}m, {:.2f}m)".format(i,j, d1.distance3d(d2),base_distances[i,j]))
                        print(base_distances[i,j])
                        return_val = True

        return return_val
            


    def overlaps(self, tx_pos, rx_pos):
        """
        This function takes the tx and rx drone positions and checks whether any of their latitudes or longitudes overlap
        If they do the function returns False and a warning is printed
        """
        drones = [tx_pos] + rx_pos
        # Drone 1 left (lower long) of 0, 2, 4, and above (higher lat) 0, 3, 4
        # Indicates which drone has overlapped                    
        if not(drones[1].long < drones[0].long and drones[1].long < drones[2].long and drones[1].long < drones[4].long):
                rospy.logfatal("Drone 1 longitude overlap")
                return False
        if not(drones[1].lat > drones[0].lat and drones[1].lat > drones[3].lat and drones[1].lat > drones[4].lat):
                rospy.logfatal("Drone 1 latitude overlap")
                return False
        # Drone 2 right of (higher long) 0, 3, and above (higher lat) 0, 3, 4
        if not(drones[2].long > drones[0].long and drones[2].long > drones[3].long):
                rospy.logfatal("Drone 2 longitude overlap")
                return False
        if not(drones[2].lat > drones[0].lat and drones[2].lat > drones[3].lat and drones[2].lat > drones[4].lat):
                rospy.logfatal("Drone 2 latitude overlap")  
                return False
        # Drone 3 left of (lower long) 0, 4, and below (lower lat) 0
        if not(drones[3].long < drones[0].long and drones[3].long < drones[4].long):
                rospy.logfatal("Drone 3 longitude overlap")
                return False
        if not(drones[3].lat < drones[0].lat):
                rospy.logfatal("Drone 3 latitude overlap")
                return False
        # Drone 4 right of (higher long) 0 and below (lower lat) 0
        if not(drones[4].long > drones[0].long):
                rospy.logfatal("Drone 4 longitude overlap")
                return False
        if not(drones[4].lat < drones[0].lat):
                rospy.logfatal("Drone 4 latitude overlap") 
                return False
            
        return True



    def move(self, pos):
        """
        This function takes too GPScoords and checks whether the distance between them is too
        large. This is to ensure the drones aren't commanded to fly away. Returns True if the 
        drones are moving at a good speed else returns False and prints an error.
        """
        return_val = True
        # If the previous position is None the failsafes object has just been initialised, return true
        if self.prev_pos is not None:
            if pos.distance(self.prev_pos) > self.max_drone_movement:
                rospy.logfatal('ERROR: Drones moving too fast ({:.2f})'.format(pos.distance(self.prev_pos)))
                return_val = False
            elif pos.distance(self.prev_pos) > self.max_drone_movement*0.8:
                rospy.logwarn('WARNING: Drones are almost moving too fast ({:.2f})'.format(pos.distance(self.prev_pos)))
                return_val = True
            else:
                return_val = True

        self.prev_pos = pos
        return return_val

def main():
    """
    Short program to test failsafes
    """
    failsafes = Failsafes(is_local=True)

    # Define drone locations
    target = GPSCoord(-43.5206,172.58302,0,[1,2,-1])
    rxs = [None]*4
    tx = update_loc(target, 0, True)
    tx = tx.add_x_offset(0)
    for i in range(1,5):
        rxs[i-1] = update_loc(target, i, True)
    # Add small error in one drones position
    rxs[0] = rxs[0].add_y_offset(3)
    # Test the function
    print('Positions output: {}'.format(failsafes.positions(tx, rxs)))
    
    tx_next = tx.add_y_offset(8)
    print('Move output: {}'.format(failsafes.move(tx)))
    print('Move2 output: {}'.format(failsafes.move(tx_next)))

if __name__ == "__main__":
    main()
