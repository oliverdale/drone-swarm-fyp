"""
filename: swarming_logic.py
author: Callum Fraser
date: 16th October 2020
description: Helper functions for Rx.py, Tx.py, to implement drone swarming 
(formation flight).

The basic operation is that the Tx checks the GPS position of all the drones. 
If the formation is adequate, the target position is sent to all UAV's, which 
then find their own position in formation. 
If the formation is poor, the average centre position of the drones are output,
until formation is restored, and we once again track the target.

The drone ID numbers correspond to their positions, where 0 is Tx, 1-4 Rx's.
 1          2

      0

 3          4
"""

import time
import math
from gpsv2 import GPSCoord


def update_loc(target, drone_num, drone_pos):
    """ returns the desired GPSCoord position of the drone in formation, 
    given the target GPSCoord, and drone ID. If target is -1, indicating an
    error, returns the drones current location """

    if target.lat == -1 or target.long == -1:
        pos = drone_pos
    else:
        if drone_num == 0:
            pos = target.add_x_offset(0)
            pos = pos.add_y_offset(0)
        elif drone_num == 1:
            pos = target.add_x_offset(-5)
            pos = pos.add_y_offset(5)
        elif drone_num == 2:
            pos = target.add_x_offset(5)
            pos = pos.add_y_offset(5)
        elif drone_num == 3:
            pos = target.add_x_offset(-5)
            pos = pos.add_y_offset(-5)
        elif drone_num == 4:
            pos = target.add_x_offset(5)
            pos = pos.add_y_offset(-5)
    return pos


class Swarming:
    """ Swarming class for the Tx drone. This is used to update the swarming
    state.
    Inputs: Drone positions, target position
    Output: The next destination for the formation
    Based upon the drone positions, a finite state machine is updated, and a
    destination is outputted
    """
    def __init__(self, drone_positions):

        # Set state to 0 (normal)
        self.swarming_state = 0
        
        self.drone_positions = drone_positions
        
        # Find the centre of the drone formation
        self.centres = self.centres_from_drones()
        self.est_centre, self.error = self.mean_centre()
        
        # Initialise all targets and destination as the formation centre
        self.target_coords = self.est_centre
        self.prev_target = self.est_centre
        self.dest_coord = self.est_centre
    
    
    def update_drones(self, drone_positions):
        """ Update the swarming variables given new
        drone positions and target coordinates """
        
        self.drone_positions = drone_positions
        self.centres = self.centres_from_drones()
        
        self.est_centre, self.error = self.mean_centre()
    
    
    def update_target(self, target_coords):
        """ Update the swarming variables given new
        drone positions and target coordinates """

        self.prev_target = self.target_coords
        self.target_coords = target_coords      
    
    
    def swarming_checks(self):
        """ Returns the desired centre position of the formation. 
        If formation is fine, output the target position, 
        otherwise outputs the averaged centre of the formation. 
        Discards unreasonable target GPS values. """

        # Update swarming formation fsm state
        self.update_fsm()
    
    
    def update_swarming_destination(self):
        # Check if the target position is valid. 
        # and keep the drones still if there is minimal change in target
        if not self.check_gps() or (self.target_coords.distance(
                self.prev_target) < 1.5):
            # Replace target with previous_target if target coord is bad
            self.target_coords = self.prev_target   
        
        # Update drones next destination from the fsm state
        self.update_destination()        
    
    
    def check_gps(self, max_diff = 10):
        """ Returns True if gps position is within max_diff = 10m of prev value,  
        and within NZ boundaries of -35 to -45 Lat, 167 to 178 Long.
        Else returns False.
         """
        gps_check = True
        if abs(self.target_coords.distance(self.prev_target) > max_diff):
            print("ERROR: GPS CHANGE = {} m".format(self.target_coords.distance(self.prev_target)))
            gps_check = False
        if not (-45 < self.target_coords.lat < -35 and 167 < self.target_coords.long < 178):
            print("ERROR: GPS OUTSIDE NZ")
            gps_check = False
        return gps_check
    
    
    def centres_from_drones(self):
        """ returns a list of the expected GPSCoords centre of the formation, 
        from the list of drone GPSCoords """
        centres = []
        drones = self.drone_positions
        if 0 < len(drones):
            centres.append(drones[0])
        if 1 < len(drones):
            centres.append(drones[1].add_x_offset(5))
            centres[1] = centres[1].add_y_offset(-5)
        if 2 < len(drones):
            centres.append(drones[2].add_x_offset(-5))
            centres[2] = centres[2].add_y_offset(-5)
        if 3 < len(drones):
            centres.append(drones[3].add_x_offset(5))
            centres[3] = centres[3].add_y_offset(5)
        if 4 < len(drones):
            centres.append(drones[4].add_x_offset(-5))
            centres[4] = centres[4].add_y_offset(5)
        return centres
    
    
    def mean_centre(self):
        """ Returns the mean average GPSCoord of the centres list, and returns the 
        error; the grestest distance in m from the mean to a value in centres """
        centres = self.centres
        lat_sum = 0
        long_sum = 0
        error = 0
        n = 0
        for centre in centres:
            lat_sum += centre.lat
            long_sum += centre.long
            n += 1
        if lat_sum != 0 and long_sum != 0:
            mean_centre = GPSCoord(lat_sum / n, long_sum / n)
        
        # Find error in mean centre
        for centre in centres:
            if centre.distance(mean_centre) > error:
                error = centre.distance(mean_centre)
        return mean_centre, error
    
    
    def check_formation(self, max_error = 3):
        """ Checks positions of drones. Returns True if formation is adequate, 
        False otherwise. Works for up to 5 drones. """
        formation = True
        # Check if est_centre is valid
        if self.error > max_error:
            formation = False
        print("Max error in centre estimate of {:.2f} m".format(self.error))
        return formation
    
    
    def critical_formation(self):
        """ Checks positions of drones relative to each other, to detect if
        drones are in positions such that formation cannot be restored """
        formation = True
        drones = self.drone_positions
        error = self.error
        
        # All drones must have at least 3 m between each other   
        for i in range(len(drones) - 1):
            drone1 = drones[i]
            for n in range(len(drones) - i - 1):
                drone2 = drones[n + i + 1]
                if drone1.distance(drone2) < 1:
                    print("ERROR: DRONES WITHIN 3M OF EACH OTHER")
                    formation = False
        
        # Check that all drones are the right direction relative to each other
        # Drone 0 is known as checked by all others
        # Drone 1 left (lower long) of 0, 2, 4, and above (higher lat) 0, 3, 4
        if not(drones[1].long < drones[0].long and drones[1].long < drones[2].long and drones[1].long < drones[4].long):
            formation = False
        if not(drones[1].lat > drones[0].lat and drones[1].lat > drones[3].lat and drones[1].lat > drones[4].lat):
            formation = False
        # Drone 2 right of (higher long) 0, 3, and above (higher lat) 0, 3, 4
        if not(drones[2].long > drones[0].long and drones[2].long > drones[3].long):
            formation = False
        if not(drones[2].lat > drones[0].lat and drones[2].lat > drones[3].lat and drones[2].lat > drones[4].lat):
            formation = False    
        # Drone 3 left of (lower long) 0, 4, and below (lower lat) 0
        if not(drones[3].long < drones[0].long and drones[3].long < drones[4].long):
            formation = False
        if not(drones[3].lat < drones[0].lat):
            formation = False
        # Drone 4 right of (higher long) 0 and below (lower lat) 0
        if not(drones[4].long > drones[0].long):
            formation = False
        if not(drones[4].lat < drones[0].lat):
            formation = False
            
        # Drones drift far from out of formation
        if error > 6:
            formation = False
            print("ERROR: DRONES TOO FAR OUT OF FORMATION")
            print("Max error in centre estimate of {:.2f} m".format(self.error))
        
        return formation
    
    
    def update_fsm(self):
        """ Update the swarming fsm for the drone formation given drone positions
        and current state. Sates are 0 (normal), 1 (reset formation), 2 (stop) """        
        
        swarming_state = self.swarming_state
        
        # Check the drone formation and update the fsm state accordingly
        if not self.critical_formation() or swarming_state == 2:
            # Set state to 2 (stop) if there is a critical (unrecoverable) formation
            swarming_state = 2
            print("ERROR: CRITICAL - STOP THE DRONES")
        elif not self.check_formation():
           # Set state to 1 (reset) if drones are out of formation (recoverable)
            swarming_state = 1
            print("RESET FORMATION")
        elif swarming_state == 1 and self.error >= 1.5:
            # Do not change from reset (1) to normal (0) until all drones are 
            # within 0.5m of their desired position
            print("RESET FORMATION")
        else:
            # if no formation errors, set state to 0 (normal)
            swarming_state = 0
        
        self.swarming_state = swarming_state
    
    def update_destination(self):
        """ Ouput destination depending fsm state """
        if self.swarming_state == 0:
            output = self.target_coords
        elif self.swarming_state == 1:
            # desination average centre of the drones to reset formation 
            output = self.est_centre            
        elif self.swarming_state == 2:
            # If formation is critical stop the drones
            # TODO: Set drone mode to hold (pixhawk)            
            output = GPSCoord(-1, -1)            
        self.dest_coord = output    
