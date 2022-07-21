"""
filename: update_store.py
author: Reka Norman
date: 16th October 2020
description:
    The Tx uses an UpdateStore to store all the updates received from the Rx
    UAVs, group them together based on their timestamp, and access values
    from these updates as needed for multilateration, swarming, etc.
"""

import time
from tkinter.constants import NO
from common import *
import logger
import rospy

# The maximum allowed time difference in seconds between the timestamps of
# range readings belonging to the same group.
SYNC_ACCURACY_S = 0.5

class TimingException(Exception):
    """  """
    pass


class UpdateStore:
    """ Stores all the updates received from the Rx's, grouping them by the
    sequence number of the reading they correspond to. Allows data from the most
    recent group of readings to be accessed for multilateration, swarming, etc.
    """
    def __init__(self):
        # A list of lists, where updates[i][j] is the update with sequence
        # number i from the Rx with ID j + 1. The list is expanded every time an
        # update with a new sequence number is received.
        self.updates = {}

        # The reading number of the most recent full group of updates.
        self.current_seq_key = str(-1)

        # The last time an update was received from each Rx, useful for
        # determining the cause of a timeout.
        self.last_update_times = [time.time()] * NUM_RXS

        items=[
            'rx_id',
            'state',
            'timestamp',
            'reading_number',
            'rx_coords',
            'range_reading',
            'target_coords'
        ]
        self.logger = logger.Logger(items=items, filename='sync_log')


    def store(self, update):
        """ Store a new RxUpdate. Returns True if a new full group of readings
        is ready after storing this update, False otherwise.
        """
        if str(update.reading_number) in self.updates:
            self.updates[str(update.reading_number)][update.rx_id - 1] = update
        else:
            new_update = ([None] * NUM_RXS)
            new_update[update.rx_id - 1] = update
            self.updates[str(update.reading_number)] = new_update

        # Log the update items for debugging
        self.logger.log_items([
            str(update.rx_id),
            str(update.state),
            str(update.timestamp),
            str(update.reading_number),
            str(update.rx_coords),
            str(update.range),
            str(update.target_coords)
        ])

        # Find the most recent full update (A full update is when all the drones have transmitted an update and the tx has recieved them)
        for key, curr_update in self.updates.items():
            if all(i != None for i in curr_update):
                if int(key) > int(self.current_seq_key):
                    self.current_seq_key = key
        
        # Remove updates whos update number is less than the most recent full update
        old_keys = []
        for key in self.updates.keys():
            if int(key) < int(self.current_seq_key):
                old_keys.append(key)
                
        for key in old_keys:
            self.updates.pop(key)

        # Check if we have a new full group of updates.
        if self.current_seq_key == str(update.reading_number):
            # TODO: this can be uncommented either if the UAV processes are all
            # run on the same machine, or once time synchronisation is
            # implemented so that the UAVs have a shared time source.
            # self.check_timestamps()

            return True
        else:
            return False
    
    def reset(self):
        """
        This function resets the updates as if the latest update is the first. It is run when o
        """
        del self.updates
        self.updates = {}
        self.current_seq_key = str(-1)

    def current_group(self):
        """ Return the list corresponding to the most recent full group of updates.
        """
        return self.updates[self.current_seq_key]

    def check_timestamps(self):
        """ Check that all the readings in the current group of updates were
        taken within a time interval less than SYNC_ACCURACY_S.
        """
        times = [update.timestamp for update in self.current_group()]
        time_range = max(times) - min(times)
        rospy.loginfo("dt:" + str(time_range))
        if time_range >= SYNC_ACCURACY_S:
            raise TimingException(
                "ERROR: timestamps of readings in group #{} differed by {:.4f} s."
                .format(self.current_seq_key, time_range))

    def get_rx_positions(self):
        """ Return a list of the current positions of each Rx, ordered by Rx ID.
        The positions are taken from the most recent full group of Rx updates.
        """
        return [update.rx_coords for update in self.current_group()]

    def get_ranges(self):
        """ Return a list of the most recent range readings, ordered by Rx ID.
        The ranges are taken from the most recent full group of Rx updates.
        """
        return [update.range for update in self.current_group()]

    def get_states(self):
        """ Return a list of the most recent state readings, ordered by Rx ID.
        """
        return [update.state for update in self.current_group()]

    def get_actual_target_coords(self):
        """ Return the actual target coordinates corresponding to the most
        recent group of updates (sent by the Rxs for use in plotting).
        """
        # Assume that all updates in the group have the same actual target
        # position, so just return the first one. Maybe this should be checked.
        return self.current_group()[0].target_coords

    def get_last_update_times(self):
        return self.last_update_times
