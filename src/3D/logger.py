"""
filename: logger.py
author: Connor O'Reilly
date: 18-07-2021
description: Logs input data to a csv file with appended timestamps
"""

import os
from rospy import loginfo
from datetime import datetime
from common import *

LOG_DIR = '//logs//'

class Logger:

    def __init__(self, drone_num=0, items=[], filename=None):
        """
        Initialise the log folder and its files in the logs folder 
        Append the supplied items as the headings on the second row
        If filename is none use the date
        """
        self.items = items
        self.drone_num = drone_num

        time_str = datetime.now().strftime('%m/%d/%Y %H:%M:%S:%f')
        file_time_str = datetime.now().strftime('%m-%d-%Y_%H-%M')
        folder_name = "flight_" + file_time_str + '//'
        # Create a new log file and check if the directory exists
        path = os.path.dirname(__file__)
        if not os.path.exists(path + LOG_DIR + folder_name):
            try:
                os.makedirs(path + LOG_DIR + folder_name)
            except:
                rospy.logwarn("Could not make log folder")

        # Create the filename if it is not supplied
        if filename == None:
            filename = "drone{}_{}".format(drone_num, file_time_str)
        
        self.filename = filename
        self.f = open(path + LOG_DIR + folder_name + filename, 'w+')
        self.f.write('START: ' + time_str + '\n')
        self.f.write('TIME;') # Write the start time to the file
        for item in items:
            self.f.write(str(item).upper() + ';') # Write the headings to the file
        self.f.write('\n')

    def log_items(self, items):
        """
        Write the supplied items to the log file and append the date
        """
        time_str = datetime.now().strftime('%H:%M:%S:%f')
        self.f.write(time_str)
        for i, item in enumerate(items):
            try:
                self.f.write(';' + str(item))
            except:
                self.f.write(', FAIL:' + str(i))
                rospy.logwarn('could not write an item')
        self.f.write('\n')

    def __del__(self):
        """
        Close the file if the logger is deleted
        """
        time_str = datetime.now().strftime('%m/%d/%Y %H:%M:%S:%f')
        self.f.write('STOP: ' + time_str + '\n')
        self.f.close()
        save_str = "{} saved successfully".format(self.filename)
        rospy.loginfo(save_str)


if __name__ == "__main__":
    """
    Quick logger test
    """
    logger = Logger(1, ['hat', 'mat', 'car'])
    logger.log_items([True, 1, 1.052])
    del logger
