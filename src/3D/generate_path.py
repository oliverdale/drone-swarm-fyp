"""
filename: generate_path.py
author: Rowan Sinclair
date: 30th September 2021
description:
    Generates a random path which simulates an insect, and saves it to a file which can be read by server_data.py
"""


import os
from datetime import datetime, timedelta
from gps import GPSCoord
from swarm import generatePathv2
import numpy as np


os.chdir(os.path.dirname(__file__)) # set the working directory to this file's directory
filename = 'test.txt'
   
def main():

    d = datetime.today()
    date_str = d.strftime("%a %b  %d %X %Y")

    #Generate path variables 
    locations = 100 # Number of target locations to filter
    maxVelocity = 5
    maxAcceleration = 5
    maxAcceleration_dot = 5
    ts = 0.5 # Timestep between locations

    #Set initial conditions for localisation testing
    target_i = GPSCoord(-43.5206, 172.583, 0)

    #Generate target path
    target_path = generatePathv2(target_i, locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts)

    f = open(filename, 'w')

    # Loop through each position in target path
    for i, p in enumerate(target_path):
        # Get the current time plus i for the file
        time_change = timedelta(seconds=i)
        date_str = (d+time_change).strftime("%a %b  %d %X %Y")
        # Write to the file
        write_str = '{},{},{},{}'.format(date_str, p[0], p[1], p[2])
        f.write(write_str + '\n')

    f.close()

# If this file is being run, run main
if __name__ == "__main__":
    main()


