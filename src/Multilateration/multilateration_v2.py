"""
filename: multilateration_v2.py
author: Oliver Dale
date: 10th July 2021
description:
    Multilateration program containing functions for testing the TSOA 
    (Time Sum of Arrival) method and allows integration into the 2020 
    system. 
"""

from scipy import optimize
from scipy.optimize.minpack import curve_fit
from gpsv2 import GPSCoord, GPSCoord3D
from swarm import Coord3D, Swarm3D, generatePathv2
import numpy as np
import matplotlib.pyplot as plt
from common import *
import math
from scipy.interpolate import make_interp_spline, BSpline
import seaborn as sns
import pandas as pd
import scipy.optimize as opt
import matplotlib.patches as patches



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
            pos = pos.add_z_offset(TXOFFSET)
        elif drone_num == 1:
            pos = target.add_x_offset(-5)
            pos = pos.add_y_offset(5)
            pos = pos.add_z_offset(0)
        elif drone_num == 2:
            pos = target.add_x_offset(5)
            pos = pos.add_y_offset(5)
            pos = pos.add_z_offset(0)
        elif drone_num == 3:
            pos = target.add_x_offset(-5)
            pos = pos.add_y_offset(-5)
            pos = pos.add_z_offset(0)
        elif drone_num == 4:
            pos = target.add_x_offset(5)
            pos = pos.add_y_offset(-5)
            pos = pos.add_z_offset(0)
    return pos

def generate_ranges(tx, rx, tar):
    """ Calculates a simulated radar range for each reciever drone. The range corresponds to the distance
        from the transmitter to the target, summed with the distance from the target to the receiver. Gaussian
        noise has been added to account for inherent inaccuracies of the GPS modules."""

    noise = generate_noise()

    target = GPSCoord3D(tar[0], tar[1], tar[2])

    range1 = tx.distance(target) + target.distance(rx[0]) + noise[0]
    range2 = tx.distance(target) + target.distance(rx[1]) + noise[1]
    range3 = tx.distance(target) + target.distance(rx[2]) + noise[2]
    range4 = tx.distance(target) + target.distance(rx[3]) + noise[3]

    return [range1, range2, range3, range4]

def generate_noise():
    """ Generate noise with a standard deviation of 2.5 m and zero mean. Consider each range reading to have independent 
        noise. """
    r1noise = np.random.normal(0,1,1)[0]
    r2noise = np.random.normal(0,1,1)[0]
    r3noise = np.random.normal(0,1,1)[0]
    r4noise = np.random.normal(0,1,1)[0]

    return [r1noise, r2noise, r3noise, r4noise]

def est_target_pos(tx, rx, ranges, prev_est):
    """ Estimates the location of the target in 3D using TSOA multilateration. 
        Function takes the 2D GPS location of each UAV with the associated range readings.
        Calculates the 3D location of target but returns the 2D location for integration. """

    if isinstance(tx, GPSCoord):
        # Convert 2D GPS coordinates to 3D GPS coordinates.
        tx = GPSCoord3D(tx.lat, tx.long, TX_ALTITUDE)
        rx[0] = GPSCoord3D(rx[0].lat, rx[0].long, RX_ALTITUDE)
        rx[1] = GPSCoord3D(rx[1].lat, rx[1].long, RX_ALTITUDE)
        rx[2] = GPSCoord3D(rx[2].lat, rx[2].long, RX_ALTITUDE)
        rx[3] = GPSCoord3D(rx[3].lat, rx[3].long, RX_ALTITUDE)
        prev_est = GPSCoord3D(prev_est.lat, prev_est.long, TARGET_ALTITUDE)

    # Convert GPS to cartersian coordinates with the transmitter as a reference. 
    txLoc = Coord3D([0, 0, 0])
    rx1Loc = Coord3D([tx.x_distance(rx[0]), tx.y_distance(rx[0]), tx.z_distance(rx[0])])
    rx2Loc = Coord3D([tx.x_distance(rx[1]), tx.y_distance(rx[1]), tx.z_distance(rx[1])])
    rx3Loc = Coord3D([tx.x_distance(rx[2]), tx.y_distance(rx[2]), tx.z_distance(rx[2])])
    rx4Loc = Coord3D([tx.x_distance(rx[3]), tx.y_distance(rx[3]), tx.z_distance(rx[3])])
    prevLoc = Coord3D([tx.x_distance(prev_est), tx.y_distance(prev_est), tx.z_distance(prev_est)])

    # Place UAV's in formation and determine target location. 
    current_swarm = Swarm3D(txLoc, rx1Loc, rx2Loc, rx3Loc, rx4Loc)
    estLoc = current_swarm.find_target(ranges[0], ranges[1], ranges[2], ranges[3], [prevLoc.x, prevLoc.y, prevLoc.z], 'TSOA')

    # Convert to GPS
    estLocGPS = tx.add_x_offset(estLoc.x).add_y_offset(estLoc.y).add_z_offset(estLoc.z)

    return estLocGPS

def plot_gps(path, initial):
    newpath = []

    for i in range(0, len(path)):
        coords = [initial.x_distance(path[i]), initial.y_distance(path[i]), initial.z_distance(path[i])]
        newpath.append(coords)
    return np.array(newpath)

def main (locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts):

    #Set initial conditions for localisation testing
    tx = GPSCoord3D(-43.52051, 172.58312, 20 + TXOFFSET)
    rx1 = GPSCoord3D(-43.52046, 172.58305, 20)
    rx2 = GPSCoord3D(-43.52056, 172.58305, 20)
    rx3 = GPSCoord3D(-43.52046, 172.58319, 20)
    rx4 = GPSCoord3D(-43.52056, 172.58319, 20)
    target = GPSCoord3D(-43.52051, 172.58312, 0)

    rx1Path = []
    rx2Path = []
    rx3Path = []
    rx4Path = []
    txPath = []
    estPath = []
    targetPath = []

    #Generate target path
    target_path = generatePathv2(target, locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts)

    prev_est = GPSCoord3D(-43.52051, 172.58312, 0)
    initial_tx = tx

    for i in range(0, len(target_path)):
        ranges = generate_ranges(tx, [rx1, rx2, rx3, rx4], target_path[i])
    
        est_pos = est_target_pos(tx, [rx1, rx2, rx3, rx4], [ranges[0], ranges[1], ranges[2], ranges[3]], prev_est)

        rx1 = update_loc(est_pos, 1, rx1)
        rx2 = update_loc(est_pos, 2, rx2)
        rx3 = update_loc(est_pos, 3, rx3)
        rx4 = update_loc(est_pos, 4, rx4)
        tx = update_loc(est_pos, 0, tx)

        rx1Path.append(rx1)
        rx2Path.append(rx2)
        rx3Path.append(rx3)
        rx4Path.append(rx4)
        txPath.append(tx)
        estPath.append(est_pos)

        prev_est = est_pos

    estPath = plot_gps(estPath, initial_tx)
    rx1Path = plot_gps(rx1Path, initial_tx)
    rx2Path = plot_gps(rx2Path, initial_tx)
    rx3Path = plot_gps(rx3Path, initial_tx)
    rx4Path = plot_gps(rx4Path, initial_tx)
    txPath = plot_gps(txPath, initial_tx)

    new = []
    i = 0
    for i in range(0, len(target_path)):
        gpspos = GPSCoord3D(target_path[i][0], target_path[i][1], target_path[i][2])
        new.append(gpspos)

    targetPath = plot_gps(new, initial_tx)

    pos_plot = plt.figure(3)
    ax = plt.axes(projection='3d')

    error = sum(abs(estPath - targetPath))/len(targetPath)

    total_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
    print("TX OFFSET: {}    TOTAL ERROR: {}     X: {}   Y: {}   Z: {}".format(TXOFFSET, total_error, error[0], error[1], error[2]))

    ax.plot3D(estPath[:, 0], estPath[:, 1], estPath[:, 2], 'red', label = 'target') #estimated
    ax.plot3D(rx1Path[:, 0], rx1Path[:, 1], rx1Path[:, 2], '--c', alpha=0.3, label = 'rx1') #rx1
    ax.plot3D(rx2Path[:, 0], rx2Path[:, 1], rx2Path[:, 2], '--c', alpha=0.3, label = 'rx2') #rx2
    ax.plot3D(rx3Path[:, 0], rx3Path[:, 1], rx3Path[:, 2], '--c', alpha=0.3, label = 'rx3') #rx3
    ax.plot3D(rx4Path[:, 0], rx4Path[:, 1], rx4Path[:, 2], '--c', alpha=0.3, label = 'rx4') #rx4
    ax.plot3D(txPath[:, 0], txPath[:, 1], txPath[:, 2], '--b', alpha=0.3, label = 'tx') #tx
    ax.plot3D(targetPath[:, 0], targetPath[:, 1], targetPath[:,2], 'green', label = 'real') #real
    plt.legend()
    #plt.show()

    return total_error

def test_target_pos():
    #Set initial conditions for localisation testing
    tx = GPSCoord3D(-43.52051, 172.58312, 30)
    rx1 = GPSCoord3D(-43.52046, 172.58305, 20)
    rx2 = GPSCoord3D(-43.52056, 172.58305, 20)
    rx3 = GPSCoord3D(-43.52046, 172.58319, 20)
    rx4 = GPSCoord3D(-43.52056, 172.58319, 20)
    prev_est = GPSCoord3D(-43.52051, 172.58312, 0)

    x_offsets = np.arange(-25, 25, 1)
    y_offsets = np.arange(-25, 25, 1)

    z_error = np.zeros((50, 50))

    x_index = 0
    y_index = 0

    for x_offset in x_offsets:
        y_index = 0
        for y_offset in y_offsets:
            targetGPS = tx.add_x_offset(x_offset).add_y_offset(y_offset).add_z_offset(-30)
            target = [targetGPS.lat, targetGPS.long, targetGPS.alt]
            error = 0

            for i in range(0, 100):
                ranges = generate_ranges(tx, [rx1, rx2, rx3, rx4], target)
                est_pos = est_target_pos(tx, [rx1, rx2, rx3, rx4], [ranges[0], ranges[1], ranges[2], ranges[3]], prev_est)

                error += (est_pos.distance(targetGPS)/100)

            z_error[y_index, x_index] = error
            print("X Off: {}    Y Off: {}   Error: {}".format(x_offset, y_offset, error))
            y_index += 1
        x_index += 1

    drones_x = [-5, 5, -5, 5, 0]
    drones_y = [5, -5, -5, 5, 0]

    fig1, ax2 = plt.subplots(constrained_layout=True)
    ax2.set_ylabel('Y Location [m]')
    ax2.set_xlabel('X Location [m]')
    CS = ax2.contourf(x_offsets, y_offsets, z_error)
    ax2.scatter(drones_x, drones_y, color='black', marker='o', s=75, label='Drones')
    ax2.legend()
    cbar = fig1.colorbar(CS)
    cbar.ax.set_ylabel('Estimate Error [m]')  
    plt.show()

if __name__ == "__main__":
    # Generate path variables 
    # locations = 80
    # maxVelocity = 3
    # maxAcceleration = 1
    # maxAcceleration_dot = 1
    # ts = 1

    # noffsets = [-30, -20, -15, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0]
    # poffsets = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 ,15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]

    # error_list = []
    
    # for TXOFFSET in poffsets:
    #     error = main(locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts)
    #     error_list.append(error)

    # error_plot = plt.figure(1)
    # ax = plt.axes()
    
    # a=[0,0]
    # b=[0,10]
    # c=[5,0]
    # d=[5,10]
    # width = c[0] - a[0]
    # height = d[1] - a[1]

    # ax.scatter(poffsets, error_list, label = '', color='black')
    # ax.add_patch(patches.Rectangle((0, 0), width, height, alpha=0.4, color='red', label = 'Unstable'))
    # ax.set(ylim=(0, 10), xlim=(0, 30))
    # ax.legend()
    # ax.set_xlabel('Transmitter Altitude Offset [m]')
    # ax.set_ylabel('Target Estimate Error [m]')
    # plt.show()

    test_target_pos()






    




    


