"""
filename: multilateration.py
author: Rowan Sinclair
date: 30th September 2021
description:
    Contains the Kalmn filter and multilateration equations to estimate the targets position using
    the drones poition and radar ranges. Also contains functions for converting from cartesian to GPS and back
    This file can be run to test the Kalman filter with an auto-generated target path
"""

from datetime import datetime
from numpy.core.records import array
from gps import GPSCoord
from swarm import generatePathv2
import numpy as np
from common import get_parameters, update_loc
from scipy.linalg import block_diag
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise, Saver
import rospy


def to_GPS(arr: np.array) -> GPSCoord:
    """
    Takes an array of GPS coodinates and converts them to a GPSCoord object
    """
    return GPSCoord(arr[0], arr[1], arr[2])


def to_arr(gps: GPSCoord) -> np.array:
    """
    Takes a GPSCoord object and convert it to a 3x1 array of [lat, long, alt]
    """
    return np.array([gps.lat, gps.long, gps.alt])


def simulate_ranges(target_loc, drones, noise=0, dropout=[False]*5):
    """
    This function simulates the ranges that each drone would measure given estimated drone positions and the real target position.
    It also adds noise and dropout to the readings by randomly setting some ranges to -1. In the real setup this function will be 
    replaced by the real radar readings with unrecieved readings also at -1. Dropout is a vector of Bools if dropout[1] = True
    range[0] = -1
    """
    # Calculate ranges plus noise
    # np.random.binomial(1,dropout_prob)
    tx_to_target = np.linalg.norm(drones[0]-target_loc)
    rs = []
    for i in range(1,5):
        if dropout[i]:
            rs.append(-1)
        else:
            rs.append(tx_to_target + np.linalg.norm(target_loc-drones[i]) + np.random.normal(0,noise,1)[0])

    return np.array(rs)


def fx(x, dt):
    """
    This is the state transition function. It predicts where the drones and target will be based on a constant
    velocity model: x = vt + x_0. Indexes 0 to 14 are the (x,y,z) positions of the drones. Indexes 15 to 17 are the
    (x,y,z) position of the target. Indexes 18 to 32 are the (x,y,z) veocities of the drones. Indexes 33 to 35 are
    the (x,y,z) velocity of the target.
    """
    x_out = np.zeros_like(x)
    for i in range(0,18):
        x_out[i] = x[i+18]*dt + x[i] # Item Position
        x_out[i+18] = x[i+18] # Item velocity

    return x_out

def hx(x):
    """
    This is the measurement function which converts the model state into values which can be compared to measurment values.
    The converted values are the ranges and the position of the drones. The ranges are calculated using the TSOA equations.
    Indexes 0 to 3 are the estimated range readings. Indexes 4 to 18 are the (x,y,z) positions of the drones.
    """
    x_out = np.zeros(19)
    a,b,c = 15,16,17
    tx_to_target = np.sqrt((x[0] - x[a])**2 + (x[1] - x[b])**2 + (x[2] - x[c])**2)
    x_out[0] = tx_to_target + np.sqrt((x[3] - x[a])**2 + (x[4] - x[b])**2 + (x[5] - x[c])**2)
    x_out[1] = tx_to_target + np.sqrt((x[6] - x[a])**2 + (x[7] - x[b])**2 + (x[8] - x[c])**2)
    x_out[2] = tx_to_target + np.sqrt((x[9] - x[a])**2 + (x[10] - x[b])**2 + (x[11] - x[c])**2)
    x_out[3] = tx_to_target + np.sqrt((x[12] - x[a])**2 + (x[13] - x[b])**2 + (x[14] - x[c])**2)
    x_out[4:19] = np.array(x[0:15])

    return x_out

def get_item_coords(xs, item_num):
    """
    This function extracts the position of an object from the state estimate vector or matrix. As inputs it takes the state estimate vector xs
    and a number 0 through 10. The number alters what the function returns as follows: tx_pos=0, rx1_pos=1, rx2_pos=2, rx3_pos=3, rx4_pos=4, 
    est_target_pos=5, tx_vel=6, rx1_vel=7, rx2_vel=8, rx3_vel=9, rx4_vel=10, est_target_vel=11. If a matrix is input a nx3 matrix of (x,y,z) values over 
    time is returned. If a vector is input a 1x3 vector is returned.
    """
    if (xs.ndim != 1): # For vector case
        drone_poss = np.zeros((np.shape(xs)[0],3))
        for i in range(0,3):
            pos = 3*item_num+i
            drone_poss[:,i] = xs[:,pos]
    else: # For matrix case
        drone_poss = np.zeros(3)
        for i in range(0,3):
            pos = 3*item_num+i
            drone_poss[i] = xs[pos]
    return drone_poss

class PosFilter:

    def GPS_to_cartesian(self, GPS):
        """
        Takes a GPS object and returns the cartesian x, y, z distance to the origin (specified when initialising PosFilter).
        Returns a 3x1 np.array
        """
        return np.array([self.initial_target.x_distance(GPS), self.initial_target.y_distance(GPS), self.initial_target.z_distance(GPS)])

    def cartesian_to_GPS(self, cart):
        """
        Takes a 3x1 array and returns a GPS object representing the corosponding GPS coordinate.
        """
        coord = self.initial_target.add_x_offset(cart[0]).add_y_offset(cart[1]).add_z_offset(cart[2])
        return coord

    def __init__(self, drones_i, z_std_drone, z_std_range, initial_target, is_local, dt=None, initial_target_uncertanty=0.1):
        """
        drones_i - The initial positions of the drones as a list of GPSCoord objects. In order tx, rx1, rx2, rx3, rx4.
        z_std_drone - The standard deviation in the GPS drone readings, as a float as meters
        z_std_range - The standard deviation in the radar range readings, as a float as meters
        initial_target - The initial GPS position of the target. This is the origin for the cartesian coordinate space
        dt - The default time step between sensor measurments. This can also be specified at each est_target_pos step.
        initial_target_uncertanty - The standard deviation of the initial target's position
        """
        
        # If no timestep is specified use the system time
        if dt == None:
            self.t_last = datetime.now()
            self.dt = None
        else: 
            self.dt = dt
            self.t_last = None

        self.z_std_drone = z_std_drone
        self.z_std_range = z_std_range
        self.initial_target = initial_target # The 0,0,0 cartesian coord in GPS as a GPS object

        # Get the formation properties from the variables file
        flight_parameters = get_parameters(is_local)
        self.drone_distance = flight_parameters['COMMON']['DRONE_DISTANCE']
        
        # Convert the list of GPS coordinates (representing the drones positions) to a list of cartesian coordinates
        drones_i_cart = np.array([])
        drones_i_cart_list = []
        for d in drones_i:
            new = self.GPS_to_cartesian(d)
            drones_i_cart_list.append(new) # This is a 5x3 matrix, which is easier to work with
            drones_i_cart = np.concatenate((drones_i_cart,new),axis=0) # This is a 1x15 vector for the filter
        # Simulate the first range reading
        ri = simulate_ranges(np.array([0,0,0]), drones_i_cart_list)

        
        # Append the drone positions and ranges to the initial measurment vector
        self.zi = np.concatenate((ri, drones_i_cart),axis=0)
        # create sigma points to use in the filter. This is standard for Gaussian processes
        points = MerweScaledSigmaPoints(36, alpha=.1, beta=2., kappa=-1)
        # Create the kalman filter object
        self.kf = UnscentedKalmanFilter(dim_x=36, dim_z=19, dt=self.dt, fx=fx, hx=hx, points=points)
        # Set the initial state of the swarm
        self.kf.x = np.concatenate((self.zi[4:19], np.zeros(21)), axis=0)
        # Set the initial uncertanty of the swarm
        P_drones = (self.z_std_drone**2)*np.identity(15)
        self.kf.P = block_diag(P_drones, initial_target_uncertanty*np.identity(3), np.identity(18)) # initial uncertainty
        # Create the measurment noise matrix
        R_ranges = (self.z_std_range**2)*np.identity(4)
        R_drones = (self.z_std_drone**2)*np.identity(15)
      
        self.kf.R = block_diag(R_ranges, R_drones) # How much we trust our measurments

        # Create the process noise matrix
        Q_drones = Q_discrete_white_noise(dim=2, dt=0.5, var=(self.z_std_drone)**2, block_size=18, order_by_dim=False)
        self.kf.Q = Q_drones # How much we trust predictions
        # Create the saver object to save the state of the filter after each run
        self.saver = Saver(self.kf)
        

    def get_swarm_item_pos(self, drone_num):
        """
        Returns a nx3 array containing all the filtered drone positions for drone number 'n' tx_pos=0, rx1_pos=1, rx2_pos=2, rx3_pos=3, rx4_pos=4, 
        est_target_pos=5, tx_vel=6, rx1_vel=7, rx2_vel=8, rx3_vel=9, rx4_vel=10, est_target_vel=11.
        """
        xs = np.array(self.saver.x)
        return get_item_coords(xs, drone_num)
    
    def est_target_pos(self, drones_GPS, ranges, dt=None):
        """ 
        Estimates the location of the target in 3D using TSOA multilateration. 
        Function takes the 3D GPS location of each UAV with the associated range readings.
        Calculates and returns the 3D location of target as a GPSCoord object. 
        """

        # Convert all the GPS inputs to cartesian
        drones = list(map(self.GPS_to_cartesian, drones_GPS))

        # Calculate the time since the last update
        if self.dt == None:
            dt = (datetime.now() - self.t_last).total_seconds()
        else:
            dt = self.dt

        # Predict the locations of the drones
        self.kf.predict(dt = dt)
        self.z = np.concatenate((ranges, drones[0], drones[1], drones[2], drones[3], drones[4]), axis=0)

        # # Detect any Dropped ranges and set their corosponding uncertanty to a very big number. This essentially makes the Kalman
        # # Filter use the estimate and not the measurment for its guess at the real location of the target.
        # if any(np.in1d(self.z,[-1])):
        #     indexes = np.where(self.z == -1)
        #     # Create the measurment noise matrix with the high uncertanty at the location of the invalid range
        #     R_ranges = (self.z_std_range**2)*np.identity(4)
        #     R_drones = (self.z_std_drone**2)*np.identity(15)
        #     R = block_diag(R_ranges, R_drones)
        #     R[indexes,indexes] = 9999999 # A big number which says this measurment is useless
        #     self.kf.update(self.z, R=R) # Update the filter with the new measurments
    

        # Extract the variance from the gps for the kalman filter to use
        R_drones = []
        for i, drone in enumerate(drones_GPS):
            if drone.var == [-1,-1,-1]:
                var_default = (self.z_std_drone**2)
                R_drones = R_drones + [var_default]*3
            else:
                R_drones = R_drones + drone.var

        R_drones_diag = np.diag(np.array(R_drones))
        R_ranges = (self.z_std_range**2)*np.identity(4)
        self.kf.R = block_diag(R_ranges, R_drones_diag) # How much we trust our measurments

        self.kf.update(self.z) # Update the filter with the new measurments

        self.t_last = datetime.now()
        self.saver.save() # Save the state of the filter to be observed later

        cart_target_pos = self.get_swarm_item_pos(5)
        GPS_target_pos = self.cartesian_to_GPS(cart_target_pos[-1, :])
        target_post_variences = self.kf.P_post[15:18,15:18]
        GPS_target_pos.var = list(np.diag(target_post_variences))
        return GPS_target_pos



import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main ():

    #Generate path variables 
    locations = 50 # Number of target locations to filter
    maxVelocity = 3
    maxAcceleration = 2
    maxAcceleration_dot = 1
    ts = 1 # Timestep between locations

    #Set initial conditions for localisation testing
    target_i = GPSCoord(-43.52051, 172.58312, 0)
    drones_i = []
    for j in range(0,5):
        ng = update_loc(target_i, j, target_i)
        drones_i.append(ng)


    #Generate target path
    target_path = generatePathv2(target_i, locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts)
    # Initialise the filter
    filt = PosFilter(drones_i, 0.5, 0.5, target_i, True, ts)

    # Loop through each target position
    for i, target_pos in enumerate(target_path):
        

        drones = []
        for j in range(0,5):
            g = to_GPS(target_pos)
            ng = update_loc(g, j, g)
            drones.append(ng)
        
        tp = filt.GPS_to_cartesian(to_GPS(target_pos))
        drones_cart = list(map(filt.GPS_to_cartesian, drones))
        ranges = simulate_ranges(tp, drones_cart)

        if filt.est_target_pos(drones, ranges, ts):
            continue
        else:
            print('filtering failed')
    

    # Create a cartesian version of the path for plotting
    target_path_cart = []
    for loc in target_path:
        target_path_cart.append(filt.GPS_to_cartesian(GPSCoord(loc[0], loc[1], loc[2])))

    # Extract the real and filtered paths from their matricies
    target_path_cart = np.array(target_path_cart)
    cart_path_filt = filt.get_swarm_item_pos(5)

    # Plot the real vs filtered path
    pos_plot = plt.figure(3)
    ax = pos_plot.add_subplot(111, projection='3d')   
    ax.plot(cart_path_filt[:, 0], cart_path_filt[:, 1], cart_path_filt[:, 2], 'magenta', label = 'filter') #estimated
    ax.plot(target_path_cart[:, 0], target_path_cart[:, 1], target_path_cart[:, 2], 'green', label = 'target') #real

    plt.legend()
    plt.show()

# If this file is being run, run main
if __name__ == "__main__":
    main()




    


