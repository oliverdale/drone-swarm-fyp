import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np

from scipy.linalg import block_diag
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise, Saver


from swarm import generatePath


dt = 1                          # Timestep between measurments
z_std_drone = 1                 # standard devaition in drone position
z_std_range = 1                 # standard devaition in range measurments
num_pos = 100                   # Number of positions to perform the simulation with
initial_target_uncertanty = 1   # Initial standard deviation in the position of the target
drone_distance = 10             # Distance between the drones
drone_height = 10               # Height the drones are above the target
dropout_prob = 0.2              # Probabibility of a range reading dropout


# Initial positions of the drones. The target is assumed to be at (0,0,0)
tx_i = np.array([0,0,drone_height])
rx1_i = np.array([drone_distance,drone_distance,drone_height])
rx2_i = np.array([drone_distance,-drone_distance,drone_height])
rx3_i = np.array([-drone_distance,drone_distance,drone_height])
rx4_i = np.array([-drone_distance,-drone_distance,drone_height])

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

def simulate_ranges(target_loc, drones, noise=0, dropout=[False]*5):
    """
    This function simulates the ranges that each drone would measure given estimated drone positions and the real target position.
    It also adds noise and dropout to the readings by randomly setting some ranges to -1. In the real setup this function will be 
    replaced by the real radar readings with unrecieved readings also at -1. Dropout is a vector of Bools if dropout[1] = True
    range[0] = -1
    """
    # Calculate ranges plus noise
    # np.random.binomial(1,dropout_prob)
    rx_to_target = np.linalg.norm(drones[0]-target_loc)
    rs = []
    for i in range(1,5):
        if dropout[i]:
            rs.append(-1)
        else:
            rs.append(rx_to_target + np.linalg.norm(target_loc-drones[i]) + np.random.normal(0,noise,1)[0])

    return np.array(rs)


def simulate_drone_pos(center, drone_distance, drone_height, noise=0, dropout=[False]*5):
    """
    This function simulates the positions of the drones given a center point (noramlly the target location). It positions the drones in a square
    with edges of length 'drone_distance' at a height of 'drone_height' above the center. May optionally generate Gausian noise added to the
    (x,y,z) coordinates of the drones with standard deviation 'noise'. Returns 5 vectors with the positions of the drones. Will be replace with 
    cartesian-converted GPS coordinates from the real drones in the final setup. Dropout is a vector of Bools if dropout[1] = True drones[1] = -1.
    """
    drones = []
    pos_coeffs = [[0,0,1],[1,1,1],[1,-1,1],[-1,1,1],[-1,-1,1]]
    for i in range(0,5):
        if dropout[i]:
            drones.append([-1,-1,-1])
        else:
            drones.append(center + np.array([pos_coeffs[i][0]*drone_distance, \
                pos_coeffs[i][1]*drone_distance, \
                pos_coeffs[i][2]*drone_height]) \
            + np.random.normal(0,noise,1)[0])

    return drones


def get_item_coords(xs, item_num):
    """
    This function extracts the position of an object from the state estimate vector or matrix. As inputs it takes the state estimate vector xs
    and a number 0 through 10. The number alters what the function returns as follows: tx_pos=0, rx1_pos=1, rx2_pos=2, rx3_pos=3, rx4_pos=4, 
    est_target_pos=5, tx_vel=0, rx1_vel=1, rx2_vel=2, rx3_vel=3, rx4_vel=4, est_target_vel=5. If a matrix is input a nx3 matrix of (x,y,z) values over 
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

def generate_random_dropout(num_pos, dropout_prob):
    """
    This function creates a list of lists containing bools. Each list in the list corosponds to a timestep and each bool corosponds to
    a drone. If a bool is true the range and the position of the drone will be dropped and the filter will have to estimate it.
    """
    v=[]
    for i in range(0,num_pos):
        v.append(np.random.binomial(1,dropout_prob,size=5))
    return v

# Generate a target path for the drones to folow
target_path = generatePath(num_pos, 4, 4, 4, dt)
# Simulate the initial range readings for the drones. The first range readings must be good (no dropout)
ri = simulate_ranges(target_path[0], [tx_i, rx1_i, rx2_i, rx3_i, rx4_i], noise=z_std_range)
# Append the drone positions and ranges to the initial measurment vector
zi = np.concatenate((ri, tx_i, rx1_i, rx2_i, rx3_i, rx4_i),axis=0)
# create sigma points to use in the filter. This is standard for Gaussian processes
points = MerweScaledSigmaPoints(36, alpha=.1, beta=2., kappa=-1)
# Create the kalman filter object
kf = UnscentedKalmanFilter(dim_x=36, dim_z=19, dt=dt, fx=fx, hx=hx, points=points)
# Set the initial state of the swarm
kf.x = np.concatenate((zi[4:19], np.zeros(21)), axis=0) 
# Set the initial uncertanty of the swarm
P_drones = (z_std_drone**2)*np.identity(15)
kf.P = block_diag(P_drones, initial_target_uncertanty*np.identity(3), np.identity(18)) # initial uncertainty

# Create the measurment noise matrix
R_ranges = (z_std_range**2)*np.identity(4)
R_drones = (z_std_drone**2)*np.identity(15)
kf.R = block_diag(R_ranges, R_drones) # How much we trust our measurments

# Create the process noise matrix
Q_drones = Q_discrete_white_noise(dim=2, dt=dt, var=z_std_drone**2, block_size=18, order_by_dim=False)
kf.Q = Q_drones/10 # How much we trust predictions

# Create the saver object to save the state of the filter after each run
saver = Saver(kf)

# Testing dropout configurations
dropouts = generate_random_dropout(num_pos, 0)
for i in range(25,30):
    dropouts[i] = [True, True, False, False, False]


# This for loop runs through all the target path position vectors. In the real thing it will run every second as new measurments come in
for i, pos in enumerate(target_path):
    # Predict the locations of the drones
    kf.predict()
    # Extract the estimated target location from the estimate vector
    est_target_loc = get_item_coords(kf.x, 5)
    # Simulate the position of the drones around the estimated target location, WITH noise
    drones = simulate_drone_pos(est_target_loc, drone_distance, drone_height, noise=z_std_drone, dropout=dropouts[i])
    # Simulate the position of the drones around the estimated target location, WITHOUT noise
    drones_nn = simulate_drone_pos(est_target_loc, drone_distance, drone_height, 0)
    # Simulate ranges base on drone locations WITHOUT noise. This function adds its own noise so avoid stacking noise
    r = simulate_ranges(pos, drones_nn, noise=z_std_range)
    # Create the measurment vector
    z = np.concatenate((r, drones[0], drones[1],drones[2],drones[3],drones[4]), axis=0)
    # Detect any Dropped ranges and set thei corosponding uncertanty to a very big number. This essentially makes the Kalman
    # Filter use the estimate and not the measurment for its guess at the real location of the target.
    if any(np.in1d(z,[-1])):
        indexes = np.where(z == -1)
        # Create the measurment noise matrix with the high uncertanty at the location of the invalid range
        R_ranges = (z_std_range**2)*np.identity(4)
        R_drones = (z_std_drone**2)*np.identity(15)
        R = block_diag(R_ranges, R_drones)
        R[indexes,indexes] = 9999999
        kf.update(z, R=R) # Update the filter with the new measurments
    else:
        kf.update(z) # Update the filter with the new measurments

    saver.save() # Save the state of the filter to be observed later


xs = np.array(saver.x) # get the kf.x state in an np.array
zs = np.array(saver.z) # get the kf.x state in an np.array
ts = np.arange(0, dt*num_pos, dt)
tp = target_path

# tx=0, rx1=1, rx2=2, rx3=3, rx4=4, est_target_pos=5
txs = get_item_coords(xs, 0)
rx1s = get_item_coords(xs, 1)
rx2s = get_item_coords(xs, 2)
rx3s = get_item_coords(xs, 3)
rx4s = get_item_coords(xs, 4)
etp = get_item_coords(xs, 5)

pos_plot = plt.figure(1)
P_t_x = np.array(saver.P_prior)[:,15,15]
std = np.sqrt(P_t_x) * 2 # number of stds to plot
plt.plot(ts, zs[:,4], '.',label='measurment')
plt.plot(ts, txs[:, 0])
plt.plot(ts, txs[:, 0]-std, color='k', ls=':', lw=1)
plt.plot(ts, txs[:, 0]+std, color='k', ls=':', lw=1)
plt.fill_between(range(len(std)), txs[:, 0]-std, txs[:, 0]+std, facecolor='#ffff00', alpha=0.3)
plt.legend()


pos_plot = plt.figure(2)
ax = Axes3D(pos_plot)#plt.axes(projection='3d')
ax.plot3D(etp[:, 0], etp[:, 1], etp[:,2], 'magenta', label = 'target') #target
ax.plot3D(txs[:, 0], txs[:, 1], txs[:,2], '--b', alpha=0.3, label = 'tx') #tx
ax.plot3D(rx1s[:, 0], rx1s[:, 1], rx1s[:,2], '--r', alpha=0.3, label = 'rx1') #rx1
ax.plot3D(rx2s[:, 0], rx2s[:, 1], rx2s[:,2], '--g', alpha=0.3, label = 'rx2') #rx2
ax.plot3D(rx3s[:, 0], rx3s[:, 1], rx3s[:,2], '--k', alpha=0.3, label = 'rx3') #rx3
ax.plot3D(rx4s[:, 0], rx4s[:, 1], rx4s[:,2], '--y', alpha=0.3, label = 'rx4') #rx4
ax.plot3D(tp[:, 0], tp[:, 1], tp[:,2], 'green', label = 'real') #real
plt.legend()


pos_plot = plt.figure(3)
error = []
for i, pos in enumerate(target_path):
    error.append(np.linalg.norm(pos-etp[i]))

plt.plot(ts, error, label='error')

plt.show()