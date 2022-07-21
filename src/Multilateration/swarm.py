import math
import scipy.optimize as opt
import numpy as np

from gpsv2 import *


class Coord2D:
    def __init__(self, xcoord=0, ycoord=0):
        self.x = xcoord
        self.y = ycoord

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "{:.6f}m, {:.6f}m".format(self.x, self.y)

    def set_position_from_GPS(tx, pos):
        """
        Sets the position of the drone in meters relative to the transmitting drone
        """
        self.x = gps.x_distance(pos, tx)
        self.y = gps.y_distance(pos, tx)


class Swarm2D:
    def __init__(self, tx=None, rx1=None, rx2=None, rx3=None):
        self.rx1 = rx1
        self.rx2 = rx2
        self.rx3 = rx3
        self.tx = tx

    def TDOAfunc2D(self, x, r1, r2, r3):
        e1 = np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2) - np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2) - r1 + r2
        e2 = np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2) - np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2) - r1 + r3
        #e3 = np.sqrt((rx2Loc.x - x[0])**2 + (rx2Loc.y - x[1])**2) - np.sqrt((rx3Loc.x - x[0])**2 + (rx3Loc.y - x[1])**2) - r2 + r3
        return [e1, e2]

    def TSOAfunc2D(self, x, r1, r2, r3):
        e1 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2) + np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2) - r1
        e2 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2) + np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2) - r2
        e3 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2) + np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2) - r3
        return [e1, e2, e3]

    def find_target(self, r1, r2, r3, method='TDOA'):
        """
        Returns the location of the target based on the positions of the drones in the swarm and the supplied ranges
        May speed up in future by supplying the last known position as the initial estimate.
        """
        location = None
        if (method=='TDOA'):
            location = opt.fsolve(self.TDOAfunc2D, [10, 10], args=(r1,r2,r3))
        else:
            location = opt.least_squares(self.TSOAfunc2D, [10, 10], args=(r1,r2,r3)).x

        return Coord2D(location[0], location[1])


class Swarm3D:
    def __init__(self, tx=None, rx1=None, rx2=None, rx3=None, rx4=None):
        self.rx1 = rx1
        self.rx2 = rx2
        self.rx3 = rx3
        self.rx4 = rx4
        self.tx = tx

    def TDOAfunc3D(self, x, r1, r2, r3, r4):
        e1 = np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2 + (self.rx1.z - x[2])**2) - np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2 + (self.rx2.z - x[2])**2) - r1 + r2
        e2 = np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2 + (self.rx1.z - x[2])**2) - np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2 + (self.rx3.z - x[2])**2) - r1 + r3
        e3 = np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2 + (self.rx1.z - x[2])**2) - np.sqrt((self.rx4.x - x[0])**2 + (self.rx4.y - x[1])**2 + (self.rx4.z - x[2])**2) - r1 + r4
        e4 = np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2 + (self.rx2.z - x[2])**2) - np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2 + (self.rx3.z - x[2])**2) - r2 + r3
        e5 = np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2 + (self.rx2.z - x[2])**2) - np.sqrt((self.rx4.x - x[0])**2 + (self.rx4.y - x[1])**2 + (self.rx4.z - x[2])**2) - r2 + r4
        e6 = np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2 + (self.rx3.z - x[2])**2) - np.sqrt((self.rx4.x - x[0])**2 + (self.rx4.y - x[1])**2 + (self.rx4.z - x[2])**2) - r3 + r4
        
        return[e1, e2, e3, e4, e5, e6]

    def TSOAfunc3D(self, x, r1, r2, r3, r4):
        e1 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2 + (self.tx.z - x[2])**2) + np.sqrt((self.rx1.x - x[0])**2 + (self.rx1.y - x[1])**2 + (self.rx1.z - x[2])**2) - r1
        e2 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2 + (self.tx.z - x[2])**2) + np.sqrt((self.rx2.x - x[0])**2 + (self.rx2.y - x[1])**2 + (self.rx2.z - x[2])**2) - r2
        e3 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2 + (self.tx.z - x[2])**2) + np.sqrt((self.rx3.x - x[0])**2 + (self.rx3.y - x[1])**2 + (self.rx3.z - x[2])**2) - r3
        e4 = np.sqrt((self.tx.x - x[0])**2 + (self.tx.y - x[1])**2 + (self.tx.z - x[2])**2) + np.sqrt((self.rx4.x - x[0])**2 + (self.rx4.y - x[1])**2 + (self.rx4.z - x[2])**2) - r4
        return [e1, e2, e3, e4]

    def find_target(self, r1, r2, r3, r4, previous, method='TDOA'):
        """
        Returns the location of the target based on the positions of the drones in the swarm and the supplied ranges
        May speed up in future by supplying the last known position as the initial estimate.
        """
        location = None
        if (method=='TDOA'):
            location = opt.least_squares(self.TDOAfunc3D, previous, args=(r1,r2,r3,r4)).x
        else:
            location = opt.least_squares(self.TSOAfunc3D, previous, args=(r1,r2,r3,r4)).x
            
        return Coord3D([location[0], location[1], location[2]])


class Coord3D:
    def __init__(self, coord):
        self.x = coord[0] #xcoord
        self.y = coord[1] #ycoord
        self.z = coord[2] #zcoord

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "{:.3f} m, {:.3f} m, {:.3f} m".format(self.x, self.y, self.z)

    def add(self, coord):
        return Coord3D([self.x+coord.x, self.y+coord.y, self.z+coord.z])

    def sub(self, coord):
        return Coord3D([self.x-coord.x, self.y-coord.y, self.z-coord.z])

    def set_position_from_GPS(tx, pos):
        """
        Sets the position of the drone in meters relative to the transmitting drone
        Sets coordinates for transmitting (Mothership) drone to move to. Next step: arrange slave drones:
            Either: 1) save the previous coordinates and calculate the difference it's moved. Apply this difference to current coordinates of slave drones (take a moving average of the x_offset and (original coordinates + the difference)).
                    2) simply move with respect to mothership.
                    3) As rowan suggested, use interpolation to apply control theory; using integral control, or PID sort of stuff 
        """
        self.x = gps.x_distance(pos, tx)
        self.y = gps.y_distance(pos, tx)
        self.z = pos.alt


def norm(d1, d2):
    len = math.sqrt((d2.x-d1.x)**2 + (d2.y-d1.y)**2 + (d2.z-d1.z)**2)
    return len

def norm2D(d1, d2):
    len = math.sqrt((d2.x-d1.x)**2 + (d2.y-d1.y)**2)
    return len

def generatePathv2(target, locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts):
    """ Emulates a series of GPS coordinates that the target follows. 
    Currently forms a random 'insect like' line in 3D space."""
    maxAngularRate = 30
    SLOWDOWN = False
    SPEEDUP = True  
    
    acceleration = [np.random.uniform(0, maxAcceleration)]
    velocity = [0]
    theta = [0] # Z-xy plane angle, 0 measured on plane.
    phi = [0] #x-y angle, 0 measured at y axis. 
    
    zdirection = [0]
    ydirection = [1]
    xdirection = [0]

    xposition = 0
    yposition = 0
    zposition = 0

    lattitude = target.lat
    longitude = target.long
    alt = target.alt

    target_path = []
    target_path.append([target.lat, target.long, target.alt])

    for i in range(1, locations):

        velocity.append(velocity[i-1] + (acceleration[i-1])*ts)

        # Check velocity of target 
        if (velocity[i] >= maxVelocity):
            velocity[i] = maxVelocity
            SLOWDOWN = True
        elif (velocity[i] <= 1):
            velocity[i] = 1
            SPEEDUP = True
        else:
            SLOWDOWN = False
            SPEEDUP = False

        # Determine acceleration for next time step
        if (SPEEDUP == True):
            acceleration.append(maxAcceleration_dot*ts)
        elif (SLOWDOWN == True):
            acceleration.append(-maxAcceleration_dot*ts)
        else:
            acceleration.append(acceleration[i-1] + np.random.uniform(-maxAcceleration_dot*ts, maxAcceleration_dot*ts))

        # Direction update
        theta.append(theta[i-1] + np.random.uniform(-(maxAngularRate)*ts, (maxAngularRate)*ts))
        phi.append(phi[i-1] + np.random.uniform(-(maxAngularRate)*ts, (maxAngularRate)*ts))

        zdirection.append(np.sin((theta[i]/180)*np.pi))
        ydirection.append(np.cos((theta[i]/180)*np.pi)*np.cos((phi[i]/180)*np.pi))
        xdirection.append(np.cos((theta[i]/180)*np.pi)*np.sin((phi[i]/180)*np.pi))

        # Determine position change during timestep
        delta_x = xdirection[i]*velocity[i]*ts
        delta_y = ydirection[i]*velocity[i]*ts
        delta_z = zdirection[i]*velocity[i]*ts

        xposition = xposition + delta_x
        yposition = yposition + delta_y
        zposition = zposition + delta_z

        # Converting to GPS Coordinates. 
        longitude = target.add_x_offset(xposition).long
        lattitude = target.add_y_offset(yposition).lat
        alt =  target.add_z_offset(zposition).alt

        target_path.append([lattitude, longitude, alt]) 

    return target_path


def generatePath(locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts):
    """ Emulates a series of x,y,z coordinates that the target follows. 
    Currently forms a simple linear line in 3D space."""
    maxAngularRate = 30
    SLOWDOWN = False
    SPEEDUP = True

    acceleration = [np.random.uniform(0, maxAcceleration)]
    velocity = [0]
    theta = [0] # Z-xy plane angle, 0 measured on plane.
    phi = [0] #x-y angle, 0 measured at y axis. 
    
    xposition = 0
    yposition = 0
    zposition = 0
    zdirection = [0]
    ydirection = [1]
    xdirection = [0]

    targetPath = []
    targetPath.append([xposition, yposition, zposition])

    for i in range(1, locations):
        velocity.append(velocity[i-1] + (acceleration[i-1])*ts)

        #Check velocity of target 
        if (velocity[i] >= maxVelocity):
            velocity[i] = maxVelocity
            SLOWDOWN = True
        elif (velocity[i] <= 1):
            velocity[i] = 1
            SPEEDUP = True
        else:
            SLOWDOWN = False
            SPEEDUP = False

        #Determine acceleration for next time step
        if (SPEEDUP == True):
            acceleration.append(maxAcceleration_dot*ts)
        elif (SLOWDOWN == True):
            acceleration.append(-maxAcceleration_dot*ts)
        else:
            acceleration.append(acceleration[i-1] + np.random.uniform(-maxAcceleration_dot*ts, maxAcceleration_dot*ts))

        #Direction update
        theta.append(theta[i-1] + np.random.uniform(-(maxAngularRate)*ts, (maxAngularRate)*ts))
        phi.append(phi[i-1] + np.random.uniform(-(maxAngularRate)*ts, (maxAngularRate)*ts))

        zdirection.append(np.sin((theta[i]/180)*np.pi))
        ydirection.append(np.cos((theta[i]/180)*np.pi)*np.cos((phi[i]/180)*np.pi))
        xdirection.append(np.cos((theta[i]/180)*np.pi)*np.sin((phi[i]/180)*np.pi))

        xposition = xposition + xdirection[i]*velocity[i]*ts
        yposition = yposition + ydirection[i]*velocity[i]*ts
        zposition = zposition + zdirection[i]*velocity[i]*ts
        targetPath.append([xposition, yposition, zposition]) 
    return targetPath







