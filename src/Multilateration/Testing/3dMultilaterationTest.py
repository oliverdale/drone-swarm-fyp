from plot import *
from swarm import *
import time

TDOAtotal = 0
TSOAtotal = 0
TDOAerror = 0
TSOAerror = 0

rx1Path = []
rx2Path = []
rx3Path = []
rx4Path = []
txPath = []

locations = 10

TDOA_est_target_loc = []
TSOA_est_target_loc = []

#Generate path variables 
locations = 10
maxVelocity = 3
maxAcceleration = 2
maxAcceleration_dot = 1
ts = 1

targetPath = generatePath(locations, maxVelocity, maxAcceleration, maxAcceleration_dot, ts)

for i in range(0, locations-1):
    if i == 0:
        prevTSOA = [0, 0, 0]
        prevTDOA = [0, 0, 0]
    else:
        prevTSOA = [TSOA_est_target_loc[i-1].x, TSOA_est_target_loc[i-1].y, TSOA_est_target_loc[i-1].z]
        prevTDOA = [TDOA_est_target_loc[i-1].x, TDOA_est_target_loc[i-1].y, TDOA_est_target_loc[i-1].z]

    rx1Loc = Coord3D(np.add([-15, 15, -2], prevTSOA))
    rx2Loc = Coord3D(np.add([15, 15, -2], prevTSOA))
    rx3Loc = Coord3D(np.add([15, -15, -2], prevTSOA))
    rx4Loc = Coord3D(np.add([-15, -15, -2], prevTSOA))
    txLoc = Coord3D(np.add([2, 3, 10], prevTSOA))
    targetLoc = Coord3D(targetPath[i])

    rx1Path.append(rx1Loc)
    rx2Path.append(rx2Loc)
    rx3Path.append(rx3Loc)
    rx4Path.append(rx4Loc)
    txPath.append(txLoc)

    """ Determining radar ranges with simulated noise. """
    r1noise = np.random.normal(0,1,1)[0]
    r2noise = np.random.normal(0,1,1)[0]
    r3noise = np.random.normal(0,1,1)[0]
    r4noise = np.random.normal(0,1,1)[0]

    r1 = norm(txLoc, targetLoc) + norm(targetLoc, rx1Loc) + r1noise
    r2 = norm(txLoc, targetLoc) + norm(targetLoc, rx2Loc) + r2noise
    r3 = norm(txLoc, targetLoc) + norm(targetLoc, rx3Loc) + r3noise
    r4 = norm(txLoc, targetLoc) + norm(targetLoc, rx4Loc) + r4noise
    t = norm(txLoc, rx1Loc)

    #Setting up swarm 
    current_swarm = Swarm3D(txLoc, rx1Loc, rx2Loc, rx3Loc, rx4Loc)

    #Calculating TDOA running time
    TDOA0 = time.time()
    TDOA_est_target_loc.append(current_swarm.find_target(r1, r2, r3, r4, prevTDOA))
    TDOA1 = time.time()

    TDOAtotal = (TDOA1-TDOA0) + TDOAtotal

    #Calculating TSOA running time
    TSOA0 = time.time()
    TSOA_est_target_loc.append(current_swarm.find_target(r1, r2, r3, r4, prevTSOA, 'TSOA'))
    TSOA1 = time.time()

    TSOAtotal = (TSOA1-TSOA0) + TSOAtotal

    #Calculating TDOA error 
    TDOAxError = abs(targetLoc.x-TDOA_est_target_loc[i].x)
    TDOAyError = abs(targetLoc.y-TDOA_est_target_loc[i].y)
    TDOAzError = abs(targetLoc.z-TDOA_est_target_loc[i].z)

    TDOAerror = math.sqrt(TDOAxError**2+TDOAyError**2+TDOAzError**2) + TDOAerror

    #Calculating TSOA error 
    TSOAxError = abs(targetLoc.x-TSOA_est_target_loc[i].x)
    TSOAyError = abs(targetLoc.y-TSOA_est_target_loc[i].y)
    TSOAzError = abs(targetLoc.z-TSOA_est_target_loc[i].z)

    TSOAerror = math.sqrt(TSOAxError**2+TSOAyError**2+TSOAzError**2) + TSOAerror

    """Plotting the drone and target locations in 3D."""
    #plot_drones3D(rx1Loc, rx2Loc, rx3Loc, rx4Loc, txLoc, targetLoc, TSOA_est_target_loc[i])

plot_path3D(rx1Path, rx2Path, rx3Path, rx4Path, txPath, targetPath, TSOA_est_target_loc, locations)


print("-----TSOA-----")
print("Average Computation Time (ms): ", (TSOAtotal/iterations)*1000)
print("Average Estimisation Error (m): ", TSOAerror/iterations)
print("Computed Location: ", [TSOA_est_target_loc[i].x, TSOA_est_target_loc[i].y, TSOA_est_target_loc[i].z])
print("Actual Location: ", [targetLoc.x, targetLoc.y, targetLoc.z])
print()

print("-----TDOA-----")
print("Average Computation Time (ms): ", (TDOAtotal/iterations)*1000)
print("Average Estimisation Error (m): ", TDOAerror/iterations)
print("Computed Location: ", [TDOA_est_target_loc[i].x, TDOA_est_target_loc[i].y, TDOA_est_target_loc[i].z])
print("Actual Location: ", [targetLoc.x, targetLoc.y, targetLoc.z])
print()

print('Average Noise: ', (abs(r1noise)+abs(r2noise)+abs(r3noise)+abs(r4noise))/4)



    



