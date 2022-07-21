import math
import scipy.optimize as opt
import numpy as np
import time

class coord:
    def __init__(self, xcoord, ycoord, zcoord):
        self.x = xcoord
        self.y = ycoord
        self.z = zcoord

rx0Loc = coord(-10, 10, 1)
rx1Loc = coord(10, 10, 1)
rx2Loc = coord(10, -10, 1)
rx3Loc = coord(-10, -10, 1)
txLoc = coord(0, 0, 10)
targetLoc = coord(2, 2, -10)

def norm(d1, d2):
    len = math.sqrt((d2.x-d1.x)**2 + (d2.y-d1.y)**2 + (d2.z-d1.z)**2)
    return len

r0noise = np.random.normal(0,1,1)[0]
r1noise = np.random.normal(0,1,1)[0]
r2noise = np.random.normal(0,1,1)[0]
r3noise = np.random.normal(0,1,1)[0]

r0 = norm(targetLoc, rx0Loc) + norm(txLoc, targetLoc)  #+ r1noise
r1 = norm(targetLoc, rx1Loc) + norm(txLoc, targetLoc)  #+ r2noise
r2 = norm(targetLoc, rx2Loc) + norm(txLoc, targetLoc)  #+ r3noise
r3 = norm(targetLoc, rx3Loc) + norm(txLoc, targetLoc)  #+ r4noise
t = norm(txLoc, rx1Loc)

d10 = r1 - r0
d20 = r2 - r0
d30 = r3 - r0

t0 = time.time()
v_1 = np.array([2*(rx1Loc.x - rx0Loc.x), 2*(rx2Loc.x - rx0Loc.x), 2*(rx3Loc.x - rx0Loc.x), 2*(rx2Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx2Loc.x)])
v_2 = np.array([2*(rx1Loc.y - rx0Loc.y), 2*(rx2Loc.y - rx0Loc.y), 2*(rx3Loc.y - rx0Loc.y), 2*(rx2Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx2Loc.x)])
v_3 = np.array([2*(rx1Loc.z - rx0Loc.z), 2*(rx2Loc.z - rx0Loc.z), 2*(rx3Loc.z - rx0Loc.z), 2*(rx2Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx1Loc.x), 2*(rx3Loc.x - rx2Loc.x)])

b1 = r0**2 - r1**2 + (rx1Loc.x**2 + rx1Loc.y**2 + rx1Loc.z**2) - (rx0Loc.x**2 + rx0Loc.y**2 + rx0Loc.x**2)
b2 = r0**2 - r2**2 + (rx2Loc.x**2 + rx2Loc.y**2 + rx2Loc.z**2) - (rx0Loc.x**2 + rx0Loc.y**2 + rx0Loc.x**2)
b3 = r0**2 - r3**2 + (rx3Loc.x**2 + rx3Loc.y**2 + rx3Loc.z**2) - (rx0Loc.x**2 + rx0Loc.y**2 + rx0Loc.x**2)
b4 = r1**2 - r2**2 + (rx2Loc.x**2 + rx2Loc.y**2 + rx2Loc.z**2) - (rx1Loc.x**2 + rx1Loc.y**2 + rx1Loc.x**2)
b5 = r1**2 - r3**2 + (rx3Loc.x**2 + rx3Loc.y**2 + rx3Loc.z**2) - (rx1Loc.x**2 + rx1Loc.y**2 + rx1Loc.x**2)
b6 = r2**2 - r3**2 + (rx3Loc.x**2 + rx3Loc.y**2 + rx3Loc.z**2) - (rx2Loc.x**2 + rx2Loc.y**2 + rx2Loc.x**2)

b = np.array([b1, b2, b3, b4, b5, b6])

A = np.array([v_1, v_2, v_3]).T


x, residuals, rank, s = np.linalg.lstsq(A,b)
t1 = time.time()
total = t1-t0

print(x)
print(residuals)
print(total*1000)