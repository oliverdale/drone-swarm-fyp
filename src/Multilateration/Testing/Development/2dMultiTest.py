import matplotlib.pyplot 

from swarm import *
from plot import plot_drones2D
#from mayavi import mlab

matplotlib.pyplot.close('all')

rx1Loc = Coord2D(0, 5)
rx2Loc = Coord2D(15, -5)
rx3Loc = Coord2D(-5, -5)
txLoc = Coord2D(10, 0)
targetLoc = Coord2D(5, 0)

r1 = norm(txLoc, targetLoc) + norm(targetLoc, rx1Loc) + np.random.normal(0,1,1)[0]
r2 = norm(txLoc, targetLoc) + norm(targetLoc, rx2Loc) + np.random.normal(0,1,1)[0]
r3 = norm(txLoc, targetLoc) + norm(targetLoc, rx3Loc) + np.random.normal(0,1,1)[0]

current_swarm = Swarm2D(tx=txLoc, rx1=rx1Loc, rx2=rx2Loc, rx3=rx3Loc)
est_target_loc = current_swarm.find_target(r1, r2, r3, 'TDOA')

print(est_target_loc)

plot_drones2D(rx1Loc, rx2Loc, rx3Loc, txLoc, targetLoc, est_target_loc)

# delta = 0.25
# x1, y1 = np.meshgrid(np.arange(-25, 25, delta), np.arange(-25, 25, delta))
# #matplotlib.pyplot.contour(x1, y1, np.sqrt((rx1Loc.x - x1)**2 + (rx1Loc.y - y1)**2) - np.sqrt((rx2Loc.x - x1)**2 + (rx2Loc.y - y1)**2) - r1 + r2,[0])
# #matplotlib.pyplot.contour(x1, y1, np.sqrt((rx1Loc.x - x1)**2 + (rx1Loc.y - y1)**2) - np.sqrt((rx3Loc.x - x1)**2 + (rx3Loc.y - y1)**2) - r1 + r3,[0])
# #matplotlib.pyplot.contour(x1, y1, np.sqrt((rx2Loc.x - x1)**2 + (rx2Loc.y - y1)**2) - np.sqrt((rx3Loc.x - x1)**2 + (rx3Loc.y - y1)**2) - r2 + r3,[0])

# plot_drones2D(rx1Loc, rx2Loc, rx3Loc, txLoc, targetLoc, coord(locationTSOA.x[0], locationTSOA.x[1]))

# # matplotlib.pyplot.contour(x1, y1, np.sqrt((rx1Loc.x - x1)**2 + (rx1Loc.y - y1)**2) + np.sqrt((txLoc.x - x1)**2 + (txLoc.y - y1)**2) - r1,[0])
# # matplotlib.pyplot.contour(x1, y1, np.sqrt((rx2Loc.x - x1)**2 + (rx2Loc.y - y1)**2) + np.sqrt((txLoc.x - x1)**2 + (txLoc.y - y1)**2) - r2,[0])
# # matplotlib.pyplot.contour(x1, y1, np.sqrt((rx3Loc.x - x1)**2 + (rx3Loc.y - y1)**2) + np.sqrt((txLoc.x - x1)**2 + (txLoc.y - y1)**2) - r3,[0])

# # matplotlib.pyplot.show()





