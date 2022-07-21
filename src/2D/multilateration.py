import math
from scipy import optimize
import time

from gps import GPSCoord

#current problems: bad formatting converting lat/long to xy
#inaccurate method, won't work if they're too far away
#using 100000 to convert lat/long to xyz, if units are different won't work

def createGrid(xRange, yRange):
    #Create a grid

    grid = []
    for x in range(xRange):
        for y in range(yRange):
            grid.append([x, y, "N"])

    return grid


def placeObjects(grid, Tx, Rx1, Rx2, Rx3, Target):
    #Place transmitter, receivers and target on grid

    for entry in grid:

        if entry[0] == Tx[0] and entry[1] == Tx[1]:
            entry[2] = 'Tx'

        if entry[0] == Rx1[0] and entry[1] == Rx1[1]:
            entry[2] = 'Rx1'

        if entry[0] == Rx2[0] and entry[1] == Rx2[1]:
            entry[2] = 'Rx2'

        if entry[0] == Rx3[0] and entry[1] == Rx3[1]:
            entry[2] = 'Rx3'

        if entry[0] == Target[0] and entry[1] == Target[1]:
            entry[2] = 'Tar'

    return grid


def findNorm(first, second):
    #find distance between 2 points
    norm = math.sqrt((first[0] - second[0])**2 + (first[1] - second[1])**2)

    return norm

def bruteForce(grid, Tx, receiverArray, rangeArray, numDrones):

    LArray = []
    
    for entry in grid:
        
        currentRange = [None] * numDrones
        L = [None] * numDrones
        for i in range(0, numDrones):
            currentRange[i] = findNorm(Tx, entry) + findNorm(receiverArray[i], entry)
            L[i] = (currentRange[i] - rangeArray[i])**2
            
        Ltotal = sum(L) / numDrones
        LArray.append(Ltotal)

    target = grid[LArray.index(min(LArray))]
    return target

def calcX(RxiCoord, TxCoord, Tx):
    #converts gps coords to cartesian X coord
    x = TxCoord.x_distance(RxiCoord)
    xGrid = Tx[0] + x
    return round(xGrid)

def calcY(RxiCoord, TxCoord, Tx):
    #converts gps coords to cartesian Y coord
    y = TxCoord.y_distance(RxiCoord)
    yGrid = Tx[1] + y
    return round(yGrid)

def cartesianToLatLong(target, Tx, TxCoord):
    #converts cartesian coords to GPS coords
    x = target[0] - Tx[0]
    y = target[1] - Tx[1]
    return TxCoord.add_x_offset(x).add_y_offset(y)
    
def estimate_target_position(TxCoord, RxCoordArray, rangeArray, numDrones):
    #start = time.time()
    #Determine size of grid (metres), create grid
    xRange = 100
    yRange = 100
    grid = createGrid(xRange, yRange)
    
    #Transmitter drone always centre of grid
    Tx = (xRange/2, yRange/2)
    
    #Map from GPS to grid locations
    Rx = [None] * numDrones
    for i in range(0, numDrones):
        Rx[i] = (calcX(RxCoordArray[i], TxCoord, Tx), calcY(RxCoordArray[i], TxCoord, Tx))
    
    #Perform Calculations
    result = bruteForce(grid, Tx, Rx, rangeArray, numDrones)

    #convert back to GPS coord
    result = cartesianToLatLong(result, Tx, TxCoord)
    #end = time.time()
    #print(end - start)    
    return result

def root_estimate():
    start = time.time()
    
    #scipy.optimise.root(fun, args = [500,500], x0 = [250,250], method = 'hybr', tol = 1)
    
    end = time.time()
    print(end - start)       
    return

def main():
    #Determine size of grid (metres), create grid (using smaller grid here for ease of testing)
    xRange = 500
    yRange = 500
    grid = createGrid(xRange, yRange)

    #Transmitter drone always centre of grid
    Tx = (xRange/2, yRange/2)
    
    #Set conditions for localisation testing
    TxCoord = GPSCoord(-43.52051, 172.58310)

    Rx1Coord = GPSCoord(-43.52046, 172.58305)
    Rx2Coord = GPSCoord(-43.52046, 172.58315)
    Rx3Coord = GPSCoord(-43.52056, 172.58305)

    targetCoord = GPSCoord(-43.52059, 172.58315)

    #Map from GPS to grid locations
    Rx1 = (calcX(Rx1Coord, TxCoord, Tx), calcY(Rx1Coord, TxCoord, Tx))
    Rx2 = (calcX(Rx2Coord, TxCoord, Tx), calcY(Rx2Coord, TxCoord, Tx))
    Rx3 = (calcX(Rx3Coord, TxCoord, Tx), calcY(Rx3Coord, TxCoord, Tx))
    target = (calcX(targetCoord, TxCoord, Tx), calcY(targetCoord, TxCoord, Tx))
    
    grid = placeObjects(grid, Tx, Rx1, Rx2, Rx3, target)

    #calculate true ranges, in practical test this will be given as the output of the SDR
    r1 = findNorm(Tx, target) + findNorm(Rx1, target)
    r2 = findNorm(Tx, target) + findNorm(Rx2, target)
    r3 = findNorm(Tx, target) + findNorm(Rx3, target)
    
    rangeArray = [r1, r2, r3, None, None]
    RxCoordArray = [Rx1Coord, Rx2Coord, Rx3Coord, None, None]
    #Perform Calculations
    result = estimate_target_position(TxCoord, RxCoordArray, rangeArray, 3)
    
    print(result)
    print(grid)

if __name__ == "__main__":
    main()
