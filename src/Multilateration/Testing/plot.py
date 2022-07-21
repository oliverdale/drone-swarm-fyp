import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from mpl_toolkits.mplot3d.proj3d import proj_transform
from matplotlib.text import Annotation

class Annotation3D(Annotation):
    '''Annotate the point xyz with text s'''

    def __init__(self, s, xyz, *args, **kwargs):
        Annotation.__init__(self,s, xy=(0,0), *args, **kwargs)
        self._verts3d = xyz        

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.xy=(xs,ys)
        Annotation.draw(self, renderer)

def annotate3D(ax, s, *args, **kwargs):
    '''add anotation text to to Axes3d ax'''

    tag = Annotation3D(s, *args, **kwargs)
    ax.add_artist(tag)

def plot_drones3D(rx1Loc, rx2Loc, rx3Loc, rx4Loc, txLoc, tarLoc, estLoc):
    
    # data: coordinates of nodes and links
    xn = [rx1Loc.x, rx2Loc.x, rx3Loc.x, rx4Loc.x, txLoc.x, tarLoc.x, estLoc.x]
    yn = [rx1Loc.y, rx2Loc.y, rx3Loc.y, rx4Loc.y, txLoc.y, tarLoc.y, estLoc.y]
    zn = [rx1Loc.z, rx2Loc.z, rx3Loc.z, rx4Loc.z, txLoc.z, tarLoc.z, estLoc.z]
    group = [1, 1, 1, 1,2,3,4]
    edges = [(0, 5), (1, 5), (2, 5), (3, 5), (4, 5)]
    xyzn = list(zip(xn, yn, zn))
    segments = [(xyzn[s], xyzn[t]) for s, t in edges]         
    labels = ['rx1', 'rx2', 'rx3', 'rx4', 'tx', 'loc', 'est']       

    # create figure        
    fig = plt.figure(dpi=60)
    ax = fig.gca(projection='3d')
    
    # plot vertices
    ax.scatter(xn,yn,zn, marker='o', c = group, s = 64)    
    # plot edges
    edge_col = Line3DCollection(segments, lw=0.2)
    ax.add_collection3d(edge_col)
    # add vertices annotation.
    for j, xyz_ in enumerate(xyzn): 
        annotate3D(ax, s=labels[j], xyz=xyz_, fontsize=10, xytext=(-3,3),
                textcoords='offset points', ha='right',va='bottom')    
    plt.show()

def plot_path3D(rx1Path, rx2Path, rx3Path, rx4Path, txPath, tarPath, estPath, locations):
    # create figure        
    fig = plt.figure(dpi=60)
    ax = fig.gca(projection='3d')
    rx1xn = []
    rx1yn = []
    rx1zn = []

    for i in range(locations-1):
        # data: coordinates of nodes and links
        xn = [rx1Path[i].x, rx2Path[i].x, rx3Path[i].x, rx4Path[i].x, txPath[i].x, tarPath[i][0], estPath[i].x]
        yn = [rx1Path[i].y, rx2Path[i].y, rx3Path[i].y, rx4Path[i].y, txPath[i].y, tarPath[i][1], estPath[i].y]
        zn = [rx1Path[i].z, rx2Path[i].z, rx3Path[i].z, rx4Path[i].z, txPath[i].z, tarPath[i][2], estPath[i].z]
        group = [1, 1, 1, 1,2,3,4]
        edges = [(0, 5), (1, 5), (2, 5), (3, 5), (4, 5)]
        xyzn = list(zip(xn, yn, zn))
        segments = [(xyzn[s], xyzn[t]) for s, t in edges]         
        labels = ['rx1', 'rx2', 'rx3', 'rx4', 'tx', 'loc', 'est']  

        rx1xn.append(rx1Path[i].x)    
        rx1yn.append(rx1Path[i].y)
        rx1zn.append(rx1Path[i].z) 
        
        # plot vertices
        ax.scatter(xn,yn,zn, marker='o', c = group, s = 64)  
        
        # plot edges
        edge_col = Line3DCollection(segments, lw=0.2)
        ax.add_collection3d(edge_col)
        # add vertices annotation.
        for j, xyz_ in enumerate(xyzn): 
            annotate3D(ax, s=labels[j], xyz=xyz_, fontsize=10, xytext=(-3,3),
                    textcoords='offset points', ha='right',va='bottom')   
    ax.plot(rx1xn, rx1yn, rx1zn) 
    plt.show()

def plot_drones2D(rx1Loc, rx2Loc, rx3Loc, txLoc, tarLoc, estLoc):
    xn = [rx1Loc.x, rx2Loc.x, rx3Loc.x, txLoc.x, tarLoc.x, estLoc.x]
    yn = [rx1Loc.y, rx2Loc.y, rx3Loc.y, txLoc.y, tarLoc.y, estLoc.y]
    edges = [(0, 5), (1, 5), (2, 5), (3, 5)]
    xyn = list(zip(xn, yn))
    segments = [(xyn[s], xyn[t]) for s, t in edges]   
    labels = ['rx1', 'rx2', 'rx3', 'tx', 'loc', 'est'] 
    colours = ['ro', 'ro', 'ro', 'bo', 'ko', 'go']

    for j in range(len(xn)):
        plt.plot(xn[j], yn[j], colours[j])
        plt.annotate(labels[j], (xn[j], yn[j]))
        

    for j in range(len(edges)): 
        plt.plot((xn[edges[j][0]], xn[edges[j][1]]), (yn[edges[j][0]], yn[edges[j][1]]), '--k', alpha=0.3)

    plt.show()