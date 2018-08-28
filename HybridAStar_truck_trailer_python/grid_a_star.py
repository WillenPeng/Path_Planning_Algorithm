# Basic Astar algorithm (will be updated some functions)
import numpy as np
from heapq import *
from scipy.spatial import KDTree
import math

from def_all import *

class node_s:
    def __init__(self, x, y, cost, pind):
        self.x = x  #x index
        self.y = y  #y index
        self.cost = cost  # cost
        self.pind = pind # parent index

def calc_dist_policy(gx, gy, ox, oy,reso, vr):
    """
    To get a map with the h cost (distance to goal node) of each grid (without obstacle)
    gx: goal x position [m]
    gy: goal y position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    vr: vehicle radius[m]
    """
    ngoal = node_s(int(round(gx/reso)), int(round(gy/reso)), 0.0, -1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)
    # print (obmap)
    # print (minx)
    # print (miny)
    # print (maxx)
    # print (maxy)
    # print (xw)
    # print (yw)

    open_set = {}
    closed_set = {}

    open_set[calc_index(ngoal, xw, minx, miny)] = ngoal


    motion = get_motion_model()
    nmotion = len(motion)
    pq =[]
    heappush(pq, (ngoal.cost, calc_index(ngoal, xw, minx, miny)))

    while (1):
        # print ("openset: ", open_set.keys())
        # print ("pq: ", pq)
        if len(open_set) == 0:
            print("Finish Search")
            break
        c_id = heappop(pq)[1]
        current = open_set[c_id]
        # print ("c_id: ", c_id)
        del open_set[c_id]
        closed_set[c_id] = current

        for i in range(nmotion): # expand search grid based on motion model
            node = node_s(current.x+int(motion[i,0]), current.y+int(motion[i,1]), current.cost+motion[i,2], c_id)
            # print ("nodex: ", node.x)
            # print ("nodey: ", node.y)
            if not verify_node(node, minx, miny, xw, yw, obmap):
                continue

            node_ind = calc_index(node, xw, minx, miny)
            # print ("node_ind: ", node_ind)
            # If it is already in the closed set, skip it
            if node_ind in closed_set:
                continue

            if node_ind in open_set:
                if open_set[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    open_set[node_ind].cost = node.cost
                    open_set[node_ind].pind = c_id
            else: # add to open_set set
                open_set[node_ind] = node
                heappush(pq, (node.cost, calc_index(node, xw, minx, miny)))

    pmap = calc_policy_map(closed_set, xw, yw, minx, miny)

    return pmap

def calc_policy_map(closed, xw, yw, minx, miny):

    pmap = np.full((xw, yw), np.Inf)

    for n in closed.values():
        pmap[n.x-minx-1, n.y-miny-1] = n.cost
    # print(pmap)
    return pmap

def calc_obstacle_map(ox, oy, reso, vr):
    # build an obstacle map and in each grid, it shows whether it has obstacle
    minx = int(round(np.min(ox)))
    miny = int(round(np.min(oy)))
    maxx = int(round(np.max(ox)))
    maxy = int(round(np.max(oy)))
    # print ("minx:", minx)
    # print ("miny:", miny)
    # print ("maxx:", maxx)
    # print ("maxy:", maxy)

    xwidth = maxx - minx
    ywidth = maxy - miny
    # xwidth = int(round(maxx - minx))
    # ywidth = int(round(maxy - miny))
    # print ("xwidth:", xwidth)
    # print ("ywidth:", ywidth)

    obmap = np.zeros((xwidth, ywidth))

    kdtree = KDTree(np.asarray([ox, oy]).T)
    for ix in range(xwidth):
        x = ix + minx + 1
        for iy in range(ywidth):
            y = iy + miny + 1
            onedist, idxs = kdtree.query([x, y], k=1)
            # print ("x: ",x)
            # print ("y: ",y)
            # print ("one: ",onedist)
            # print ("id: ",idxs)
            if onedist <= vr/reso:
                obmap[ix,iy] = 1

    # print ("calc_obstacle_map done")

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth

def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin)*xwidth + (node.x - xmin)

def get_motion_model():
    # dx, dy, cost
    motion=np.asarray([[1,0,1], 
                       [0,1,1],
                       [-1,0,1],
                       [0,-1,1],
                       [-1,-1,math.sqrt(2)],
                       [-1,1,math.sqrt(2)],
                       [1,-1,math.sqrt(2)],
                       [1,1,math.sqrt(2)]])
    return motion

def calc_cost(n, ngoal):
    return (n.cost + h(n.x - ngoal.x, n.y - ngoal.y))

def h(x, y):
    """
    Heuristic cost function
    """
    return math.sqrt(x**2+y**2)

def verify_node(node, minx, miny, xw, yw, obmap):

    if ((node.x - minx) >= xw):
        return 0
    elif ((node.x - minx) <= 0):
        return 0
    if ((node.y - miny) >= yw):
        return 0
    elif ((node.y - miny) <= 0):
        return 0
    #collision check
    if obmap[node.x-minx-1, node.y-miny-1]:
        return 0
    return 1
