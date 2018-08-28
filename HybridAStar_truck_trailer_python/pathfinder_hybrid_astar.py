# use hybrid astar method to find the path
from scipy.spatial import KDTree
import numpy as np
from heapq import *
import math

from def_all import *
from config import Config
from node import Node
import grid_a_star
import rs_path
import tralierlib
import animation

import time

class Path:
    def __init__(self, x, y, yaw, yaw1, direction, cost):
        self.x = x # x position [m]
        self.y = y # y position [m]
        self.yaw = yaw # yaw angle [rad]
        self.yaw1 = yaw1 # trailer angle [rad]
        self.direction = direction # direction forward: true, back false
        self.cost = cost # cost

def calc_hybrid_astar_path(sx, sy, syaw, syaw1, gx, gy, gyaw, gyaw1, ox, oy, xyreso, yawreso):
    """
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """
    ox = np.asarray(ox)
    oy = np.asarray(oy)
    
    syaw = rs_path.pi_2_pi(syaw)
    gyaw = rs_path.pi_2_pi(gyaw)

    kdtree = KDTree(np.asarray([ox, oy]).T)

    # get the configuration of whole map (length, width. obstacles)
    c = Config()
    ox, oy = c.calc_config(ox, oy, xyreso, yawreso)
    nstart = Node(int(round(sx/xyreso)), int(round(sy/xyreso)), int(round(syaw/yawreso)), 1, np.asarray([sx]), np.asarray([sy]), np.asarray([syaw]), np.asarray([syaw1]), np.asarray([1]), 0.0, 0.0, -1)
    ngoal = Node(int(round(gx/xyreso)), int(round(gy/xyreso)), int(round(gyaw/yawreso)), 1, np.asarray([gx]), np.asarray([gy]), np.asarray([gyaw]), np.asarray([gyaw1]), np.asarray([1]), 0.0, 0.0, -1)

    # To get a map with the h cost (distance to goal node) of each grid (without obstacle)
    h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)

    # print (h_dp)

    open_set = {}
    closed_set = {}
    fnode = None

    open_set[calc_index(nstart, c)] = nstart
    pq =[]
    heappush(pq, (calc_cost(nstart, h_dp, c), calc_index(nstart, c)))

    u, d = calc_motion_inputs()
    nmotion = len(u)

    while (1):
        starttime = time.time()
        # print ("pq: ",pq)
        print ("pq: ",len(pq))
        # print ("open_set: ",open_set)
        if len(open_set) == 0:
            print("Error: Cannot find path, No open set")
            return []

        c_id = heappop(pq)[1]
        current = open_set[c_id]
        #move current node from open to closed
        del open_set[c_id]
        closed_set[c_id] = current

        # use Reed-Shepp model to find a path between current node and goal node without obstacles, which will not run every time for computational reasons.
        isupdated, fpath = update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, gyaw1)
        print ("isupdated: ", isupdated)
        if isupdated: # found
            fnode = fpath
            break

        inityaw1 = current.yaw1[0]

        for i in range(nmotion):
            # print ("current.xind: ", current.xind, "  current.yind: ", current.yind, "  current.cost: ", current.cost, "  c_id: ", c_id, "  u[i]: ", u[i], "  d[i]: ", d[i])
            # For each node, it will have multiple possible points but with one x,y index
            node = calc_next_node(current, c_id, u[i], d[i], c)
            # print ("node.xind: ", node.xind, "  node.yind: ", node.yind, "  node.cost: ", node.cost)

            if not verify_index(node, c, ox, oy, inityaw1, kdtree): 
                continue

            node_ind = calc_index(node, c)

            # If it is already in the closed set, skip it
            if node_ind in closed_set:  
                continue

            if node_ind not in open_set:
                open_set[node_ind] = node
                heappush(pq, (calc_cost(node, h_dp, c), node_ind))
                
            else:
                if open_set[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    open_set[node_ind] = node
        endtime = time.time()
        print ("onetime: ", endtime-starttime)
    print("final expand node:", len(open_set) + len(closed_set))
    ##################################
    path = get_final_path(closed_set, fnode, nstart, c)

    # ====Animation=====
    animation.show_animation(path, ox, oy, sx, sy, syaw, syaw1, gx, gy, gyaw, gyaw1)

    return path

def update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, gyaw1):
    # use Reed-Shepp model to find a path between current node and goal node without obstacles, which will not run every time for computational reasons.
    apath = analystic_expantion(current, ngoal, c, ox, oy, kdtree)
    if apath != None:
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw =  apath.yaw[1:]
        steps = MOTION_RESOLUTION*apath.directions
        yaw1 = tralierlib.calc_trailer_yaw_from_xyyaw(apath.x, apath.y, apath.yaw, current.yaw1[-1], steps)
        if abs(rs_path.pi_2_pi(yaw1[-1] - gyaw1)) >= GOAL_TYAW_TH:
            return 0, None #no update

        fcost = current.cost + calc_rs_path_cost(apath, yaw1)
        fyaw1 = yaw1[1:]
        fpind = calc_index(current, c)

        fd = []
        for d in apath.directions[1:]:
            if d >= 0:
                fd.append(1)
            else:
                fd.append(0)

        fsteer = 0.0

        fpath = Node(current.xind, current.yind, current.yawind, current.direction, fx, fy, fyaw, fyaw1, fd, fsteer, fcost, fpind)

        return 1, fpath

    return 0, None #no update

def calc_rs_path_cost(rspath, yaw1):
    # calculate the path cost
    cost = 0.0
    for l in rspath.lengths:
        if l >= 0: # forward
            cost += l
        else: # back
            cost += abs(l) * BACK_COST
    
    # swich back penalty
    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i+1] < 0.0: # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S": # curve
            cost += STEER_COST*abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = np.zeros(nctypes)
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER
 
    for i in range(len(rspath.ctypes) - 1):
        cost += STEER_CHANGE_COST*abs(ulist[i+1] - ulist[i])
    # print ("cost: ", cost)
    cost += JACKKNIF_COST * sum([abs(rs_path.pi_2_pi(rspath.yaw[i] - yaw1[i])) for i in range(len(yaw1))])
    return cost

def analystic_expantion(n, ngoal, c, ox, oy, kdtree):

    sx = n.x[-1]
    sy = n.y[-1]
    syaw = n.yaw[-1]

    max_curvature = math.tan(MAX_STEER)/WB
    paths = rs_path.calc_paths(sx,sy,syaw,ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1], max_curvature, step_size=MOTION_RESOLUTION)

    if len(paths) == 0:
        return None

    pathqueue = []
    for path in paths:

        steps = MOTION_RESOLUTION*path.directions
        yaw1 = tralierlib.calc_trailer_yaw_from_xyyaw(path.x, path.y, path.yaw, n.yaw1[-1], steps)
        heappush(pathqueue, (calc_rs_path_cost(path, yaw1), path))
        # print ("cost: ", calc_rs_path_cost(path, yaw1))
        # print ("yaw1: ", yaw1)

    for i in range(len(pathqueue)):
        path = heappop(pathqueue)[1]
        steps = MOTION_RESOLUTION*path.directions
        yaw1 = tralierlib.calc_trailer_yaw_from_xyyaw(path.x, path.y, path.yaw, n.yaw1[-1], steps)
        ind = range(0, len(path.x), SKIP_COLLISION_CHECK)
        if tralierlib.check_trailer_collision(ox, oy, path.x[ind], path.y[ind], path.yaw[ind], yaw1[ind], kdtree=kdtree):
            # plot(path.x, path.y, "-^b")
            return path # path is ok

    return None

def calc_motion_inputs():
    # get the motion around current node with length 1 and different heading degree
    up = [i for i in np.arange(MAX_STEER/N_STEER, MAX_STEER + MAX_STEER/N_STEER, MAX_STEER/N_STEER)]
    u = [0.0] + up + [-i for i in up]
    d = [1.0 for i in u] + [-1.0 for i in u]
    u = u + u

    return u, d

def verify_index(node, c, ox, oy, inityaw1, kdtree):

    # overflow map
    if (node.xind - c.minx) >= c.xw:
        return 0
    elif (node.xind - c.minx) <= 0:
        return 0

    if (node.yind - c.miny) >= c.yw:
        return 0
    elif (node.yind - c.miny) <= 0:
        return 0

    # check collisiton
    steps = MOTION_RESOLUTION*np.asarray(node.directions)
    yaw1 = tralierlib.calc_trailer_yaw_from_xyyaw(node.x, node.y, node.yaw, inityaw1, steps)
    ind = range(0, len(node.x), SKIP_COLLISION_CHECK)
    if not tralierlib.check_trailer_collision(ox, oy, node.x[ind], node.y[ind], node.yaw[ind], yaw1[ind], kdtree = kdtree):
        return 0

    return 1 #index is ok"

def calc_next_node(current, c_id, u, d, c):

    arc_l = XY_GRID_RESOLUTION*1.5

    nlist = int(math.floor(arc_l/MOTION_RESOLUTION)) + 1
    xlist = np.zeros(nlist)
    ylist = np.zeros(nlist)
    yawlist = np.zeros(nlist)
    yaw1list = np.zeros(nlist)

    xlist[0] = current.x[-1] + d * MOTION_RESOLUTION*math.cos(current.yaw[-1])
    ylist[0] = current.y[-1] + d * MOTION_RESOLUTION*math.sin(current.yaw[-1])
    yawlist[0] = rs_path.pi_2_pi(current.yaw[-1] + d*MOTION_RESOLUTION/WB * math.tan(u))
    yaw1list[0] = rs_path.pi_2_pi(current.yaw1[-1] + d*MOTION_RESOLUTION/LT*math.sin(current.yaw[-1]-current.yaw1[-1]))
 
    for i in range(nlist-1):
        xlist[i+1] = xlist[i] + d * MOTION_RESOLUTION*math.cos(yawlist[i])
        ylist[i+1] = ylist[i] + d * MOTION_RESOLUTION*math.sin(yawlist[i])
        yawlist[i+1] = rs_path.pi_2_pi(yawlist[i] + d*MOTION_RESOLUTION/WB * math.tan(u))
        yaw1list[i+1] = rs_path.pi_2_pi(yaw1list[i] + d*MOTION_RESOLUTION/LT*math.sin(yawlist[i]-yaw1list[i]))
 
    xind = int(round(xlist[-1]/c.xyreso))
    yind = int(round(ylist[-1]/c.xyreso))
    yawind = int(round(yawlist[-1]/c.yawreso))


    addedcost = 0.0
    if d > 0:
        direction = 1
        addedcost += abs(arc_l)
    else:
        direction = 0
        addedcost += abs(arc_l) * BACK_COST

    # switch back penalty
    if direction != current.direction: # switch back penalty
        addedcost += SB_COST

    # steer penalty
    addedcost += STEER_COST*abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current.steer - u)

    # jacknif cost
    addedcost += JACKKNIF_COST * sum([abs(rs_path.pi_2_pi(yawlist[i] - yaw1list[i])) for i in range(len(yawlist))])

    cost = current.cost + addedcost 

    directions = [direction for i in range(len(xlist))]
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, yaw1list, directions, u, cost, c_id)

    return node

def is_same_grid(node1, node2):

    if node1.xind != node2.xind:
        return 0        
    if node1.yind != node2.yind:
        return 0     
    if node1.yawind != node2.yawind:
        return 0
    return 1

def calc_index(node, c):
    # calculate the index of node which count x, y, yaw and yaw1 to code index (4D)
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)

    # 4D grid
    yaw1ind = int(round(node.yaw1[-1]/c.yawreso))
    ind += (yaw1ind - c.minyawt) *c.xw*c.yw*c.yaww

    if ind <= 0:
        print("Error(calc_index): ", ind)
    return ind

def calc_holonomic_with_obstacle_heuristic(gnode, ox, oy, xyreso):
    # To get a map with the h cost (distance to goal node) of each grid (without obstacle)
    h_dp = grid_a_star.calc_dist_policy(gnode.x[-1], gnode.y[-1], ox, oy, xyreso, 1.0)
    return h_dp

def get_final_path(closed, ngoal, nstart, c):
    # ngoal is the result from Analytic Expansions and the final path is the combination of points in ngoal and point in closed set
    rx, ry, ryaw = np.flip(ngoal.x,0), np.flip(ngoal.y,0), np.flip(ngoal.yaw,0)
    ryaw1 = np.flip(ngoal.yaw1,0)
    direction = np.flip(ngoal.directions,0)
    nid = ngoal.pind
    finalcost = ngoal.cost

    while (1):
        n = closed[nid]
        rx = np.append(rx, np.flip(n.x,0))
        ry = np.append(ry, np.flip(n.y,0))
        ryaw = np.append(ryaw, np.flip(n.yaw,0))
        ryaw1 = np.append(ryaw1, np.flip(n.yaw1,0))
        direction = np.append(direction, np.flip(n.directions,0))
        nid = n.pind
        if is_same_grid(n, nstart):
            break

    rx = np.flip(rx,0)
    ry = np.flip(ry,0)
    ryaw = np.flip(ryaw,0)
    ryaw1 = np.flip(ryaw1,0)
    direction = np.flip(direction,0)

    # adjuct first direction
    direction[0] = direction[1]

    path = Path(rx, ry, ryaw, ryaw1, direction, finalcost)

    return path

def calc_cost(n, h_dp, c):
    # get the f cost of node n
    # print ("xind: ", n.xind)
    # print ("minx: ", c.minx)
    # print ("h_dp: ", h_dp.shape)
    return (n.cost + H_COST*h_dp[n.xind - c.minx-1, n.yind - c.miny-1])
