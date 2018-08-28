# path class
import numpy as np
import math

from def_all import *

STEP_SIZE = 0.1
MAX_PATH_LENGTH = 1000.0

class Path:
    def __init__(self, lengths, ctypes, L, x, y, yaw, directions):
        self.lengths = lengths #lengths of each part of the path +: forward, -: backward
        self.ctypes = ctypes # type of each part of the path
        self.L = L # total path length
        self.x = x # final x positions [m]
        self.y = y # final y positions [m]
        self.yaw = yaw # final yaw angles [rad]
        self.directions = directions # forward:1, backward:-1 

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size = STEP_SIZE):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    # use Reed-Shepp model to find multiple paths between current node and goal node without obstacles, but each path only contain the length, types and final position without each point in the path
    paths = generate_path(q0, q1, maxc)
    for path in paths:
        # get local coordinates of all points in each path
        x, y, yaw, directions = generate_local_course(path.L, path.lengths, path.ctypes, maxc, step_size*maxc)

        # convert global coordinate
        # path.x = np.asarray([math.cos(-q0[2]) * ix + math.sin(-q0[2]) * iy + q0[0] for (ix, iy) in zip(x, y)])
        # path.y = np.asarray([-math.sin(-q0[2]) * ix + math.cos(-q0[2]) * iy + q0[1] for (ix, iy) in zip(x, y)])
        # path.yaw = np.asarray([pi_2_pi(iyaw + q0[2]) for iyaw in yaw])
        xpath = []
        ypath = []
        yawpath = []
        for (ix, iy, iyaw) in zip(x, y, yaw):
            xpath.append(math.cos(-q0[2]) * ix + math.sin(-q0[2]) * iy + q0[0])
            ypath.append(-math.sin(-q0[2]) * ix + math.cos(-q0[2]) * iy + q0[1])
            yawpath.append(pi_2_pi(iyaw + q0[2]))        
        path.x = np.asarray(xpath)
        path.y = np.asarray(ypath)
        path.yaw = np.asarray(yawpath)
        path.directions = np.asarray(directions)
        path.lengths = np.asarray([l/maxc for l in path.lengths])
        path.L = path.L/maxc
    
    return paths

def polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta

def mod2pi(x):
    v = np.mod(x, 2.0 * math.pi)
    if v < -math.pi:
        v += 2.0 * math.pi
    else:
        if v > math.pi:
            v -= 2.0 * math.pi
    return v

def LSL(x, y, phi):
    u, t = polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= 0.0:
        v = mod2pi(phi - t)
        if v >= 0.0:
            return 1, t, u, v
    return 0, 0.0, 0.0, 0.0

def LSR(x, y, phi):
    u1, t1 = polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 ** 2
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)

        if t >= 0.0 and v >= 0.0:
            return 1, t, u, v    
    return 0, 0.0, 0.0, 0.0

def LRL(x, y, phi):
    u1, t1 = polar(x - math.sin(phi), y - 1.0 + math.cos(phi))

    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u + math.pi)
        v = mod2pi(phi - t + u)

        if t >= 0.0 and u <= 0.0:
            return 1, t, u, v     
    return 0, 0.0, 0.0, 0.0

def set_path(paths, lengths, ctypes):

    path = Path([],[],0.0,[],[],[],[])
    path.ctypes = ctypes
    path.lengths = lengths

    # check same path exist
    for tpath in paths:
        if tpath.ctypes == path.ctypes:
             if sum(tpath.lengths) - sum(path.lengths) <= 0.01:
                return paths  # not insert path
        
    path.L = sum([abs(i) for i in lengths])

    if path.L >= MAX_PATH_LENGTH:
        return paths # not insert path

    paths.append(path)

    return paths

def SCS(x, y, phi, paths):
    flag, t, u, v = SLS(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["S","L","S"])
    
    flag, t, u, v = SLS(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["S","R","S"])
    return paths

def SLS(x, y, phi):
    phi = mod2pi(phi)
    if y > 0.0 and phi > 0.0 and phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return 1, t, u, v
    elif y < 0.0 and phi > 0.0 and phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = -math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return 1, t, u, v

    return 0, 0.0, 0.0, 0.0

def CSC(x, y, phi, paths):
    flag, t, u, v = LSL(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["L","S","L"])
    
    flag, t, u, v = LSL(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L","S","L"])

    flag, t, u, v = LSL(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["R","S","R"])
    
    flag, t, u, v = LSL(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R","S","R"])
    
    flag, t, u, v = LSR(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["L","S","R"])
    
    flag, t, u, v = LSR(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L","S","R"])
    
    flag, t, u, v = LSR(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["R","S","L"])
    
    flag, t, u, v = LSR(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R","S","L"])

    return paths

def CCC(x, y, phi, paths):

    flag, t, u, v = LRL(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["L","R","L"])
    
    flag, t, u, v = LRL(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L","R","L"])
    
    flag, t, u, v = LRL(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, v], ["R","L","R"])
    
    flag, t, u, v = LRL(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R","L","R"])

    # backwards
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    flag, t, u, v = LRL(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["L","R","L"])
    
    flag, t, u, v = LRL(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["L","R","L"])
    
    flag, t, u, v = LRL(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["R","L","R"])
    
    flag, t, u, v = LRL(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["R","L","R"])
     
    return paths

def calc_tauOmega(u, v, xi, eta, phi):
    delta = mod2pi(u-v)
    A = np.sin(u) - np.sin(delta)
    B = np.cos(u) - np.cos(delta) - 1.0

    t1 = np.arctan2(eta*A - xi*B, xi*A + eta*B)
    t2 = 2.0 * (np.cos(delta) - np.cos(v) - np.cos(u)) + 3.0

    if t2 < 0:
        tau = mod2pi(t1+pi)
    else:
        tau = mod2pi(t1)
    
    omega = mod2pi(tau - u + v - phi)

    return tau, omega

def LRLRn(x, y, phi):
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho = 0.25 * (2.0 + np.sqrt(xi*xi + eta*eta))

    if rho <= 1.0:
        u = np.arccos(rho)
        t, v = calc_tauOmega(u, -u, xi, eta, phi)
        if t >= 0.0 and v <= 0.0:
            return 1, t, u, v
        
    return 0, 0.0, 0.0, 0.0



def LRLRp(x, y, phi):
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho = (20.0 - xi*xi - eta*eta) / 16.0

    if (rho>=0.0 and rho<=1.0):
        u = -np.arccos(rho)
        if (u >= -0.5 * np.pi):
            t, v = calc_tauOmega(u, u, xi, eta, phi)
            if t >= 0.0 and v >= 0.0:
                return 1, t, u, v      
    
    return 0, 0.0, 0.0, 0.0

def CCCC(x, y, phi, paths):

    flag, t, u, v = LRLRn(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, -u, v], ["L","R","L","R"])
    
    flag, t, u, v = LRLRn(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, -u, u, -v], ["L","R","L","R"])
     
    flag, t, u, v = LRLRn(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, -u, v], ["R","L","R","L"])
     
    flag, t, u, v = LRLRn(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, -u, u, -v], ["R","L","R","L"])
    
    flag, t, u, v = LRLRp(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, u, u, v], ["L","R","L","R"])
    
    flag, t, u, v = LRLRp(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -u, -v], ["L","R","L","R"])
    
    flag, t, u, v = LRLRp(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, u, u, v], ["R","L","R","L"])
    
    flag, t, u, v = LRLRp(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, -u, -u, -v], ["R","L","R","L"])
    
    return paths

def LRSR(x, y, phi):

    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho, theta = polar(-eta, xi)

    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = mod2pi(t + 0.5*np.pi - phi)
        if t >= 0.0 and u <= 0.0 and v <=0.0:
            return 1, t, u, v 
    
    return 0, 0.0, 0.0, 0.0

def LRSL(x, y, phi):
    xi = x - np.sin(phi)
    eta = y - 1.0 + np.cos(phi)
    rho, theta = polar(xi, eta)

    if rho >= 2.0:
        r = np.sqrt(rho*rho - 4.0)
        u = 2.0 - r
        t = mod2pi(theta + np.arctan2(r, -2.0))
        v = mod2pi(phi - 0.5*np.pi - t)
        if t >= 0.0 and u<=0.0 and v<=0.0:
            return 1, t, u, v      
    
    return 0, 0.0, 0.0, 0.0

def CCSC(x, y, phi, paths):

    flag, t, u, v = LRSL(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, v], ["L","R","S","L"])
    
    flag, t, u, v = LRSL(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, -v], ["L","R","S","L"])
    
    flag, t, u, v = LRSL(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, v], ["R","L","S","R"])
    
    flag, t, u, v = LRSL(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, -v], ["R","L","S","R"])
    
    flag, t, u, v = LRSR(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, v], ["L","R","S","R"])
    
    flag, t, u, v = LRSR(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, -v], ["L","R","S","R"])
    
    flag, t, u, v = LRSR(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, v], ["R","L","S","L"])
    
    flag, t, u, v = LRSR(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, -v], ["R","L","S","L"])
    
    # backwards
    xb = x*np.cos(phi) + y*np.sin(phi)
    yb = x*np.sin(phi) - y*np.cos(phi)
    flag, t, u, v = LRSL(xb, yb, phi) 
    if flag:
        paths = set_path(paths, [v, u, -0.5*np.pi, t], ["L","S","R","L"])
    
    flag, t, u, v = LRSL(-xb, yb, -phi) 
    if flag:
        paths = set_path(paths, [-v, -u, 0.5*np.pi, -t], ["L","S","R","L"])
    
    flag, t, u, v = LRSL(xb, -yb, -phi) 
    if flag:
        paths = set_path(paths, [v, u, -0.5*np.pi, t], ["R","S","L","R"])
    
    flag, t, u, v = LRSL(-xb, -yb, phi) 
    if flag:
        paths = set_path(paths, [-v, -u, 0.5*np.pi, -t], ["R","S","L","R"])
    
    flag, t, u, v = LRSR(xb, yb, phi) 
    if flag:
        paths = set_path(paths, [v, u, -0.5*np.pi, t], ["R","S","R","L"])
    
    flag, t, u, v = LRSR(-xb, yb, -phi) 
    if flag:
        paths = set_path(paths, [-v, -u, 0.5*np.pi, -t], ["R","S","R","L"])
    
    flag, t, u, v = LRSR(xb, -yb, -phi) 
    if flag:
        paths = set_path(paths, [v, u, -0.5*np.pi, t], ["L","S","L","R"])
    
    flag, t, u, v = LRSR(-xb, -yb, phi) 
    if flag:
        paths = set_path(paths, [-v, -u, 0.5*np.pi, -t], ["L","S","L","R"])
    
    return paths

def LRSLR(x, y, phi):
    # formula 8.11 *** TYPO IN PAPER ***
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho, theta = polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - np.sqrt(rho*rho - 4.0)
        if u <= 0.0:
            t = mod2pi(np.arctan2((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta))
            v = mod2pi(t - phi)

            if t >= 0.0 and v >=0.0:
                return 1, t, u, v

    return 0, 0.0, 0.0, 0.0

def CCSCC(x, y, phi, paths):
    flag, t, u, v = LRSLR(x, y, phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, -0.5*np.pi, v], ["L","R","S","L","R"])
    
    flag, t, u, v = LRSLR(-x, y, -phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, 0.5*np.pi, -v], ["L","R","S","L","R"])
    
    flag, t, u, v = LRSLR(x, -y, -phi) 
    if flag:
        paths = set_path(paths, [t, -0.5*np.pi, u, -0.5*np.pi, v], ["R","L","S","R","L"])
    
    flag, t, u, v = LRSLR(-x, -y, phi) 
    if flag:
        paths = set_path(paths, [-t, 0.5*np.pi, -u, 0.5*np.pi, -v], ["R","L","S","R","L"])
    
    return paths

def generate_local_course(L, lengths, mode, maxc, step_size):
    npoint = math.trunc(L / step_size) + len(lengths) + 4
    # print(npoint)
    # print(L)
    # print(len(lengths))

    px = [0.0 for i in range(npoint)]
    py = [0.0 for i in range(npoint)]
    pyaw = [0.0 for i in range(npoint)]
    directions = [0.0 for i in range(npoint)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1.0
    else:
        directions[0] = -1.0
    

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size
    
    pd = d
    ll = 0.0

    for (m, l, i) in zip(mode, lengths, range(len(mode))):

        if l > 0.0:
            d = step_size
        else:
            d = -step_size
        
        # set origin state
        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i-1]*lengths[i]) > 0:
            pd = - d - ll
        else:
            pd = d - ll
        
        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d
        
        ll = l - pd - d # calc remain length

        ind += 1
        px, py, pyaw, directions = interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
    
    #remove unused data
    while px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions

def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):

    if m == "S":
        px[ind] = ox + l / maxc * np.cos(oyaw)
        py[ind] = oy + l / maxc * np.sin(oyaw)
        pyaw[ind] =  oyaw
    else: # curve
        ldx = math.sin(l) / maxc
        if m == "L":  # left turn
            ldy = (1.0 - math.cos(l)) / maxc
        elif m == "R":  # right turn
            ldy = (1.0 - math.cos(l)) / -maxc
        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy
    
    if m == "L":  # left turn
        pyaw[ind] = oyaw + l
    elif m == "R":  # right turn
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1.0
    else:
        directions[ind] = -1.0
    
    return px, py, pyaw, directions

def generate_path(q0, q1, maxc):
    # use Reed-Shepp model to find a path between current node and goal node without obstacles.
    # Reed-Shepp model is non-holonomic-without-obstacles to find a path between two node and car can both forward and reverse.
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = math.cos(q0[2])
    s = math.sin(q0[2])
    x = (c*dx + s*dy)*maxc
    y = (-s*dx + c*dy)*maxc

    paths = []
    # different condition to build a path(S:straight, C:Curve)
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)
    paths = CCCC(x, y, dth, paths)
    paths = CCSC(x, y, dth, paths)
    paths = CCSCC(x, y, dth, paths)

    return paths
