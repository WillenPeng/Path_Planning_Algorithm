# some libraries and defs of trailier
import numpy as np
import math

from def_all import *

def check_collision(x, y, yaw, kdtree, ox, oy, wbd, wbr, vrx, vry):

    for (ix, iy, iyaw) in zip(x, y, yaw):
        cx = ix + wbd*math.cos(iyaw)
        cy = iy + wbd*math.sin(iyaw)

        # Whole bubble check
        ids = kdtree.query_ball_point([cx, cy], wbr)
        # println(length(ids))
        if len(ids) == 0:
            continue 

        if not rect_check(ix, iy, iyaw, ox[ids], oy[ids], vrx, vry):
            # print("collision")
            return 0 #collision
        
        # print(ids)    
    return 1 #OK

def rect_check(ix, iy, iyaw, ox, oy, vrx, vry):

    c = math.cos(-iyaw)
    s = math.sin(-iyaw)

    for (iox, ioy) in zip(ox, oy):
        tx = iox - ix
        ty = ioy - iy
        lx = (c*tx - s*ty)
        ly = (s*tx + c*ty)

        sumangle = 0.0
        for i in range(len(vrx)-1):
            x1 = vrx[i] - lx
            y1 = vry[i] - ly
            x2 = vrx[i+1] - lx
            y2 = vry[i+1] - ly
            d1 = np.hypot(x1,y1)
            d2 = np.hypot(x2,y2)
            theta1 = math.atan2(y1,x1)
            tty = (-math.sin(theta1)*x2 + math.cos(theta1)*y2)
            tmp = (x1*x2+y1*y2)/(d1*d2)

            if tmp >= 1.0:
                tmp = 1.0
            elif tmp <= 0.0:
                tmp = 0.0
            
            if tty >= 0.0:
                sumangle += math.acos(tmp)
            else:
                sumangle -= math.acos(tmp)
        
        if sumangle >= np.pi:
            return 0 #collision
    
    return 1 #OK

def calc_trailer_yaw_from_xyyaw(x, y, yaw, init_tyaw, steps):
    """
    calc trailer yaw from x y yaw lists
    """
    tyaw = np.zeros(len(x))
    tyaw[0] = init_tyaw

    for i in range(1, len(x)):
        tyaw[i] += tyaw[i-1] + steps[i-1]/LT*math.sin(yaw[i-1] - tyaw[i-1])

    return tyaw

def trailer_motion_model(x, y, yaw0, yaw1, D, d, L, delta):
    """
    Motion model for trailer 
    see:
    http://planning.cs.uiuc.edu/node661.html#77556
    """
    x += D*math.cos(yaw0)
    y += D*math.sin(yaw0)
    yaw0 += D/L*math.tan(delta)
    yaw1 += D/d*math.sin(yaw0 - yaw1)

    return x, y, yaw0, yaw1

def check_trailer_collision(ox, oy, x, y, yaw0, yaw1, kdtree = None):
    """
    collision check def for trailer

    """

    if kdtree == None:
        kdtree = KDTree(np.asarray([ox, oy]).T)
    
    vrxt = [LTF, LTF, -LTB, -LTB, LTF]
    vryt = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]

    # bubble parameter
    DT = (LTF + LTB)/2.0 - LTB
    DTR = (LTF + LTB)/2.0 + 0.3 

    # check trailer
    if not check_collision(x, y, yaw1, kdtree, ox, oy, DT, DTR, vrxt, vryt):
        return 0
    

    vrxf = [LF, LF, -LB, -LB, LF]
    vryf = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]
  
    # bubble parameter
    DF = (LF + LB)/2.0 - LB
    DFR = (LF + LB)/2.0 + 0.3 

    # check front trailer
    if not check_collision(x, y, yaw0, kdtree, ox, oy, DF, DFR, vrxf, vryf):
        return 0
    
    return 1 #OK
