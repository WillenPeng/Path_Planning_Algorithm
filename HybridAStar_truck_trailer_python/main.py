# main function to do auto-perpendicular parking
# Writen by Weilun Peng
# 06/19/2018

import pathfinder_hybrid_astar
from def_all import *


def main():
    print("Program Start!!")

    # sx = 14.0  # [m]
    # sy = 10.0  # [m]
    # syaw0 = 0.0*D2R
    # syaw1 = 0.0*D2R

    sx = 0.0  # [m]
    sy = 10.0  # [m]
    syaw0 = 0.0*D2R
    syaw1 = 0.0*D2R

    gx = 0.0  # [m]
    gy = 0.0  # [m]
    gyaw0 = 90.0*D2R
    gyaw1 = 90.0*D2R

    ox = []
    oy = []
    for i in range(-25, 26):
        ox.append(i)
        oy.append(15.0)

    for i in range(-25, -3):
        ox.append(i)
        oy.append(4.0)

    for i in range(-15, 5):
        ox.append(-4.0)
        oy.append(i)

    for i in range(-15, 5):
        ox.append(4.0)
        oy.append(i)
    
    for i in range(4, 26):
        ox.append(i)
        oy.append(4.0)
    
    for i in range(-4, 5):
        ox.append(i)
        oy.append(-15.0)

    path = pathfinder_hybrid_astar.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    print (path)
    print ("Program Done!!")


if __name__ == "__main__":
    main()
