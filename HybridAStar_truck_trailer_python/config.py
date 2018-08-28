# config class for hybrid A* DB
import numpy as np
import math

from def_all import *

class Config:

    def calc_config(self, ox, oy, xyreso, yawreso):
        min_x_m = np.min(ox) - EXTEND_AREA
        min_y_m = np.min(oy) - EXTEND_AREA
        max_x_m = np.max(ox) + EXTEND_AREA
        max_y_m = np.max(oy) + EXTEND_AREA

        ox = np.append(ox, min_x_m)
        oy = np.append(oy, min_y_m)
        ox = np.append(ox, max_x_m)
        oy = np.append(oy, max_y_m)

        minx = int(round(min_x_m/xyreso))
        miny = int(round(min_y_m/xyreso))
        maxx = int(round(max_x_m/xyreso))
        maxy = int(round(max_y_m/xyreso))

        xw = int(round(maxx - minx))
        yw = int(round(maxy - miny))

        minyaw = int(round(-np.pi/yawreso)) - 1
        maxyaw = int(round(np.pi/yawreso))
        # yaww = int(round(maxyaw - minyaw))
        yaww = maxyaw - minyaw

        minyawt = minyaw
        maxyawt = maxyaw
        yawtw = yaww

        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.minyawt = minyawt
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.maxyawt = maxyawt
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.yawtw = yawtw
        self.xyreso = xyreso
        self.yawreso = yawreso

        return ox, oy
