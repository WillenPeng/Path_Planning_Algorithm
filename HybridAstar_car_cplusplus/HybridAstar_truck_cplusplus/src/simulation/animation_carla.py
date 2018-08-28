# simulation plt.plot
import matplotlib.pyplot as plt
import tralierlib
from def_all import *

import math
import numpy as np

class Path:
    def __init__(self, x, y, yaw, yaw1, direction):
        self.x = x # x position [m]
        self.y = y # y position [m]
        self.yaw = yaw # yaw angle [rad]
        self.yaw1 = yaw1 # trailer angle [rad]
        self.direction = direction # direction forward: true, back false

def show_animation(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1):
    # plt.plot(oox, ooy, ".k")
    # plot_trailer(sx, sy, syaw0, syaw1, 0.0)
    # plt.grid()
    # plt.axis('equal')
    # plt.show()
    # plot_trailer(gx, gy, gyaw0, gyaw1, 0.0)
    x = -path.x
    y = path.y
    yaw = np.pi - path.yaw
    yaw1 = np.pi - path.yaw1
    direction = path.direction

    steer = 0.0
    img_name = "result/"
    for ii in range(len(x)):
        plt.cla()
        plt.plot(-oox, ooy, ".k")
        plt.plot(x, y, "-r", label="Hybrid A* path")

        if ii < len(x)-1:
            k = (yaw[ii+1] - yaw[ii])/MOTION_RESOLUTION
            if not direction[ii]:
                k *= -1
            steer = math.atan2(WB*k, 1.0)
        else:
            steer = 0.0
        plot_trailer(x[ii], y[ii], yaw[ii], yaw1[ii], steer)
        plt.grid()
        plt.axis('equal')
        plt.pause(0.001)

        plt.savefig(img_name+str(ii)+".png")
    # plt.show()

def plot_trailer(x, y, yaw, yaw1, steer):

    truckcolor = "-k"

    LENGTH = LB+LF
    LENGTHt = LTB+LTF

    truckOutLine = np.asarray([[-LB, (LENGTH - LB), (LENGTH - LB), (-LB), (-LB)], [W/2, W/2, -W/2, -W/2, W/2]])
    trailerOutLine = np.asarray([[-LTB, (LENGTHt - LTB), (LENGTHt - LTB), (-LTB), (-LTB)], [W/2, W/2, -W/2, -W/2, W/2]])

    rr_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0+TW,  -W/12.0+TW, W/12.0+TW, W/12.0+TW, -W/12.0+TW]])
                
    rl_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0-TW,  -W/12.0-TW, W/12.0-TW, W/12.0-TW, -W/12.0-TW]])

    fr_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0+TW, -W/12.0+TW, W/12.0+TW, W/12.0+TW, -W/12.0+TW]])
                
    fl_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0-TW, -W/12.0-TW, W/12.0-TW, W/12.0-TW, -W/12.0-TW]])
    tr_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0+TW,  -W/12.0+TW, W/12.0+TW, W/12.0+TW, -W/12.0+TW]])
                
    tl_wheel = np.asarray([[TR, -TR, -TR, TR, TR], [-W/12.0-TW,  -W/12.0-TW, W/12.0-TW, W/12.0-TW, -W/12.0-TW]])
 
    Rot1 = np.asarray([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.asarray([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])
    Rot3 = np.asarray([[math.cos(yaw1), math.sin(yaw1)],[-math.sin(yaw1), math.cos(yaw1)]])

    fr_wheel = np.dot(fr_wheel.T, Rot2).T
    fl_wheel = np.dot(fl_wheel.T, Rot2).T
    fr_wheel[0,:] += WB
    fl_wheel[0,:] += WB
    fr_wheel = np.dot(fr_wheel.T, Rot1).T
    fl_wheel = np.dot(fl_wheel.T, Rot1).T

    tr_wheel[0,:] -= LT
    tl_wheel[0,:] -= LT
    tr_wheel = np.dot(tr_wheel.T, Rot3).T
    tl_wheel = np.dot(tl_wheel.T, Rot3).T

    truckOutLine = np.dot(truckOutLine.T, Rot1).T
    trailerOutLine = np.dot(trailerOutLine.T, Rot3).T
    rr_wheel = np.dot(rr_wheel.T, Rot1).T
    rl_wheel = np.dot(rl_wheel.T, Rot1).T

    truckOutLine[0, :] += x
    truckOutLine[1, :] += y
    trailerOutLine[0, :] += x
    trailerOutLine[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    tr_wheel[0, :] += x
    tr_wheel[1, :] += y
    tl_wheel[0, :] += x
    tl_wheel[1, :] += y

    plt.plot(truckOutLine[0, :], truckOutLine[1, :], truckcolor)
    # plt.plot(trailerOutLine[0, :], trailerOutLine[1, :], truckcolor)
    plt.plot(fr_wheel[0, :], fr_wheel[1, :], truckcolor)
    plt.plot(rr_wheel[0, :], rr_wheel[1, :], truckcolor)
    plt.plot(fl_wheel[0, :], fl_wheel[1, :], truckcolor)
    plt.plot(rl_wheel[0, :], rl_wheel[1, :], truckcolor)

    # plt.plot(tr_wheel[0, :], tr_wheel[1, :], truckcolor)
    # plt.plot(tl_wheel[0, :], tl_wheel[1, :], truckcolor)
    plt.plot(x, y, "*")

def main():
    x, y, yaw, yaw1, direction = np.loadtxt('path.dat', delimiter=' ', unpack=True)
    path = Path(x, y, yaw, yaw1, direction)
    oox, ooy = np.loadtxt('map.dat', delimiter=' ', unpack=True)
    sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1 = np.loadtxt('startandgoal.dat', delimiter=' ', unpack=True)
    show_animation(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)


if __name__ == "__main__":
    main()
