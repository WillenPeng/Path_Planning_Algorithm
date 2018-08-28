# Include all global viarables
import numpy as np

D2R = np.pi/180.0
R2D =  180.0/np.pi

MOTION_RESOLUTION = 0.5 #[m] path interporate resolution

# Vehicle parameter
WB = 3.7  #[m] wheel base: rear to front steer
LT = 8.0 #[m] rear to trailer wheel
W = 2.6 #[m] width of vehicle
LF = 4.5 #[m] distance from rear to vehicle front end of vehicle
LB = 1.0 #[m] distance from rear to vehicle back end of vehicle
LTF = 1.0 #[m] distance from rear to vehicle front end of trailer
LTB = 9.0 #[m] distance from rear to vehicle back end of trailer
MAX_STEER = 0.6 #[rad] maximum steering angle 
TR = 0.5 # Tire radius [m] for plot
TW = 1.0 # Tire width [m] for plot

# for collision check
# WBUBBLE_DIST = 3.5 #distance from rear and the center of whole bubble
# WBUBBLE_R = 10.0 # whole bubble radius
# B = 4.45 # distance from rear to vehicle back end
# C = 11.54 # distance from rear to vehicle front end
# I = 8.55 # width of vehicle
# VRX = [C, C, -B, -B, C ]
# VRY = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]