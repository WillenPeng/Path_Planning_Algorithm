# Node class
class Node:
    def __init__(self, xind, yind, yawind, direction, x, y, yaw, yaw1, directions, steer, cost, pind):
        self.xind = xind  #x index
        self.yind = yind  #y index
        self.yawind = yawind  #yaw index
        self.direction = direction  # moving direction forword:1, backword:0
        self.x = x  # x position [m]
        self.y = y  # y position [m]
        self.yaw = yaw  # yaw angle [rad]
        self.yaw1 = yaw1  # trailer yaw angle [rad]
        self.directions = directions  # directions of each points forward: 1, backward:0
        self.steer = steer  # steer input
        self.cost = cost  # cost
        self.pind = pind  # parent index