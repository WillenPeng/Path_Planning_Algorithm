// Node class
#ifndef NODE_H
#define NODE_H

#include <vector>

using namespace std;

class Node;

class Node {
  public:
	Node() {};
	Node(int xind, int yind, int yawind, bool direction, vector<double> x, vector<double> y, vector<double> yaw, vector< double > yaw1, vector<bool> directions, double steer, double cost, int pind)
		: xind(xind)
		, yind(yind)
		, yawind(yawind)
		, direction(direction)
		, x(x)
		, y(y)
		, yaw(yaw)
		, yaw1(yaw1)
		, directions(directions)
		, steer(steer)
		, cost(cost)
		, pind(pind)
	{};

  public:
    int xind;  // x index
    int yind;  // y index
    int yawind;  // yaw index
    bool direction;  // moving direction forword:1, backword:0
    vector<double> x;  // x position [m]
    vector<double> y;  // y position [m]
    vector<double> yaw;  // yaw angle [rad]
    vector<double> yaw1;  // trailer yaw angle [rad]
    vector<bool> directions;  // directions of each points forward: 1, backward:0
    double steer;  // steer input
    double cost;  // cost
    int pind;  // parent index
};

#endif