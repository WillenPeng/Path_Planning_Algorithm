/* Copyright 2018 Weilun Peng */
//map class with all information about obstacles
#ifndef HYBRIDASTAR_H
#define HYBRIDASTAR_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <math.h>
#include <time.h>

#include "node.hpp"
#include "map.hpp"
#include "def_all.hpp"
#include "trailerlib.hpp"
#include "rs_path.hpp"
#include "grid_a_star.hpp"

using namespace std;

class Path_Final;
Path_Final* calc_hybrid_astar_path(double sx, double sy, double syaw, double syaw1, double gx, double gy, double gyaw, double gyaw1, Map map);
bool is_same_grid(Node* node1, Node* node2);
Path_Final* get_final_path(unordered_map<int, Node>* closed, Node* ngoal, Node* nstart, Map* map);
bool verify_index(Node* node, Map* map, double inityaw1);
Node calc_next_node(Node* current, int c_id, double u, double d, Map* map);
vector<Node> update_node_with_analystic_expantion(Node* current, Node* ngoal, Map* map, double gyaw1);
vector<Path*> analystic_expantion(Node* n, Node* ngoal, Map* map);
pair<vector< double >, vector< double >> calc_motion_inputs();

struct CompareByFirst_path {
    constexpr bool operator()(pair<double, Path*> const & a,
                              pair<double, Path*> const & b) const noexcept
    { return a.first > b.first; }
};

// Path class
class Path_Final {
  public:
	Path_Final() {};
	Path_Final(vector<double> x, vector<double> y, vector<double> yaw, vector<double> yaw1, vector<double> steer, vector<bool> direction, double cost) 
      : x(x)
      , y(y)
      , yaw(yaw)
      , yaw1(yaw1)
      , steer(steer)
      , direction(direction)
      , cost(cost)
    {}
  public:
    vector<double> x; // x position [m]
    vector<double> y; // y position [m]
    vector<double> yaw; // yaw angle of front [rad]
    vector<double> yaw1; // trailer angle [rad]
    vector<double> steer; // steering angle of the wheel [rad]
    vector<bool> direction; // direction forward: true, back false
    double cost; // cost
};


Path_Final* calc_hybrid_astar_path(double sx, double sy, double syaw, double syaw1, double gx, double gy, double gyaw, double gyaw1, Map* map) {
    /*
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    */
    cout << "Start Find Path!" << endl;
    syaw = pi_2_pi(syaw);
    gyaw = pi_2_pi(gyaw);

    Node nstart(int(round(sx/XY_GRID_RESOLUTION)), int(round(sy/XY_GRID_RESOLUTION)), int(round(syaw/YAW_GRID_RESOLUTION)), true, {sx}, {sy}, {syaw}, {syaw1}, {true}, 0.0, 0.0, -1);
    Node ngoal(int(round(gx/XY_GRID_RESOLUTION)), int(round(gy/XY_GRID_RESOLUTION)), int(round(gyaw/YAW_GRID_RESOLUTION)), true, {gx}, {gy}, {gyaw}, {gyaw1}, {true}, 0.0, 0.0, -1);

    // To get a map with the h cost (distance to goal node) of each grid (without obstacle)
    map->calc_holonomic_with_obstacle_heuristic(&ngoal);
    // initilize the open set, closed set, priority queue
    unordered_map<int, Node> open_set;
    unordered_map<int, Node> closed_set;
    // fnode = None
    open_set[map->calc_index(&nstart)] = nstart;

    priority_queue<pair<double, int>,
               std::vector<pair<double, int> >,
               CompareByFirst_node> pq;
    pq.push(make_pair(map->calc_cost(&nstart), map->calc_index(&nstart)));

    // get motion primitives
    pair<vector< double >, vector< double >> u_and_d = calc_motion_inputs();
    vector< double > u = u_and_d.first, d = u_and_d.second;
    size_t nmotion = u.size();

    int c_id;
    Node current, fnode;
    vector<Node> finalnode;
    // main iteration
    time_t startime = time(NULL);
    while (1) {
        if (open_set.empty()) {
            cout << "Error: Cannot find path, No open set" << endl;
            exit(1);
        }
        c_id = pq.top().second;
        pq.pop();
        current = open_set[c_id];
        // cout << "index: " << current.xind << " " << current.yind << endl;
        // move current node from open to closed
        open_set.erase(c_id);  
        closed_set[c_id] = current;

        // use Reed-Shepp model to find a path between current node and goal node without obstacles, which will not run every time for computational reasons.
        if (abs(current.x.back()-ngoal.x.back()) < XY_GRID_RESOLUTION/2 && abs(current.y.back()-ngoal.y.back()) < XY_GRID_RESOLUTION/2 && abs(current.yaw.back()-ngoal.yaw.back()) < GOAL_TYAW_TH) {
            cout << "Goal Find!" << endl;
            fnode = current;
            break;
        }

        if (abs(current.x.back()-ngoal.x.back()) > 10*XY_GRID_RESOLUTION && abs(current.y.back()-ngoal.y.back()) > 10*XY_GRID_RESOLUTION) {
            cout << "Start Use Reed-Shepp Model!" << endl;
            cout << "currentxy: " << current.x.back() << " " << current.y.back() << " " << current.yaw.back() << endl;
            finalnode = update_node_with_analystic_expantion(&current, &ngoal, map, gyaw1);
            cout << "finalnode: " << finalnode.size() << endl;
            if (!finalnode.empty()) { // found
                fnode = finalnode[0];
                break;
            }
        } 
        
            
         
        
        double inityaw1 = current.yaw1[0];
        Node node;
        int node_ind;
        for (size_t i = 0; i < nmotion; i++) {
            // For each node, it will have multiple possible points but with one x,y index
            node = calc_next_node(&current, c_id, u[i], d[i], map);

            if (!verify_index(&node, map, inityaw1)) 
                continue;

            node_ind = map->calc_index(&node);

            // If it is already in the closed set, skip it
            if (closed_set.find(node_ind) != closed_set.end()) 
                continue;

            if (open_set.find(node_ind) == open_set.end()) {
                open_set[node_ind] = node;
                pq.push(make_pair(map->calc_cost(&node), node_ind));
            }
            else{
                if (open_set[node_ind].cost > node.cost)
                    // If so, update the node to have a new parent
                    open_set[node_ind] = node;
            }
        }
    }

    time_t finaltime = time(NULL);
    cout << "Calculated Time: " << finaltime -startime << "[s]" << endl;

    return get_final_path(&closed_set, &fnode, &nstart, map);

}


vector<Node> update_node_with_analystic_expantion(Node* current, Node* ngoal, Map* map, double gyaw1) {
    // use Reed-Shepp model to find a path between current node and goal node without obstacles, which will not run every time for computational reasons.
    vector<Path*> finalpath = analystic_expantion(current, ngoal, map);
    vector<Node> finalnode;
    if (!finalpath.empty()) {
        Path* apath = finalpath[0];
        vector<double> fx((apath->x).begin()+1, (apath->x).end()); 
        vector<double> fy((apath->y).begin()+1, (apath->y).end()); 
        vector<double> fyaw((apath->yaw).begin()+1, (apath->yaw).end());

        // if (abs(pi_2_pi(apath->yaw1.back() - gyaw1)) >= GOAL_TYAW_TH)
        //     return finalnode; //no update


        double fcost = current->cost + apath->pathcost;
        vector<double> fyaw1(apath->yaw1.begin()+1, apath->yaw1.end());
        int fpind = map->calc_index(current);

        vector<bool> fd;
        fd.reserve(apath->directions.size());
        double d;
        for (size_t i = 1; i < apath->directions.size(); i++) {
            d = apath->directions[i];
            if (d >= 0)
                fd.emplace_back(1);
            else
                fd.emplace_back(0);
        }

        double fsteer = 0.0;

        Node fpath(current->xind, current->yind, current->yawind, current->direction, fx, fy, fyaw, fyaw1, fd, fsteer, fcost, fpind);
        delete apath;
        finalnode.emplace_back(fpath);
        return finalnode;
    }
    return finalnode; // no update
}

vector<Path*> analystic_expantion(Node* n, Node* ngoal, Map* map) {
    
    vector<Path*> finalpath;
    double sx = n->x.back();
    double sy = n->y.back();
    double syaw = n->yaw.back();

    double max_curvature = tan(REED_STEER_CONST)/WB;
    vector<Path*> paths = calc_paths(sx, sy, syaw, ngoal->x.back(), ngoal->y.back(), ngoal->yaw.back(), max_curvature,MOTION_RESOLUTION);
    if (paths.empty())
        return finalpath;

    priority_queue<pair<double, Path*>,
                   std::vector<pair<double, Path*> >,
                   CompareByFirst_path> pathqueue;
    vector<double> steps;
    for (size_t i = 0; i < paths.size(); i++) {
        steps.resize(paths[i]->directions.size());
        for (size_t j = 0; j < paths[i]->directions.size(); j++) {
            steps[j] = MOTION_RESOLUTION*paths[i]->directions[j];
        }
        paths[i]->yaw1 = calc_trailer_yaw_from_xyyaw(&(paths[i]->yaw), n->yaw1.back(), &steps);
        pathqueue.push(make_pair(paths[i]->calc_rs_path_cost(), paths[i]));
    }
    Path* path;
    vector<double> newpathx, newpathy, newpathyaw, newpathyaw1;
    while (!pathqueue.empty()) {
        path = pathqueue.top().second;
        pathqueue.pop();
        newpathx.clear();
        newpathy.clear();
        newpathyaw.clear();
        newpathyaw1.clear();
        for (size_t i = 0; i < path->x.size(); i+= SKIP_COLLISION_CHECK) {
            newpathx.emplace_back(path->x[i]);
            newpathy.emplace_back(path->y[i]);
            newpathyaw.emplace_back(path->yaw[i]);
            newpathyaw1.emplace_back(path->yaw1[i]);
        }
        if (check_trailer_collision(map, &newpathx, &newpathy, &newpathyaw, &newpathyaw1)) {
            finalpath.emplace_back(path);
            return finalpath; // path is ok
        }
    }
    return finalpath;
}


bool is_same_grid(Node* node1, Node* node2) {
	if (node1->xind != node2->xind || node1->yind != node2->yind || node1->yawind != node2->yawind) {
		return false;
	}
    return true;
}

Path_Final* get_final_path(unordered_map<int, Node>* closed, Node* ngoal, Node* nstart, Map* map) {
    // ngoal is the result from Analytic Expansions and the final path is the combination of points in ngoal and point in closed set
    vector< double > rx, ry, ryaw, ryaw1;
    vector< bool > direction;
    for (int i = ngoal->x.size() - 1; i >= 0; i--) {
        rx.emplace_back(ngoal->x[i]);
        ry.emplace_back(ngoal->y[i]);
        ryaw.emplace_back(ngoal->yaw[i]);
        ryaw1.emplace_back(ngoal->yaw1[i]);
        direction.emplace_back(ngoal->directions[i]);
    }
    int nid = ngoal->pind;
    double finalcost = ngoal->cost;

    Node n;
    while (1) {
        n = (*closed)[nid];
        for (int i = n.x.size() - 1; i >= 0; i--) {
            rx.emplace_back(n.x[i]);
            ry.emplace_back(n.y[i]);
            ryaw.emplace_back(n.yaw[i]);
            ryaw1.emplace_back(n.yaw1[i]);
            direction.emplace_back(n.directions[i]);
        }
        nid = n.pind;
        if (is_same_grid(&n, nstart)) {
            break;
        }
    }

    vector< double > rx_new, ry_new, ryaw_new, ryaw1_new;
    vector< bool > direction_new;
    for (int i = rx.size() - 1; i >= 0; i--) {
        rx_new.emplace_back(rx[i]);
        ry_new.emplace_back(ry[i]);
        ryaw_new.emplace_back(ryaw[i]);
        ryaw1_new.emplace_back(ryaw1[i]);
        direction_new.emplace_back(direction[i]);
    }

    // adjuct first direction
    direction_new[0] = direction_new[1];
    double tem_len;
    vector<double> steer;
    steer.reserve(rx_new.size());
    for (size_t i = 0; i < rx_new.size(); i++) {
        if (i < rx_new.size() - 1) {
            tem_len = (ryaw_new[i+1] - ryaw_new[i])/MOTION_RESOLUTION;
            if (!direction[i]) tem_len *= -1;
            steer.emplace_back(atan2(WB*tem_len, 1.0));
        }
        else 
            steer.emplace_back(0.0);
    }
    cout << "x: " << endl;
    for (auto each_x: rx_new) {
        cout << each_x << " ";
    }
    cout << endl;
    cout << "y: " << endl;
    for (auto each_y: ry_new) {
        cout << each_y << " ";
    }
    cout << endl;
    cout << "yaw: " << endl;
    for (auto each_yaw: ryaw_new) {
        cout << each_yaw << " ";
    }
    cout << endl;
    cout << "direction_new: " << endl;
    for (auto each_direction_new: direction_new) {
        cout << each_direction_new << " ";
    }
    cout << endl;
    Path_Final* path = new Path_Final(rx_new, ry_new, ryaw_new, ryaw1_new, steer, direction_new, finalcost);
	return path;
}

bool verify_index(Node* node, Map* map, double inityaw1) {

    // overflow map
    if ((node->xind - map->minx) >= map->xw)
        return false;
    else if ((node->xind - map->minx) <= 0)
        return false;

    if ((node->yind - map->miny) >= map->yw)
        return false;
    else if ((node->yind - map->miny) <= 0)
        return false;

    // check collisiton
    vector<double> steps;
    for (auto each_direction: node->directions) {
        steps.emplace_back(MOTION_RESOLUTION*each_direction);
    }
    vector<double> yaw1 = calc_trailer_yaw_from_xyyaw(&(node->yaw), inityaw1, &steps);
    vector<double> newnodex, newnodey, newnodeyaw, newnodeyaw1;
    for (size_t i = 0; i < node->x.size(); i+= 1) {
        newnodex.emplace_back(node->x[i]);
        newnodey.emplace_back(node->y[i]);
        newnodeyaw.emplace_back(node->yaw[i]);
        newnodeyaw1.emplace_back(yaw1[i]);
    }
    if (!check_trailer_collision(map, &newnodex, &newnodey, &newnodeyaw, &newnodeyaw1))
        return false;

    return true; //index is ok"
}

    

Node calc_next_node(Node* current, int c_id, double u, double d, Map* map) {
    
    double arc_l = XY_GRID_RESOLUTION*1.5;

    int nlist = int(floor(arc_l/MOTION_RESOLUTION)) + 1;
    vector<double> xlist(nlist, 0.0);
    vector<double> ylist(nlist, 0.0);
    vector<double> yawlist(nlist, 0.0);
    vector<double> yaw1list(nlist, 0.0);

    xlist[0] = current->x.back() + d * MOTION_RESOLUTION*cos(current->yaw.back());
    ylist[0] = current->y.back() + d * MOTION_RESOLUTION*sin(current->yaw.back());
    yawlist[0] = pi_2_pi(current->yaw.back() + d*MOTION_RESOLUTION/WB * tan(u));
    yaw1list[0] = pi_2_pi(current->yaw1.back() + d*MOTION_RESOLUTION/LT*sin(current->yaw.back()-current->yaw1.back()));
    for (size_t i = 0; i < nlist-1; i++) {
        xlist[i+1] = xlist[i] + d * MOTION_RESOLUTION*cos(yawlist[i]);
        ylist[i+1] = ylist[i] + d * MOTION_RESOLUTION*sin(yawlist[i]);
        yawlist[i+1] = pi_2_pi(yawlist[i] + d*MOTION_RESOLUTION/WB * tan(u));
        yaw1list[i+1] = pi_2_pi(yaw1list[i] + d*MOTION_RESOLUTION/LT*sin(yawlist[i]-yaw1list[i]));
    }

    int xind = int(round(xlist.back()/XY_GRID_RESOLUTION));
    int yind = int(round(ylist.back()/XY_GRID_RESOLUTION));
    int yawind = int(round(yawlist.back()/YAW_GRID_RESOLUTION));


    double addedcost = 0.0;
    bool direction;
    if (d > 0) {
        direction = true;
        addedcost += abs(arc_l);
    }
    else {
        direction = false;
        addedcost += abs(arc_l) * BACK_COST;
    }
    // cout << "lengthcost: " << addedcost << endl;

    // switch back penalty
    if (direction != current->direction) // switch back penalty
        addedcost += SB_COST;
    // cout << "SB_COST: " << addedcost << endl;
    // steer penalty
    addedcost += STEER_COST*abs(u);
    // cout << "STEER_COST: " << addedcost << endl;
    // steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current->steer - u);
    // cout << "STEER_CHANGE_COST: " << addedcost << endl;
    // jacknif cost
    double jack_cost = 0.0;
    for (size_t i = 0; i < yawlist.size(); i++) {
        jack_cost += abs(pi_2_pi(yawlist[i] - yaw1list[i]));
    }
    addedcost += JACKKNIF_COST * jack_cost;

    double cost = current->cost + addedcost;

    vector<bool> directions(xlist.size(), direction);

    Node node(xind, yind, yawind, direction, xlist, ylist, yawlist, yaw1list, directions, u, cost, c_id);

    return node;
}


pair<vector< double >, vector< double >> calc_motion_inputs() {
    // get the motion around current node with length 1 and different heading degree
    vector< double > up, u;
    for (double i = MAX_STEER/N_STEER; i <= MAX_STEER; i+= MAX_STEER/N_STEER) {
        up.emplace_back(i);
    }
    u.emplace_back(0.0);
    u.insert(u.end(), up.begin(), up.end());
    for (vector<double>::iterator it = up.begin() ; it != up.end(); ++it) u.emplace_back(-(*it));
    vector<double> d(u.size(), 1.0);
    vector<double> uminus1(u.size(), -1.0);
    d.insert(d.end(), uminus1.begin(), uminus1.end());
    u.insert(u.end(), u.begin(), u.end());

    return make_pair(u, d);
}
    
#endif