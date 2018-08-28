// Basic Astar algorithm (will be updated some functions)
#ifndef GRID_A_STAR_H
#define GRID_A_STAR_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <utility>
#include <limits>
#include <iostream>
#include <math.h>
#include "kdtree.h"

using namespace std;

class node_s;
class grid_a_star;

vector<vector<double> > get_motion_model();

struct CompareByFirst_node {
    constexpr bool operator()(pair<double, int> const & a,
                              pair<double, int> const & b) const noexcept
    { return a.first > b.first; }
};

// Node class
class node_s {
  public:
	node_s() {};
    node_s(int xind, int yind, double cost, int pind) 
      : x(xind)
      , y(yind)
      , cost(cost)
      , pind(pind)
    {}

  public:
    int x;  // x index
    int y;  // y index
    double cost;  // cost
    int pind;  // parent index
};

class grid_a_star {
  private:
    int minx; 
    int miny;  
    int maxx; 
    int maxy;  
    int xw; 
    int yw;
    vector<vector<bool> > obmap;

  public:
    vector <vector< double > > calc_dist_policy(double gx, double gy, vector<double>* oox, vector<double>* ooy, double reso, double vr){
        /*
        To get a map with the h cost (distance to goal node) of each grid (without obstacle)
        gx: goal x position [m]
        gy: goal y position [m]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        vr: vehicle radius[m]
        */
		node_s ngoal(int(round(gx / reso)), int(round(gy / reso)), 0.0, -1);

        vector<double> ox, oy;
        ox.reserve(oox->size());
        oy.reserve(ooy->size());
        for (size_t i = 0; i < oox->size(); i++) {
			ox.emplace_back((*oox)[i] / reso);
			oy.emplace_back((*ooy)[i] / reso);
        }

        calc_obstacle_map(&ox, &oy, reso, vr);

        unordered_map<int, node_s> open_set;
        unordered_map<int, node_s> closed_set;

        open_set[calc_index(&ngoal)] = ngoal;

        vector<vector<double> > motion = get_motion_model();
        size_t nmotion = motion.size();
        priority_queue<pair<double, int>,
                   std::vector<pair<double, int> >,
                   CompareByFirst_node> pq;
        pq.push(make_pair(ngoal.cost, calc_index(&ngoal)));

        int c_id;
        node_s current;

        while (1){
            if (open_set.empty()) {
                break;
            }
            c_id = pq.top().second;
            pq.pop();
            current = open_set[c_id];
            open_set.erase(c_id);  
            closed_set[c_id] = current;

            for (size_t i = 0; i < nmotion; i++){ // expand search grid based on motion model
                node_s node(current.x+int(motion[i][0]), current.y + int(motion[i][1]), current.cost + motion[i][2], c_id);
                if (!verify_node(&node)){
                    continue;
                }
                
                int node_ind = calc_index(&node);
                // If it is already in the closed set, skip it
                if (closed_set.find(node_ind) != closed_set.end()) continue;

                if (open_set.find(node_ind) != closed_set.end()) {
                    if (open_set[node_ind].cost > node.cost){
                        // If so, update the node to have a new parent
                        open_set[node_ind].cost = node.cost;
                        open_set[node_ind].pind = c_id;
                    }
                }
                else { // add to open_set set
                    open_set[node_ind] = node;
                    pq.push(make_pair(ngoal.cost, calc_index(&node)));
                }
            }            
        }
        vector <vector< double > > pmap = calc_policy_map(&closed_set);
		return pmap;
    }

    vector <vector< double > > calc_policy_map(unordered_map<int, node_s>* closed) {

        vector<vector<double> > pmap(xw, vector<double>(yw, numeric_limits<double>::infinity()));
        for (auto it = closed->begin(); it != closed->end(); ++it){
            node_s n = it->second;
            pmap[n.x-minx-1][n.y-miny-1] = n.cost;
        }
        return pmap;
    }


    void calc_obstacle_map(vector< double >* ox, vector< double >* oy, double reso, double vr) {
    // build an obstacle map and in each grid, it shows whether it has obstacle

        auto ox_min_max = minmax_element(ox->begin(), ox->end());
        auto oy_min_max = minmax_element(oy->begin(), oy->end());
        minx = int(round(*(ox_min_max.first)));
        miny = int(round(*(oy_min_max.first)));
        maxx = int(round(*(ox_min_max.second)));
        maxy = int(round(*(oy_min_max.second)));

        xw = maxx - minx;
        yw = maxy - miny;

        vector<vector<bool> > obmap_init(xw, vector<bool>(yw, false));
        obmap = obmap_init;

        vector<vector<double> > samples;
        size_t N = ox->size();
        samples.resize(N);
        for (size_t i = 0; i < N; i++) {
            samples[i].emplace_back((*ox)[i]);
            samples[i].emplace_back((*oy)[i]);
        }

        kdt::KDTree<vector<double>> kdtree(samples);

        double x, y;
        int knnIndices;
        for (int ix = 0; ix < xw; ix++) {
            x = ix + minx + 1;
            for (int iy = 0; iy < yw; iy++) {
                y = iy + miny + 1;
                knnIndices = kdtree.nnSearch({x, y});
                double dis = sqrt(pow(x - samples[knnIndices][0], 2)+pow(y - samples[knnIndices][1], 2));
                if (dis <= vr/reso) {
                    obmap[ix][iy] = true;
                }
            }
        }
    }

    int calc_index(node_s* node) {
        return (node->y - miny)*xw + (node->x - minx);
    }

    // double calc_cost(node_s* n, node_s* ngoal) {
    //     return (n->cost + h(n->x - ngoal->x, n->y - ngoal->y));
    // }


    // double h(double x, double y) {
        
    //     Heuristic cost function
        
    //     return sqrt(pow(x,2) + pow(y,2));
    // }

    bool verify_node(node_s* node) {
        if ((node->x - minx) >= xw) {
            return false;
        }
        else if ((node->x - minx) <= 0) {
            return false;
        }
        else if ((node->y - miny) >= yw) {
            return false;
        }
        else if ((node->y - miny) <= 0) {
            return false;
        }
        // collision check
        if (obmap[node->x-minx-1][node->y-miny-1]) {
            return false;
        }
        return true;
    }  
};

vector<vector<double> > get_motion_model() {
    // dx, dy, cost
    vector<vector<double> > motion({{1,0,1}, 
                                    {0,1,1},
                                    {-1,0,1},
                                    {0,-1,1},
                                    {-1,-1,sqrt(2)},
                                    {-1,1,sqrt(2)},
                                    {1,-1,sqrt(2)},
                                    {1,1,sqrt(2)}});
    return motion;
}
    

#endif