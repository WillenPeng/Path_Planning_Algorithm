/* Copyright 2018 Weilun Peng */
// main function

#include <iostream>
#include <fstream>
#include <vector>

#include "map.hpp"
#include "pathfinder_hybrid_astar.hpp"
#include "def_all.hpp"

using namespace std;

int main(int argc, char **argv) {
    // set start and goal configuration
    double sx = 10.0;  // [m]
    double sy = 10.0;  // [m]
    double syaw0 = 0.0*D2R;
    double syaw1 = 0.0*D2R;

    double gx = 0.0;  // [m]
    double gy = 0.0;  // [m]
    double gyaw0 = 90.0*D2R;
    double gyaw1 = 90.0*D2R;
    
    cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
    cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

    // build the map by own setting
    Map map;
    map.build_own_map();
    //find the final path
    Path_Final path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, map);

    cout << "Program Done!!" <<endl;

    // save data to a dat
    ofstream savefile;
    savefile.open ("simulation/path.dat");
    for (size_t i = 0; i < path.x.size(); i++) {
        savefile << path.x[i] << " " << path.y[i] << " " << path.yaw[i] << " " << path.yaw1[i] << " " << path.direction[i] << endl;
    }
    savefile.close();

    savefile.open ("simulation/startandgoal.dat");
    savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    savefile.close();

    savefile.open ("simulation/map.dat");
    for (size_t i = 0; i < map.ox.size(); i++) {
        savefile << map.ox[i] << " " << map.oy[i] << endl;
    }
    savefile.close();
    
    return 0;

}