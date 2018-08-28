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
    // double sx = 88.6;  // [m]
    // double sy = 145.8;  // [m]
    // double syaw0 = 90.0*D2R;
    // double syaw1 = 90.0*D2R;

    double sx = 153;  // [m]
    double sy = 20;  // [m]
    double syaw0 = 90.0*D2R;
    double syaw1 = 90.0*D2R;

    // // go straight and turn left 
    // double gx = 120.0;  // [m]
    // double gy = 199.0;  // [m]
    // double gyaw0 = 0.0*D2R;
    // double gyaw1 = 0.0*D2R;

    // // go back and park inside
    // double gx = 100.0;  // [m]
    // double gy = 135.0;  // [m]
    // double gyaw0 = 0.0*D2R;
    // double gyaw1 = 0.0*D2R;

    // // park back
    // double gx = 100.0;  // [m]
    // double gy = 133.0;  // [m]
    // double gyaw0 = 180.0*D2R;
    // double gyaw1 = 180.0*D2R;

    double gx = 88.0;  // [m]
    double gy = 82.0;  // [m]
    double gyaw0 = 90.0*D2R;
    double gyaw1 = 90.0*D2R;

    // double gx = 88.0;  // [m]
    // double gy = 195.0;  // [m]
    // double gyaw0 = 90.0*D2R;
    // double gyaw1 = 90.0*D2R;
    
    cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
    cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

    // build the map by own setting
    double min_x_m = min(sx, gx) - 10;//80;
    double min_y_m = min(sy, gy) - 10;//140;
    double max_x_m = max(sx, gx) + 10;//110;
    double max_y_m = max(sy, gy) + 10;//210;

    // double min_x_m = -25;//80;
    // double min_y_m = -17;//140;
    // double max_x_m = 412;//110;
    // double max_y_m = 350;//210;

    ifstream infile;
    infile.open("obstacle.dat");
    vector<double> ox_map;
    vector<double> oy_map;
    double x_val, y_val;
    while (infile >> x_val >> y_val)
    {
        if (x_val > min_x_m && x_val < max_x_m && y_val > min_y_m && y_val < max_y_m) {
            ox_map.emplace_back(x_val);
            oy_map.emplace_back(y_val);
        }
    }
    
    Map map(min_x_m, min_y_m, max_x_m, max_y_m, &ox_map, &oy_map);
    //find the final path
    Path_Final* path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, &map);
    cout << "Program Done!!" <<endl;

    // save data to a txt
    ofstream savefile;
    savefile.open ("simulation/path.dat");
    for (size_t i = 0; i < path->x.size(); i++) {
        savefile << path->x[i] << " " << path->y[i] << " " << path->yaw[i] << " " << path->yaw1[i] << " " << path->direction[i] << endl;
    }
    savefile.close();

    savefile.open ("simulation/startandgoal.dat");
    savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    savefile.close();

    savefile.open ("simulation/map.dat");
    for (size_t i = 0; i < ox_map.size(); i++) {
        savefile << ox_map[i] << " " << oy_map[i] << endl;
    }
    savefile.close();
    
    return 0;

}