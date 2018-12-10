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

    // double gx = 0.0;  // [m]
    // double gy = 0.0;  // [m]
    // double gyaw0 = 90.0*D2R;
    // double gyaw1 = 90.0*D2R;

    // double syaw0 = 0.0*D2R;
    // double syaw1 = 0.0*D2R;

    // // build the map by own setting
    // Map map;
    // map.build_own_map();


    // ofstream savefile;
    // savefile.open ("simulation/test_case.dat");

    // for (double sx = -20; sx <= 20; sx += 0.5) {
    //     for (double sy = 4; sy <= 10; sy += 0.5) {
    //         cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
    //         cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

    //         //find the final path
    //         bool havepath = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, &map);

    //         savefile << sx << " " << sy << " " << havepath << endl;

    //         cout << "Program Done!!" <<endl;
    //     }
    // }

    // savefile.close();



    double gx = 0.0;  // [m]
    double gy = 0.0;  // [m]
    double gyaw0 = 90.0*D2R;
    double gyaw1 = 90.0*D2R;

    double sy = 16.0;

    // build the map by own setting
    Map map;
    map.build_own_map();


    ofstream savefile;
    savefile.open ("simulation/test_case.dat");

    for (double sx = -20; sx <= 0; sx += 0.5) {
        for (double syaw0 = 0; syaw0 < 360; syaw0 += 15) {
            double syaw1 = syaw0;
            cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
            cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

            //find the final path
            bool havepath = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, &map);

            savefile << sx << " " << sy << " " << havepath << endl;

            cout << "Program Done!!" <<endl;
        }
    }

    savefile.close();


    // set start and goal configuration
    // double sx = 10.0;  // [m]
    // double sy = 12.0;  // [m]
    // double syaw0 = 0.0*D2R;
    // double syaw1 = 0.0*D2R;

    
    
    

    // save data to a dat
    // ofstream savefile;
    // savefile.open ("simulation/path.dat");
    // for (size_t i = 0; i < path.x.size(); i++) {
    //     savefile << path.x[i] << " " << path.y[i] << " " << path.yaw[i] << " " << path.yaw1[i] << " " << path.direction[i] << endl;
    // }
    // savefile.close();

    // savefile.open ("simulation/startandgoal.dat");
    // savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    // savefile.close();

    // savefile.open ("simulation/map.dat");
    // for (size_t i = 0; i < map.ox.size(); i++) {
    //     savefile << map.ox[i] << " " << map.oy[i] << endl;
    // }
    // savefile.close();
    
    return 0;

}