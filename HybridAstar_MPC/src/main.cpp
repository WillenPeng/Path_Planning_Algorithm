/* Copyright 2018 Weilun Peng */
// main function

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include "../HybridAstar_truck_cplusplus/src/map.hpp"
#include "../HybridAstar_truck_cplusplus/src/pathfinder_hybrid_astar.hpp"
#include "../HybridAstar_truck_cplusplus/src/def_all.hpp"

#include "../MPC/mpc_controller.hpp"

//#include "../MPC/pid_controller.hpp"

using namespace std;

int main(int argc, char **argv) {
    // set start and goal configuration
    // //Case 1 (Worked):
    // double sx = 0.0;  // [m]
    // double sy = 10.0;  // [m]
    // double syaw0 = 90.0*D2R;
    // double syaw1 = 90.0*D2R;

    // //Case 6 (Worked, Front Parking):
    // double sx = -5.0;  // [m]
    // double sy = 15.0;  // [m]
    // double syaw0 = -45.0*D2R;
    // double syaw1 = -120.0*D2R;

    // // Case 7 (Worked):
    // double sx = 1.0;  // [m]
    // double sy = 15.0;  // [m]
    // double syaw0 = 120.0*D2R;
    // double syaw1 = 90.0*D2R;

    // //Case 2 (Worked):
    // double sx = 10.0;  // [m]
    // double sy = 10.0;  // [m]
    // double syaw0 = 0.0*D2R;
    // double syaw1 = 0.0*D2R;

    // //Case 3 (Worked):
    // double sx = 10.0;  // [m]
    // double sy = 15.0;  // [m]
    // double syaw0 = 0.0*D2R;
    // double syaw1 = 0.0*D2R;

    // //Case 8 (hard, worked):
    // double sx = 10.0;  // [m]
    // double sy = 20.0;  // [m]
    // double syaw0 = 45.0*D2R;
    // double syaw1 = 45.0*D2R;

    // //Case 4 (Very hard, worked):
    // double sx = 5.0;  // [m]
    // double sy = 15.0;  // [m]
    // double syaw0 = 45.0*D2R;
    // double syaw1 = 90.0*D2R;

    //Case 5 (hard, worked):
    double sx = 20.0;  // [m]
    double sy = 15.0;  // [m]
    double syaw0 = 0.0*D2R;
    double syaw1 = 0.0*D2R;

    // double gx = 0.0;  // [m]
    // double gy = 0.0;  // [m]
    // double gyaw0 = 90.0*D2R;
    // double gyaw1 = 90.0*D2R;

    // Reverse Parking
    double gx = 0.0;  // [m]
    double gy = 0.0;  // [m]
    double gyaw0 = 90.0*D2R;
    double gyaw1 = 90.0*D2R;

    // // Front Parking
    // double gx = 0.0;  // [m]
    // double gy = 0.0;  // [m]
    // double gyaw0 = -90.0*D2R;
    // double gyaw1 = -90.0*D2R;
    
    cout << "Start Configuration: (" << sx << ", " << sy << ", " << syaw0*R2D << ", " << syaw1*R2D << ")" << endl;
    cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;

    // build the map by own setting
    Map map;
    map.build_own_map();
    //find the final path
    Path_Final plan_path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, map);

    cout << "Path Planning Done!!" <<endl;

    // double delta_x=0;
    // double delta_y=0;

    // for(int i=0;i<plan_path.x.size()-1;++i){
    //     delta_x = abs(plan_path.x[i+1]-plan_path.x[i]);
    //     delta_y = abs(plan_path.y[i+1]-plan_path.y[i]);
    //     cout<<"Current Distance is "<<sqrt(delta_x*delta_x+delta_y*delta_y)<<endl;
    // }

    // cout<<"END!!"<<endl;

    vector<vector<double>> realpath = real_path(plan_path.x, plan_path.y, plan_path.yaw, plan_path.direction);

    // // for (auto eachreal: realpath){
    // //     for (auto eacheachreal: eachreal) {
    // //         cout << eacheachreal << " ";
    // //     }
    // //     cout << endl;
    // // }

    cout << "MPC Done!!" <<endl;

    // vector<vector<double>> realpath_pid = real_path(plan_path.x, plan_path.y, plan_path.yaw, plan_path.direction);

    // cout<<"PID Done!!"<<endl;

    // save data to a dat
    ofstream savefile;
    savefile.open ("../simulation/simulatedpath.dat");
    for (size_t i = 0; i < plan_path.x.size(); i++) {
        savefile << plan_path.x[i] << " " << plan_path.y[i] << " " << plan_path.yaw[i] << " " << plan_path.yaw1[i] << " " << plan_path.direction[i] << endl;
    }
    savefile.close();

    savefile.open ("../simulation/startandgoal.dat");
    savefile << sx << " " << sy << " " << syaw0 << " " << syaw1 << " " << gx << " " << gy << " " << gyaw0 << " " << gyaw1 << endl;
    savefile.close();

    savefile.open ("../simulation/map.dat");
    for (size_t i = 0; i < map.ox.size(); i++) {
        savefile << map.ox[i] << " " << map.oy[i] << endl;
    }
    savefile.close();

    savefile.open ("../simulation/realpath.dat");
    for (size_t i = 0; i < realpath[0].size(); i++) {
        savefile << realpath[0][i] << " " << realpath[1][i] << " " << realpath[2][i] <<  " " << realpath[3][i] <<  " " << realpath[4][i] <<  " " << realpath[5][i] <<  " " << realpath[6][i] <<  " " << realpath[7][i]<< endl;
    }
    savefile.close();


    // savefile.open ("../simulation/realyaw.dat");
    // for (size_t i = 0; i < realpath[0].size(); i++) {
    //     savefile << realpath[2][i] << endl;
    // }
    // savefile.close();

    // savefile.open ("../simulation/realspeed.dat");
    // for (size_t i = 0; i < realpath[0].size(); i++) {
    //     savefile << realpath[3][i] << endl;
    // }
    // savefile.close();
    
    return 0;

}