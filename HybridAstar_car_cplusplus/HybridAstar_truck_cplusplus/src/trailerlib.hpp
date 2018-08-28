// some libraries and defs of trailier
#ifndef TRAILERLIB_H
#define TRAILERLIB_H

#include <iostream>
#include <vector>
#include <math.h>
#include "kdtree.h"

#include "map.hpp"
#include "def_all.hpp"

using namespace std;

bool check_collision(vector<double>* x, vector<double>* y, vector<double>* yaw, Map* map);
bool rect_check(double ix, double iy, double iyaw, vector<double>* ox, vector<double>* oy);
vector<double> calc_trailer_yaw_from_xyyaw(vector<double>* yaw, double init_tyaw, vector<double>* steps);
bool check_trailer_collision(Map* map, vector<double>* x, vector<double>* y, double yaw0, double yaw1);


bool check_collision(vector<double>* x, vector<double>* y, vector<double>* yaw, Map* map) {
    double ix, iy, iyaw, cx, cy;
    vector<double> oxnew, oynew;
    for (size_t i = 0; i < x->size(); i++) {
        ix = (*x)[i];
        iy = (*y)[i];
        iyaw = (*yaw)[i];
        // calculate rectangular center position based on rear-axis center
        cx = ix + DF*cos(iyaw);
        cy = iy + DF*sin(iyaw);
        // cx = ix;
        // cy = iy;
        // cout << "cxy: " << cx << " " << cy << " " << DFR << endl;
        const std::vector<int> radIndices = map->kdtree.radiusSearch({cx, cy}, DFR);
        if (!radIndices.empty()) {
            // cout << "radIndices: " << radIndices.size() << " " << (*map->ox)[radIndices[0]] << " " << (*map->oy)[radIndices[0]] << endl;
            return false; // collision
        }
        // if (radIndices.empty())
        //     continue; 

        // oxnew.clear();
        // oynew.clear();
        // oxnew.reserve(radIndices.size());
        // oynew.reserve(radIndices.size());
        // for (size_t j = 0; j < radIndices.size(); j++){
        //     oxnew.emplace_back((*map->ox)[radIndices[j]]);
        //     oynew.emplace_back((*map->oy)[radIndices[j]]);
        // }
        // if (!rect_check(ix, iy, iyaw, &oxnew, &oynew))
        //     return false; // collision
            
    }
    return true; // OK
} 

bool rect_check(double ix, double iy, double iyaw, vector<double>* ox, vector<double>* oy) {
    double c = cos(-iyaw);
    double s = sin(-iyaw);
    double iox, ioy, tx, ty, lx, ly;
    for (size_t i = 0; i < ox->size(); i++) {
        iox = (*ox)[i];
        ioy = (*oy)[i];
        tx = iox - ix;
        ty = ioy - iy;
        // the projection on vehicle direction (x) and its perpendicular direction (y)
        lx = (c*tx - s*ty);
        ly = (s*tx + c*ty);

        double sumangle = 0.0;
        double x1, y1, x2, y2, d1, d2, theta1, tty, tmp;
        for (size_t j = 0; j < VRXT.size()-1; j++) {
            x1 = VRXT[j] - lx;
            y1 = VRYT[j] - ly;
            x2 = VRXT[j+1] - lx;
            y2 = VRYT[j+1] - ly;
            d1 = hypot(x1, y1);
            d2 = hypot(x2, y2);
            theta1 = atan2(y1,x1);
            tty = (-sin(theta1)*x2 + cos(theta1)*y2);
            tmp = (x1*x2+y1*y2)/(d1*d2);

            if (tmp >= 1.0)
                tmp = 1.0;
            else if (tmp <= 0.0)
                tmp = 0.0;
            
            if (tty >= 0.0)
                sumangle += acos(tmp);
            else
                sumangle -= acos(tmp);
        }
        if (sumangle >= PI)
            return false; // collision
    }
    return true; // OK
}   

vector<double> calc_trailer_yaw_from_xyyaw(vector<double>* yaw, double init_tyaw, vector<double>* steps) {
    /*
    calc trailer yaw from x y yaw lists
    */
    vector<double> tyaw(yaw->size(), 0.0);
    tyaw[0] = init_tyaw;

    for (size_t i = 0; i < yaw->size(); i++) {
        tyaw[i] += tyaw[i-1] + (*steps)[i-1]/LT*sin((*yaw)[i-1]- tyaw[i-1]);
    }
    return tyaw;
}

bool check_trailer_collision(Map* map, vector<double>* x, vector<double>* y, vector<double>* yaw0, vector<double>* yaw1) {
    /*
    collision check def for trailer

    */
    // check collision for rear trailer
    // if (!check_collision(x, y, yaw1, map)) {
    //     return false;
    // }

    // check collision for front truck
    if (!check_collision(x, y, yaw0, map)){
        return false;
    }

    // avoid large difference between yaw0 and yaw1 
    // for (size_t i = 0; i < yaw0->size(); i++) {
    //     if (cos((*yaw0)[i])*cos((*yaw1)[i]) + sin((*yaw0)[i])*sin((*yaw1)[i]) <= 0) {
    //         return false;
    //     }
    // }
    
    return true; // OK
}


#endif