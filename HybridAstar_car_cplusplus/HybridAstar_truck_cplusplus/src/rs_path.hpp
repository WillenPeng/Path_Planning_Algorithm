// path class
#ifndef RS_PATH_H
#define RS_PATH_H

#include <vector>
#include <string>
#include <math.h>
#include <numeric>

#include "def_all.hpp"

using namespace std;

struct LengthPart;
class LocalPoints;
class Path;

double pi_2_pi(double angle);
vector<Path*> calc_paths(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size);
pair<double, double> polar(double x, double y);
double mod2pi(double x);
LengthPart LSL(double x, double y, double phi);
LengthPart LSR(double x, double y, double phi);
LengthPart LRL(double x, double y, double phi);
void set_path(vector<Path*> &paths, vector<double> lengths, string ctypes);
void SCS(double x, double y, double phi, vector<Path*> &paths);
LengthPart SLS(double x, double y, double phi);
void CSC(double x, double y, double phi, vector<Path*> &paths);
void CCC(double x, double y, double phi, vector<Path*> &paths);
pair<double, double> calc_tauOmega(double u, double v, double xi, double eta, double phi);
LengthPart LRLRn(double x, double y, double phi);
LengthPart LRLRp(double x, double y, double phi);
void CCCC(double x, double y, double phi, vector<Path*> &paths);
LengthPart LRSR(double x, double y, double phi);
LengthPart LRSL(double x, double y, double phi);
void CCSC(double x, double y, double phi, vector<Path*> &paths);
LengthPart LRSLR(double x, double y, double phi);
void CCSCC(double x, double y, double phi, vector<Path*> &paths);
LocalPoints generate_local_course(double L, vector<double> lengths, string mode, double maxc, double step_size);
void interpolate(int ind, double l, char m, double maxc, double ox, double oy, double oyaw, LocalPoints &localpoints);
vector<Path*> generate_path(vector<double> q0, vector<double> q1, double maxc);


const double MAX_PATH_LENGTH = 1000.0;

struct LengthPart {
    bool flag = false;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
};

class LocalPoints {
  public:
	LocalPoints() {};
    LocalPoints(vector<double> px, vector<double> py, vector<double> pyaw, vector<double> directions)
        :px(px)
        , py(py)
        , pyaw(pyaw)
        , directions(directions) 
    {}

  public:
    vector<double> px;
    vector<double> py;
    vector<double> pyaw;
    vector<double> directions;
};

class Path {
  public:
	Path() {};
    Path(vector<double> lengths, string ctypes,double L, vector<double> x, vector<double> y, vector<double> yaw, vector<double> yaw1, vector<double> directions, double pathcost)
        :lengths(lengths)
        , ctypes(ctypes)
        , L(L)
        , x(x)
        , y(y)
        , yaw(yaw)
        , yaw1(yaw1)
        , directions(directions)
        , pathcost(pathcost) 
    {}

    double calc_rs_path_cost() {
        // calculate the path cost
        double cost = 0.0;
        for (auto l: lengths) {
            if (l >= 0) // forward
                cost += l;
            else // back
                cost += abs(l) * BACK_COST;
        }
        // swich back penalty
        for (size_t i = 0; i < lengths.size()-1; i++) {
            if (lengths[i] * lengths[i+1] < 0.0) // switch back
                cost += SB_COST;
        }
        // steer penalyty
        for (auto ctype: ctypes) {
            if (ctype != 'S') // curve
                cost += STEER_COST*abs(MAX_STEER);
        }

        // ==steer change penalty
        // calc steer profile
		size_t nctypes = ctypes.size();
        vector<double> ulist(nctypes, 0.0);
        for (size_t i = 0; i < nctypes; i++) {
            if (ctypes[i] == 'R')
                ulist[i] = - MAX_STEER;
            else if (ctypes[i] == 'L')
                ulist[i] = MAX_STEER;
        }
        for (size_t i = 0; i < nctypes-1; i++) {
            cost += STEER_CHANGE_COST*abs(ulist[i+1] - ulist[i]);
        }
        double jack_cost = 0.0;
        for (size_t i = 0; i < yaw1.size(); i++) {
            jack_cost += abs(pi_2_pi(yaw[i] - yaw1[i]));
        }
        cost += (JACKKNIF_COST * jack_cost);
        pathcost = cost;
        return cost;
    }

  public:
        vector<double> lengths; //lengths of each part of the path +: forward, -: backward
        string ctypes; // type of each part of the path
        double L; // total path length
        vector<double> x; // final x positions [m]
        vector<double> y; // final y positions [m]
        vector<double> yaw; // final yaw angles [rad]
		vector<double> yaw1; // final tralier yaw angles [rad]
        vector<double> directions; // forward:1, backward:-1 
		double pathcost; // pathcost
};
    
vector<Path*> calc_paths(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size) {
    vector<double> q0 = {sx, sy, syaw};
    vector<double> q1 = {gx, gy, gyaw};

    // use Reed-Shepp model to find multiple paths between current node && goal node without obstacles, but each path only contain the length, types && final position without each point in the path
    vector<Path*> paths = generate_path(q0, q1, maxc);
    LocalPoints localpoints;
    for (auto path: paths){
        // get local coordinates of all points in each path
        localpoints = generate_local_course(path->L, path->lengths, path->ctypes, maxc, step_size*maxc);
        vector<double> xpath;
        vector<double> ypath;
        vector<double> yawpath;
        double ix, iy, iyaw;
        for (size_t i = 0; i < localpoints.px.size(); i++) {
            ix = localpoints.px[i];

            iy = localpoints.py[i];
            iyaw = localpoints.pyaw[i];
            xpath.emplace_back(cos(-q0[2]) * ix + sin(-q0[2]) * iy + q0[0]);
            ypath.emplace_back(-sin(-q0[2]) * ix + cos(-q0[2]) * iy + q0[1]);
            yawpath.emplace_back(pi_2_pi(iyaw + q0[2]));
        }
               
        path->x = xpath;
        path->y = ypath;
        path->yaw = yawpath;
        path->directions = localpoints.directions;
        for (size_t i = 0; i < path->lengths.size(); i++) {
            path->lengths[i] /= maxc;
        }
        path->L /= maxc;
    } 
    return paths;
}

void set_path(vector<Path*> &paths, vector<double> lengths, string ctypes) {
    Path* path = new Path(lengths,ctypes,0.0,{},{},{},{},{},0.0);

    // check same path exist
    for (auto tpath: paths) {
        if (tpath->ctypes == path->ctypes) {
            if (accumulate(tpath->lengths.begin(), tpath->lengths.end(), 0) - accumulate(path->lengths.begin(), path->lengths.end(), 0) <= 0.01){
                delete path;
                return;  // not insert path
            }
        }
        
    }
    for (auto eachlengths: lengths) {
        path->L += abs(eachlengths);
    }

    if (path->L >= MAX_PATH_LENGTH) {
        delete path;
        return; // not insert path
    }

    paths.emplace_back(path);

    return;
}

vector<Path*> generate_path(vector<double> q0, vector<double> q1, double maxc) {
    // use Reed-Shepp model to find a path between current node && goal node without obstacles.
    // Reed-Shepp model is non-holonomic-without-obstacles to find a path between two node && car can both forward && reverse.
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double dth = q1[2] - q0[2];
    double c = cos(q0[2]);
    double s = sin(q0[2]);
    double x = (c*dx + s*dy)*maxc;
    double y = (-s*dx + c*dy)*maxc;
 
    vector<Path*> paths;
    // different condition to build a path(S:straight, C:Curve)
    SCS(x, y, dth, paths); 
    CSC(x, y, dth, paths); 
    CCC(x, y, dth, paths);
    CCCC(x, y, dth, paths);
    CCSC(x, y, dth, paths);
    // CCSCC(x, y, dth, paths);

    return paths;
}

LocalPoints generate_local_course(double L, vector<double> lengths, string mode, double maxc, double step_size) {
    int npoint = int(trunc(L / step_size)) + lengths.size() + 3;
    vector<double> px(npoint, 0.0);
    vector<double> py(npoint, 0.0);
    vector<double> pyaw(npoint, 0.0);
    vector<double> directions(npoint, 0.0);
    LocalPoints localpoints(px, py, pyaw, directions);
    int ind = 1;
    double d;
    
    if (lengths[0] > 0.0) {
        directions[0] = 1.0;
        d = step_size;
    }
    else{
        directions[0] = -1.0;
        d = -step_size;
    }

    double pd = d;
    double ll = 0.0;

    char m;
    double l;
    double ox, oy, oyaw;
    for (size_t i = 0; i < mode.size(); i++) {
        m = mode[i];
        l = lengths[i];

        if (l > 0.0)
            d = step_size;
        else
            d = -step_size;
        
        // set origin state
        ox = localpoints.px[ind];
        oy = localpoints.py[ind];
        oyaw = localpoints.pyaw[ind];
        ind -= 1;
        if (i >= 1 && (lengths[i-1]*lengths[i]) > 0)
            pd = - d - ll;
        else
            pd = d - ll;
        while (abs(pd) <= abs(l)) {
            ind += 1;
            interpolate(ind, pd, m, maxc, ox, oy, oyaw, localpoints);
            pd += d;
        }
        
        ll = l - pd - d; // calc remain length

        ind += 1;
        interpolate(ind, l, m, maxc, ox, oy, oyaw, localpoints);
    }

    //remove unused data
    while (abs(localpoints.px.back()) < 1E-5) {
        localpoints.px.pop_back();
        localpoints.py.pop_back();
        localpoints.pyaw.pop_back();
        localpoints.directions.pop_back();
    }
    return localpoints;
}

void interpolate(int ind, double l, char m, double maxc, double ox, double oy, double oyaw, LocalPoints &localpoints) {

    if (m == 'S'){
        localpoints.px[ind] = ox + l / maxc * cos(oyaw);
        localpoints.py[ind] = oy + l / maxc * sin(oyaw);
        localpoints.pyaw[ind] =  oyaw;
    }      
    else{ // curve
        double ldx = sin(l) / maxc;
        double ldy;
        if (m == 'L')  // left turn
            ldy = (1.0 - cos(l)) / maxc;
        else if (m == 'R')  // right turn
            ldy = (1.0 - cos(l)) / -maxc;
        double gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy;
        double gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy;
        localpoints.px[ind] = ox + gdx;
        localpoints.py[ind] = oy + gdy;
    }
    
    if (m == 'L')  // left turn
        localpoints.pyaw[ind] = oyaw + l;
    else if (m == 'R')  // right turn
        localpoints.pyaw[ind] = oyaw - l;

    if (l > 0.0)
        localpoints.directions[ind] = 1.0;
    else
        localpoints.directions[ind] = -1.0;
}

double pi_2_pi(double angle) {
    while(angle > PI) angle -= 2.0 * PI;
    while(angle < -PI) angle += 2.0 * PI;
    return angle;
}

pair<double, double> polar(double x, double y) {
    double r = sqrt(pow(x,2) + pow(y,2));
    double theta = atan2(y, x);
    return make_pair(r, theta);
}
    
double mod2pi(double x) {
    double v = fmod(x, 2.0 * PI);
    if (v < -PI) {
        v += 2.0 * PI;
    }
    else{
        if (v > PI){
            v -= 2.0 * PI;
        }
    }
    return v;
}
    
LengthPart LSL(double x, double y, double phi) {
    LengthPart lengthpart;
    pair<double, double> u_and_t;
    u_and_t = polar(x - sin(phi), y - 1.0 + cos(phi));
    double u = u_and_t.first, t = u_and_t.second;
    if (t >= 0.0) {
        double v = mod2pi(phi - t);
        if (v >= 0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }
    }
    return lengthpart;
}
    

LengthPart LSR(double x, double y, double phi) {
    LengthPart lengthpart;
    pair<double, double> u1_and_t1;
    u1_and_t1 = polar(x + sin(phi), y - 1.0 - cos(phi));
    double u1 = u1_and_t1.first, t1 = u1_and_t1.second;
    u1 = pow(u1,2);
    if (u1 >= 4.0) {
        double u = sqrt(u1 - 4.0);
        double theta = atan2(2.0, u);
        double t = mod2pi(t1 + theta);
        double v = mod2pi(t - phi);
        if (t >= 0.0 && v >= 0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }
    }  
    return lengthpart;
}

LengthPart LRL(double x, double y, double phi) {
    LengthPart lengthpart;
    pair<double, double> u1_and_t1;
    u1_and_t1 = polar(x - sin(phi), y - 1.0 + cos(phi));
    double u1 = u1_and_t1.first, t1 = u1_and_t1.second;
    if (u1 <= 4.0) {
        double u = -2.0 * asin(0.25 * u1);
        double t = mod2pi(t1 + 0.5 * u + PI);
        double v = mod2pi(phi - t + u);
        if (t >= 0.0 && u <= 0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }
    }  
    return lengthpart;
} 

void SCS(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = SLS(x, y, phi);
    if (lengthpart.flag) 
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "SLS");
    
    lengthpart = SLS(x, -y, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "SRS");
}

LengthPart SLS(double x, double y, double phi) {
    LengthPart lengthpart;
    phi = mod2pi(phi);
    double xd;
    if (y > 0.0 && phi > 0.0 && phi < PI * 0.99) {
        xd = - y / tan(phi) + x;
        lengthpart.flag = true;
        lengthpart.t = xd - tan(phi / 2.0);
        lengthpart.u = phi;
        lengthpart.v = sqrt(pow(x - xd,2) + pow(y,2)) - tan(phi / 2.0);
        return lengthpart;
    }    
    if (y < 0.0 && phi > 0.0 && phi < PI * 0.99) {
        xd = - y / tan(phi) + x;
        lengthpart.flag = true;
        lengthpart.t = xd - tan(phi / 2.0);
        lengthpart.u = phi;
        lengthpart.v = -sqrt(pow(x - xd,2) + pow(y,2)) - tan(phi / 2.0);
        return lengthpart;
    }
    return lengthpart;
}

void CSC(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = LSL(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "LSL");
    lengthpart = LSL(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "LSL");
    lengthpart = LSL(x, -y, -phi);
    if (lengthpart.flag) {
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "RSR");
    }
    lengthpart = LSL(-x, -y, phi);
    if (lengthpart.flag) {
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "RSR");
    }
    
    lengthpart = LSR(x, y, phi); 
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "LSR");
    
    lengthpart = LSR(-x, y, -phi); 
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "LSR");
    
    lengthpart = LSR(x, -y, -phi); 
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "RSL");
    
    lengthpart = LSR(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "RSL");
}

void CCC(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = LRL(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "LRL");
    
    lengthpart = LRL(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "LRL");
    
    lengthpart = LRL(x, -y, -phi); 
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.v}, "RLR");
    
    lengthpart = LRL(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.v}, "RLR");

    // backwards
    double xb = x * cos(phi) + y * sin(phi);
    double yb = x * sin(phi) - y * cos(phi);

    lengthpart = LRL(xb, yb, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, lengthpart.t}, "LRL");
    
    lengthpart = LRL(-xb, yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, -lengthpart.t}, "LRL");
    
    lengthpart = LRL(xb, -yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, lengthpart.t}, "RLR");
    
    lengthpart = LRL(-xb, -yb, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, -lengthpart.t}, "RLR");
     
    
}

pair<double, double> calc_tauOmega(double u, double v, double xi, double eta, double phi) {
    LengthPart lengthpart;
    double delta = mod2pi(u-v);
    double A = sin(u) - sin(delta);
    double B = cos(u) - cos(delta) - 1.0;

    double t1 = atan2(eta*A - xi*B, xi*A + eta*B);
    double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;
    double tau;
    if (t2 < 0)
        tau = mod2pi(t1+PI);
    else
        tau = mod2pi(t1);
    
    double omega = mod2pi(tau - u + v - phi);

    return make_pair(tau, omega);
}
    

LengthPart LRLRn(double x, double y, double phi) {
    LengthPart lengthpart;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = 0.25 * (2.0 + sqrt(xi*xi + eta*eta));

    if (rho <= 1.0){
        double u = acos(rho);
        pair<double, double> t_and_v;
        t_and_v = calc_tauOmega(u, -u, xi, eta, phi);
        double t = t_and_v.first, v = t_and_v.second;
        if (t >= 0.0 && v <= 0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }
    }  
    return lengthpart;
}

LengthPart LRLRp(double x, double y, double phi) {
    LengthPart lengthpart;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = (20.0 - xi*xi - eta*eta) / 16.0;

    if (rho>=0.0 && rho<=1.0) {
        double u = -acos(rho);
        if (u >= -0.5 * PI) {
            pair<double, double> t_and_v;
            t_and_v = calc_tauOmega(u, u, xi, eta, phi);
            double t = t_and_v.first, v = t_and_v.second;
            if (t >= 0.0 && v >= 0.0) {
                lengthpart.flag = true;
                lengthpart.t = t;
                lengthpart.u = u;
                lengthpart.v = v;
                return lengthpart; 
            }
        }
    }   
    return lengthpart;
}

void CCCC(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = LRLRn(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, -lengthpart.u, lengthpart.v}, "LRLR");
    
    lengthpart = LRLRn(-x, y, -phi); 
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, lengthpart.u, -lengthpart.v}, "LRLR");
     
    lengthpart = LRLRn(x, -y, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, -lengthpart.u, lengthpart.v}, "RLRL");
     
    lengthpart = LRLRn(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, lengthpart.u, -lengthpart.v}, "RLRL");
    
    lengthpart = LRLRp(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.u, lengthpart.v}, "LRLR");
    
    lengthpart = LRLRp(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.u, -lengthpart.v}, "LRLR");
    
    lengthpart = LRLRp(x, -y, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, lengthpart.u, lengthpart.u, lengthpart.v}, "RLRL");
    
    lengthpart = LRLRp(-x, -y, phi); 
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, -lengthpart.u, -lengthpart.u, -lengthpart.v}, "RLRL");
    
    
}

LengthPart LRSR(double x, double y, double phi) {
    LengthPart lengthpart;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    pair<double, double> rho_and_theta;
    rho_and_theta = polar(-eta, xi);
    double rho = rho_and_theta.first, theta = rho_and_theta.second;

    if (rho >= 2.0) {
        double t = theta;
        double u = 2.0 - rho;
        double v = mod2pi(t + 0.5*PI - phi);
        if (t >= 0.0 && u <= 0.0 && v <=0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }
    }
    return lengthpart;
}

LengthPart LRSL(double x, double y, double phi) {
    LengthPart lengthpart;
    double xi = x - sin(phi);
    double eta = y - 1.0 + cos(phi);
    pair<double, double> rho_and_theta;
    rho_and_theta = polar(xi, eta);
    double rho = rho_and_theta.first, theta = rho_and_theta.second;

    if (rho >= 2.0) {
        double r = sqrt(rho*rho - 4.0);
        double u = 2.0 - r;
        double t = mod2pi(theta + atan2(r, -2.0));
        double v = mod2pi(phi - 0.5*PI - t);
        if (t >= 0.0 && u<=0.0 && v<=0.0) {
            lengthpart.flag = true;
            lengthpart.t = t;
            lengthpart.u = u;
            lengthpart.v = v;
            return lengthpart;
        }  
    }
    return lengthpart;
}

void CCSC(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = LRSL(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, lengthpart.v}, "LRSL");
    
    lengthpart = LRSL(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, -lengthpart.v}, "LRSL");
    
    lengthpart = LRSL(x, -y, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, lengthpart.v}, "RLSR");
    
    lengthpart = LRSL(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, -lengthpart.v}, "RLSR");
    
    lengthpart = LRSR(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, lengthpart.v}, "LRSR");
    
    lengthpart = LRSR(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, -lengthpart.v}, "LRSR");
    
    lengthpart = LRSR(x, -y, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, lengthpart.v}, "RLSL");
    
    lengthpart = LRSR(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, -lengthpart.v}, "RLSL");
    
    // backwards
    double xb = x*cos(phi) + y*sin(phi);
    double yb = x*sin(phi) - y*cos(phi);
    lengthpart = LRSL(xb, yb, phi); 
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, -0.5*PI, lengthpart.t}, "LSRL");
    
    lengthpart = LRSL(-xb, yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, 0.5*PI, -lengthpart.t}, "LSRL");
    
    lengthpart = LRSL(xb, -yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, -0.5*PI, lengthpart.t}, "RSLR");
    
    lengthpart = LRSL(-xb, -yb, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, 0.5*PI, -lengthpart.t}, "RSLR");
    
    lengthpart = LRSR(xb, yb, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, -0.5*PI, lengthpart.t}, "RSRL");
    
    lengthpart = LRSR(-xb, yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, 0.5*PI, -lengthpart.t}, "RSRL");
    
    lengthpart = LRSR(xb, -yb, -phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.v, lengthpart.u, -0.5*PI, lengthpart.t}, "LSLR");
    
    lengthpart = LRSR(-xb, -yb, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.v, -lengthpart.u, 0.5*PI, -lengthpart.t}, "LSLR");
    
    
}   

LengthPart LRSLR(double x, double y, double phi) {
    // formula 8.11 *** TYPO IN PAPER ***
    LengthPart lengthpart;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    pair<double, double> rho_and_theta;
    rho_and_theta = polar(xi, eta);
    double rho = rho_and_theta.first, theta = rho_and_theta.second;
    if (rho >= 2.0){
        double u = 4.0 - sqrt(rho*rho - 4.0);
        if (u <= 0.0){
            double t = mod2pi(atan2((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta));
            double v = mod2pi(t - phi);

            if (t >= 0.0 && v >=0.0){
                lengthpart.flag = true;
                lengthpart.t = t;
                lengthpart.u = u;
                lengthpart.v = v;
                return lengthpart;
            }
        }  
    }
    return lengthpart;
}

void CCSCC(double x, double y, double phi, vector<Path*> &paths) {
    LengthPart lengthpart;
    lengthpart = LRSLR(x, y, phi);
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, -0.5*PI, lengthpart.v}, "LRSLR");
    
    lengthpart = LRSLR(-x, y, -phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, 0.5*PI, -lengthpart.v}, "LRSLR");
    
    lengthpart = LRSLR(x, -y, -phi); 
    if (lengthpart.flag)
        set_path(paths, {lengthpart.t, -0.5*PI, lengthpart.u, -0.5*PI, lengthpart.v}, "RLSRL");
    
    lengthpart = LRSLR(-x, -y, phi);
    if (lengthpart.flag)
        set_path(paths, {-lengthpart.t, 0.5*PI, -lengthpart.u, 0.5*PI, -lengthpart.v}, "RLSRL");
    
    
}

#endif