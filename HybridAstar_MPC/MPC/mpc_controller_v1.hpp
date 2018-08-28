#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3.5/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "MPC.h"
#include <math.h>

using namespace std;  

const int waypoints_count = 6;
const double dist_between_waypoints = 0.5;
const double side_friction_factor = 0.1;
const double superelevation_rate = 0;

// The timestep length and duration
int N_in = 50;
double dt_in = 0.1;

// Function Definition
double polyeval(Eigen::VectorXd coeffs, double x);
double get_ref_v(double path_length,double time_delta);
double get_path_length();
double get_time_delta();
double get_radius(Eigen::VectorXd coeffs, double x);
double get_v_limit(double radius);
double ms2mph(double v_ms);
double mph2ms(double v_mph);
double ms2kph(double v);
double kph2ms(double v);
double mph2kph(double v);
double kph2mph(double v);
vector<vector<double>> real_path(vector<double> path_x,vector<double> path_y, vector<double> path_yaw, vector<bool> path_direction);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
vector<double> Nonlinear_EOM(double steering_angle,double throttle,double cte,double epsi,double px,double py,double psi,double v);


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double get_radius(Eigen::VectorXd coeffs, double x){
  double num = coeffs[1] + 2*coeffs[2]*x + 3*coeffs[3]*pow(x,2);
  num = pow(num,2);
  num += 1;
  num = pow(num,1.5);
  double den = 2*coeffs[2] + 6*coeffs[3]*x;
  return abs(num/den);
}

double get_v_limit(double radius){
  double v = sqrt(127 * radius * (superelevation_rate+side_friction_factor));
  v = kph2mph(v);
  return v;
}

double get_path_length(){
  return waypoints_count * dist_between_waypoints ;
}

double get_ref_v(int direction, double radius){
  double v_out = get_path_length()/get_time_delta();
  double v_limit = get_v_limit(radius);
  v_out = min(v_out,v_limit);
  return v_out * direction;
}

double get_time_delta(){
  return N_in*dt_in/5.0;
}

double ms2mph(double v_ms){
  return v_ms * 2.23694;
}

double mph2ms(double v_mph){
  return v_mph / 2.23694;
}

double ms2kph(double v_ms){
  return v_ms * 3.6;
}

double kph2ms(double v_kph){
  return v_kph / 3.6;
}

double kph2mph(double v_kph){
  return v_kph * 0.621371;
}

double mph2kph(double v_mph){
  return v_mph / 0.621371;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  // auto result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(yvals);
  // auto result = A.colPivHouseholderQr().solve(yvals);
  // cout<<"result = "<<result<<endl;
  return result;
}

vector<vector<double>> real_path(vector<double> path_x, vector<double> path_y, vector<double> path_yaw, vector<bool> path_direction) {
  // printf("factor_cte    %f\n", factor_cte);
  // printf("factor_epsi   %f\n", factor_epsi);
  // printf("factor_v      %f\n", factor_v);
  // printf("factor_steering %f\n", factor_steering);
  // printf("factor_throttle %f\n", factor_throttle);
  // printf("factor_seq_steering %f\n", factor_seq_steering);
  // printf("factor_seq_throttle %f\n", factor_seq_throttle);

  //MPC mpc;
  double px = path_x[0];
  double py = path_y[0];
  //double psi = atan((path_y[1]-py)/(path_x[1]-px));
  double psi = path_yaw[0];
  int curr_direction = 1;
  double v = 0;
  double steering_angle = 0;
  double next_epsi = 0;
  double throttle = 0;
  vector<double> px_total;
  vector<double> py_total;
  vector<double> psi_total;
  vector<double> v_total;
  vector<vector<double>> real_path;

  int curr_idx_1 = 0;
  int curr_idx_2 = 0;
  int curr_idx = 0;
  int min_idx = 0;
  double x_dist = 0;
  double y_dist = 0;
  double dist_temp = 0;
  double min_dist = 999999;
  double ref_v_in;
  double radius = 1;
  double px_prev;
  Eigen::VectorXd coeffs_in;
  vector<int> curr_idx_set = {};
  int count = 0;
  int count_total = 0;
  vector<double> ptsx;
  vector<double> ptsy;
  int j;
  double prev_idx;

  while(1){
    min_dist = 999999;
    // next_epsi = 0.5*(path_yaw[]
    ptsx.clear();
    ptsy.clear();
    curr_idx_1 = curr_idx;
    curr_idx_2 = curr_idx;
    curr_idx_set.clear();
    count = 0;
    if(path_direction[curr_idx+1]) curr_direction = 1;
    else curr_direction = -1;
    if(count_total==0){ref_v_in = 0;}
    else{
      radius = get_radius(coeffs_in,px_prev);
      ref_v_in = get_ref_v(curr_direction,radius);
      cout<<"radius = "<<radius<<endl;
      cout<<"ref_v_in = "<<ref_v_in<<endl;
    }
    cout<<"count_total = "<<count_total<<endl;
    cout<<"direction = "<<curr_direction<<endl;

    MPC mpc(curr_direction,ref_v_in,N_in,dt_in);

    // //Scheme 1
    // for(int i = 0;i<path_x.size()-1;++i){
    //   if(abs(path_y[i+1]-path_y[i])>abs(path_x[i+1]-path_x[i])){
    //     if((py>=path_y[i] && py<=path_y[i+1]) || (py<=path_y[i] && py>=path_y[i+1])){
    //       curr_idx_1 = i+1;
    //       break;
    //     }
    //   }
    //   else{
    //     if((px>=path_x[i] && px<=path_x[i+1]) || (px<=path_x[i] && px>=path_x[i+1])){
    //       curr_idx_1 = i+1;
    //       break;
    //     }
    //   }
    // }

    // Scheme 2
    for(int i = 0;i<path_x.size();++i){
      x_dist = abs(px - path_x[i]);
      y_dist = abs(py - path_y[i]);
      dist_temp = sqrt(x_dist*x_dist+y_dist*y_dist);
      // cout<<"dist_temp = "<<dist_temp<<endl;
      if(dist_temp<min_dist){
        min_dist = dist_temp;
        min_idx = i;
      }
      // cout<<"min_dist = "<<min_dist<<endl;
      // cout<<"min_idx = "<<min_idx<<endl;
    }

    if(min_dist>3) break;

    // curr_idx_2 = min_idx;
    // curr_idx_1 = min(curr_idx_1 ,curr_idx_2);
    // curr_idx = min(curr_idx_1,curr_idx+1);

    // curr_idx = curr_idx_1;

    curr_idx = min_idx;
    cout<<"curr_idx = "<<curr_idx<<endl;
    // cout<<"curr_idx_1 = "<<curr_idx_1<<endl;
    // cout<<"curr_idx_2 = "<<curr_idx_2<<endl;
    // cout<<"min_dist = "<<min_dist<<endl;
    // if(curr_idx == path_x.size()-5){
    //   break;
    // }
    j = curr_idx;
    while(count < 6){
      if(j < path_x.size()) {
      //if(curr_idx >= 0) {
        curr_idx_set.push_back(j);
        j += 1;
      }
      // else{
      //   curr_idx = 0;
      //   //curr_idx = path_x.size()-1;
      //   curr_idx_set.push_back(curr_idx);
      //   curr_idx += 1;
      // }
      count += 1;
    }
    if(curr_idx_set.size()<6) break;
    if(prev_idx>curr_idx && curr_idx != 0) break;
    // cout<<"idx_set_size = "<<curr_idx_set.size()<<endl;
    for(int i=0;i<curr_idx_set.size();++i){
      ptsx.push_back(path_x[curr_idx_set[i]]);
      ptsy.push_back(path_y[curr_idx_set[i]]);
    }

    Eigen::VectorXd ptsx_rel(ptsx.size());
    Eigen::VectorXd ptsy_rel(ptsx.size());

    // Compute relative values
    for(int i=0;i<ptsx.size();++i)
    {
      double x = ptsx[i] - px;
      double y = ptsy[i] - py;

      // double x = ptsx[i] - ptsx[0];
      // double y = ptsy[i] - ptsx[0];

      ptsx_rel[i] = x*cos(psi) + y*sin(psi);
      ptsy_rel[i] = y*cos(psi) - x*sin(psi);
    }

    // Fit a polynomial
    auto coeffs = polyfit(ptsx_rel, ptsy_rel, 3);
    coeffs_in = coeffs;

    // Generate cte and epsi estimate
    double cte  = polyeval(coeffs, 0.0);
    // double epsi = -atan(coeffs[1]);
  // if(curr_idx>=1){
  //   if(path_direction[curr_idx] != path_direction[curr_idx+1]){
  //     psi += M_PI;
  //   }
  // }
    double epsi = - atan(coeffs[1]);

    // if(!path_direction[curr_idx]){
    //   epsi += M_PI;
    // }
    //cout<<"epsi = "<<epsi<<endl;
    // double epsi = 

    // Create a state vector
    Eigen::VectorXd state(ptsx.size());
    state << 0.0, 0.0, 0.0, v, cte, epsi;

    // Run MPC solver
    auto results = mpc.Solve(state, coeffs);

    steering_angle = results[0];
    throttle = results[1];
    //throttle *= curr_direction;
    px_total.push_back(px);
    py_total.push_back(py);
    psi_total.push_back(psi);
    v_total.push_back(v);
    px_prev = px;
    vector<double> state_updated = Nonlinear_EOM(steering_angle,throttle,cte,epsi,px,py,psi,v);
    px = state_updated[0];
    py = state_updated[1];
    psi = state_updated[2];
    v = state_updated[3];
    cout<<"px = "<<px<<endl;
    cout<<"py = "<<py<<endl;
    cout<<"psi = "<<psi<<endl;
    cout<<"v = "<<v<<endl; 
    cout<<"cte = "<<state_updated[4]<<endl;
    cout<<"epsi = "<<state_updated[5]<<endl;
    count_total += 1;   
    prev_idx = curr_idx;
  }

  real_path.push_back(px_total);
  real_path.push_back(py_total);
  real_path.push_back(psi_total);
  real_path.push_back(v_total);
  return real_path;
}

vector<double> Nonlinear_EOM(double steering_angle,double throttle,double cte,double epsi,double px,double py,double psi,double v){
  double Lf = 2.67;
  double x1    = (px + v * cos(psi) * dt_in);
  double y1    = (py + v * sin(psi) * dt_in);
  double psi1  = (psi + v * steering_angle / Lf * dt_in);
  double v1    = (v + throttle * dt_in);
  double cte1  = (cte + (v * sin(epsi) * dt_in));
  double epsi1 = (epsi + v * steering_angle / Lf * dt_in);
  vector<double> state_new;
  state_new.push_back(x1);
  state_new.push_back(y1);
  state_new.push_back(psi1);
  state_new.push_back(v1);
  state_new.push_back(cte1);
  state_new.push_back(epsi1);
  return state_new;
}