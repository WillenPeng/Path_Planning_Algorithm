#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <math.h>

using namespace std;  

double polyeval(Eigen::VectorXd coeffs, double x);
vector<vector<double>> real_path_pid(vector<double> path_x,vector<double> path_y, vector<double> path_yaw, vector<bool> path_direction);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
vector<double> Nonlinear_EOM(double steering_angle,double throttle,double cte,double epsi,double px,double py,double psi,double v);

// //PID gain
// double K


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
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

vector<double> Nonlinear_EOM(double steering_angle,double throttle,double cte,double epsi,double px,double py,double psi,double v){
  double dt = 0.1;
  double Lf = 2.67;
  double x1    = (px + v * cos(psi) * dt);
  double y1    = (py + v * sin(psi) * dt);
  double psi1  = (psi + v * steering_angle / Lf * dt);
  double v1    = (v + throttle * dt);
  double cte1  = (cte + (v * sin(epsi) * dt));
  double epsi1 = (epsi + v * steering_angle / Lf * dt);
  vector<double> state_new;
  state_new.push_back(x1);
  state_new.push_back(y1);
  state_new.push_back(psi1);
  state_new.push_back(v1);
  state_new.push_back(cte1);
  state_new.push_back(epsi1);
  return state_new;
}

vector<vector<double>> real_path_pid(vector<double> path_x,vector<double> path_y, vector<double> path_yaw, vector<bool> path_direction){
	vector<vector<double>> state_total;
	state_total.push_back(px_total);
  	state_total.push_back(py_total);
  	state_total.push_back(psi_total);
  	state_total.push_back(v_total);
}