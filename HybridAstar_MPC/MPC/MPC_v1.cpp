#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iostream>

using CppAD::AD;
using namespace std;

// The timestep length and duration
int N = 50;
double dt = 0.1;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Factors for cost computation
double factor_cte   ;
double factor_epsi  ;
double factor_v     ;
double factor_steering;
double factor_throttle;
double factor_seq_steering;
double factor_seq_throttle;
int current_direction = 1;
double next_psi;

// Reference operating points
const double ref_cte  = 0.0;
const double ref_epsi = 0.0;
// const double ref_v    = 1.0;
double ref_v;

// Single vector of state variables and actuator variables
const int x_start     = 0;
const int y_start     = x_start     + N;
const int psi_start   = y_start     + N;
const int v_start     = psi_start   + N;
const int cte_start   = v_start     + N;
const int epsi_start  = cte_start   + N;
const int delta_start = epsi_start  + N;
const int a_start     = delta_start + N-1;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) 
  { 
    this->coeffs = coeffs; 
    // for(int i=0;i<coeffs.size();++i){
    //     cout<<"coeffs" + std::to_string(i) + "="<<coeffs[i]<<endl;
    // }
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {

    // cost
    fg[0] = 0.0;

    // Compute cost for state
    for (int i=0; i<N; i++)
    {
      fg[0] += factor_cte  * CppAD::pow(vars[cte_start + i]  - ref_cte, 2);
      fg[0] += factor_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi,2);
      fg[0] += factor_v    * CppAD::pow(vars[v_start + i]    - ref_v,2);
    }
    // cout<<"current_direction = "<<current_direction<<endl;

    // Compute cost for actuator values
    for (int i=0; i<N-1; i++)
    {
      fg[0] += factor_steering * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += factor_throttle * CppAD::pow(vars[a_start + i],     2);
    }

    // Compute cost for actuator sequence
    for (int i=0; i<N-2; i++)
    {
      fg[0] += factor_seq_steering * CppAD::pow(vars[delta_start + i+1] - vars[delta_start + i], 2);
      fg[0] += factor_seq_throttle * CppAD::pow(vars[a_start     + i+1] - vars[a_start + i],     2);
    }

    // Initial constraints
    fg[x_start    + 1] = vars[x_start];
    fg[y_start    + 1] = vars[y_start];
    fg[psi_start  + 1] = vars[psi_start];
    fg[v_start    + 1] = vars[v_start];
    fg[cte_start  + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];

    // The rest of the constraints
    for (int i=0; i<N-1; i++)
    {
      // State at time t+1
      AD<double> x1    = vars[x_start    + i+1];
      AD<double> y1    = vars[y_start    + i+1];
      AD<double> psi1  = vars[psi_start  + i+1];
      AD<double> v1    = vars[v_start    + i+1];
      AD<double> cte1  = vars[cte_start  + i+1];
      AD<double> epsi1 = vars[epsi_start + i+1];

      // State at time t
      AD<double> x0    = vars[x_start    + i];
      AD<double> y0    = vars[y_start    + i];
      AD<double> psi0  = vars[psi_start  + i];
      AD<double> v0    = vars[v_start    + i];
      AD<double> cte0  = vars[cte_start  + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Actuation at time t
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0     = vars[a_start     + i];

      // 3rd degree interpolated function
      AD<double> f0 = coeffs[0] + coeffs[1]  * x0 + coeffs[2] * pow(x0,2) + coeffs[3] * pow(x0,3);
      AD<double> psides0  = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 *coeffs[3]* pow(x0, 2));

      // Kinematic (LHS should be equal to 0 ?)
      fg[x_start    + i+2] = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start    + i+2] = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[psi_start  + i+2] = psi1  - (psi0 + v0 * delta0 / Lf * dt);
      fg[v_start    + i+2] = v1    - (v0 + a0 * dt);
      fg[cte_start  + i+2] = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[epsi_start + i+2] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(int curr_direction, double ref_v_in, int N_in, double dt_in){
  factor_cte    = 10;
  factor_epsi   = 25;
  factor_v      = 1;
  //factor_steering = 1000;
  factor_steering = 0.1;
  factor_throttle = 0.05;
  factor_seq_steering = 3.0;
  factor_seq_throttle = 0.2;
  current_direction = curr_direction;
  ref_v = ref_v_in;
  N = N_in;
  dt = dt_in;
}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // 6N variables and constraints, 2(N-1) actuations
  int n_vars        = 6*N + 2*(N-1);
  int n_constraints = 6*N;


  // Fill with zeros
  Dvector vars(n_vars);
  for (int i=0; i<n_vars; i++)
  {
    vars[i] = 0.0;
  }

  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  // Limits
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Non-actuators
  for (int i=0; i<delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // Steering limits
  for (int i=delta_start; i<a_start; i++)
  {
    // vars_lowerbound[i] = -0.25;
    // vars_upperbound[i] =  0.25;
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Accelerator / decelerator limits
  for (int i=a_start; i<n_vars; i++)
  {
    // vars_lowerbound[i] = -1;
    // vars_upperbound[i] =  1;
    vars_lowerbound[i] = -0.05;
    vars_upperbound[i] =  0.05;
    // vars_lowerbound[i] = -0.1;
    // vars_upperbound[i] =  0.1;
  }

  // Constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i=0; i<n_constraints; i++)
  {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }

  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // Options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // Solve
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>( options,
                                         vars,
                                         vars_lowerbound,
                                         vars_upperbound,
                                         constraints_lowerbound,
                                         constraints_upperbound,
                                         fg_eval,
                                         solution );


  // Store position vector data
  this->mpc_x = {};
  this->mpc_y = {};
  for (int i=0; i<N; i++)
  {
    this->mpc_x.push_back(solution.x[x_start+i]);
    this->mpc_y.push_back(solution.x[y_start+i]);
  }

  double steering_angle = solution.x[delta_start] + solution.x[delta_start+1];
  double throttle = solution.x[a_start]+ solution.x[a_start+1];
  // double steering_angle = solution.x[delta_start];
  // double throttle = solution.x[a_start];


  // double steering_angle = solution.x[delta_start];
  // double throttle = solution.x[a_start];

  return {steering_angle, throttle};
}

vector<double> MPC::GetMpcX()
{
  return mpc_x;
}

vector<double> MPC::GetMpcY()
{
  return mpc_y;
}
