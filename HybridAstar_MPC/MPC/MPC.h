#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

extern double factor_cte   ;
extern double factor_epsi  ;
extern double factor_v     ;
extern double factor_steering;
extern double factor_throttle;
extern double factor_seq_steering;
extern double factor_seq_throttle;
//extern int current_direction;

class MPC
{
public:
  MPC(int curr_direction, double ref_v_in, int N_in, double dt_in);
  virtual ~MPC();
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> GetMpcX();
  vector<double> GetMpcY();

  //double Update_ref_v(double ref_v, int curr_direction);

private:
  vector<double> mpc_x;
  vector<double> mpc_y;

};

#endif /* MPC_H */
