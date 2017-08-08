//
//  trajectory.cpp
//  
//
//  Created by Michael on 8/6/17.
//
//

#include "trajectory.hpp"
#include "constants.cpp"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

BehaviorPlanner bp;

vector<double> JMT(vector<double> start, vector <double> end, double T) {
  
  MatrixXd t_matrix(3, 3);
  MatrixXd s_matrix(3, 1);
  
  t_matrix << pow(T,3), pow(T,4), pow(T,5),
              3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T,4),
              6 * T, 12 * pow(T,2), 20 * pow(T,3);
  s_matrix << end[0] - (start[0] + (start[1] * T) + (0.5 * start[2] * pow(T,2))),
              end[1] - (start[1] + (start[2] * T)),
              end[2] - start[2];
  MatrixXd t_inverse = t_matrix.inverse();
  MatrixXd coeffs = t_inverse * s_matrix;
  
  return {start[0], start[1], 0.5 * start[2],
    coeffs.data()[0], coeffs.data()[1], coeffs.data()[2]};
  
}

vector<vector<double>> trajectory(double s, double d, double speed, vector<vector<double>> sensor_fusion) {
  vector<double> s_start = {s, speed, 0};  // *** Need to add actual acceleration ***
  vector<double> d_start = {d, 0, 0};  // *** Need to calc actual change in d ***
  double move = double(bp.lanePlanner(s, d, sensor_fusion));
  double lane = double(bp.curr_lane);
  
  double T = 5;  // *** Tune this for duration to calculate trajectory ***
  vector<double> s_goal = {s + T * (SPEED_LIMIT - 10), speed, 0};  // *** Want to set speed based on cars in front, or limit ***
  vector<double> d_goal = {((lane * 4) + 2 + move), 0, 0};
  
  vector<double> s_coeffs = JMT(s_start, s_goal, T);
  vector<double> d_coeffs = JMT(d_start, d_goal, T);
  
  return {s_coeffs, d_coeffs};  // *** This is wrong, need to calculate trajectory and return different values ***
  
}
