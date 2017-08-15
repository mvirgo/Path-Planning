//
//  trajectory.cpp
//  
//
//  Created by Michael on 8/6/17.
//
//

#include <random>
#include <string>
#include "trajectory.hpp"
#include "behavior.cpp"
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

vector<vector<double>> trajectory(double s, double d, double speed, vector<vector<double>> sensor_fusion, double T) {
  vector<double> s_start = {s, speed, 0};  // *** Need to add actual acceleration ***
  vector<double> d_start = {d, 0, 0};  // *** Need to calc actual change in d ***
  double move = double(bp.lanePlanner(s, d, sensor_fusion));
  double lane = double(bp.curr_lane);
  double goal_speed = bp.target_vehicle_speed;

  vector<double> s_goal = {s + (T * goal_speed), goal_speed, 0};
  vector<double> d_goal = {((lane * 4) + 2 + move), 0, 0};
  
  vector<vector<double>> all_s_coeffs;
  vector<vector<double>> all_d_coeffs;
  vector<vector<double>> new_goal;
  vector<double> temp_s_goal;
  vector<double> temp_d_goal;
  vector<double> temp_T_goal;
  
  for (int i = 0; i < N_SAMPLES; i++) {
    new_goal = perturbGoal(s_goal, SIGMA_S, d_goal, SIGMA_D, T, SIGMA_T);
    temp_s_goal = new_goal[0];
    temp_d_goal = new_goal[1];
    temp_T_goal = new_goal[2];
    
    all_s_coeffs.push_back(JMT(s_start, temp_s_goal, temp_T_goal[0]));
    all_d_coeffs.push_back(JMT(d_start, temp_d_goal, temp_T_goal[0]));
  }
  
  vector<double> s_coeffs = all_s_coeffs[0];  // *** Change to take in lowest cost function ***
  vector<double> d_coeffs = all_d_coeffs[0];  // *** Change to take in lowest cost function ***
  
  return {s_coeffs, d_coeffs};
  
}

vector<vector<double>> perturbGoal(vector<double> s_goal, vector<double> sig_s, vector<double> d_goal, vector<double> sig_d,
                                   double t_goal, double sig_t) {
  random_device rd;
  default_random_engine gen(rd());
  
  vector<vector<double>> perturbed_goal = {{},{},{}};
  normal_distribution<double> dist_t(t_goal, sig_t);
  
  for (int i = 0; i < 3; i++) {
    normal_distribution<double> dist_s(s_goal[i], sig_s[i]);
    normal_distribution<double> dist_d(d_goal[i], sig_d[i]);
    
    perturbed_goal[0].push_back(dist_s(gen));
    perturbed_goal[1].push_back(dist_d(gen));
    perturbed_goal[2].push_back(dist_t(gen));
  }
  
  return perturbed_goal;
}
