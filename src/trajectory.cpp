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
#include "costfunctions.cpp"
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
  // Set current state, calculate mean goals desired
  vector<double> s_start = {s, speed, 0};  // *** Need to add actual acceleration ***
  vector<double> d_start = {d, 0, 0};  // *** Need to calc actual change in d ***
  double move = bp.lanePlanner(s, d, sensor_fusion);
  double lane = bp.curr_lane;
  double goal_speed = bp.target_vehicle_speed;
  int best;

  bp.s_goal = {s + (T * goal_speed), goal_speed, 0};
  bp.d_goal = {((lane * 4) + 2 + move), 0, 0};
  bp.T_goal = T;
  
  // Vectors to hold coefficients for goal perturbing
  vector<vector<double>> all_s_coeffs;
  vector<vector<double>> all_d_coeffs;
  vector<vector<double>> all_T;
  vector<vector<double>> new_goal;
  
  // Take samples from a normal distribution around the mean goal s, d and T
  for (int i = 0; i < N_SAMPLES; i++) {
    new_goal = perturbGoal(bp.s_goal, SIGMA_S, bp.d_goal, SIGMA_D, bp.T_goal, SIGMA_T);  // Outputs perturbed s_goal, d_goal, T_goal
    
    all_s_coeffs.push_back(JMT(s_start, new_goal[0], new_goal[2][0]));
    all_d_coeffs.push_back(JMT(d_start, new_goal[1], new_goal[2][0]));
    all_T.push_back(new_goal[2]);
  }
  
  best = bestTraj(all_s_coeffs, all_d_coeffs, all_T);
  
  return {all_s_coeffs[best], all_d_coeffs[best], all_T[best]};  // Best s_coeffs, d_coeffs, and T
  
}

vector<vector<double>> perturbGoal(vector<double> s_goal, vector<double> sig_s, vector<double> d_goal, vector<double> sig_d,
                                   double t_goal, double sig_t) {
  random_device rd;
  default_random_engine gen(rd());
  
  vector<vector<double>> perturbed_goal = {{},{},{}};
  normal_distribution<double> dist_t(t_goal, sig_t);
  perturbed_goal[2].push_back(dist_t(gen));
  
  for (int i = 0; i < s_goal.size(); i++) {
    normal_distribution<double> dist_s(s_goal[i], sig_s[i]);
    normal_distribution<double> dist_d(d_goal[i], sig_d[i]);
    
    perturbed_goal[0].push_back(dist_s(gen));
    perturbed_goal[1].push_back(dist_d(gen));
  }
  
  return perturbed_goal;
}

int bestTraj(vector<vector<double>> s_coeffs, vector<vector<double>> d_coeffs, vector<vector<double>> all_T) {
  vector<double> costs;
  
  for (int i = 0; i < s_coeffs.size(); i++) {
    costs.push_back(calcCost({s_coeffs[i], d_coeffs[i], all_T[i]}));
  }
  
  return min_element(costs.begin(), costs.end()) - costs.begin();
}

double calcCost(vector<vector<double>> trajectory) {
  double cost = 0;
  vector <double> weights = {999, 999, 2, 999, 999, 2, 999, 50, 5, 10, 20, 100};  // *** Can tune for cost functions ***
  
  cost += exceeds_speed_limit(trajectory) * weights[0];
  cost += stays_on_road_cost(trajectory) * weights[1];
  cost += total_accel_cost(trajectory) * weights[2];
  cost += max_accel_cost(trajectory) * weights[3];
  cost += max_jerk_cost(trajectory) * weights[4];
  cost += total_jerk_cost(trajectory) * weights[5];
  cost += collision_cost(trajectory, bp.target_vehicle_s, bp.target_vehicle_speed) * weights[6];
  cost += buffer_cost(trajectory, bp.target_vehicle_s, bp.target_vehicle_speed) * weights[7];
  cost += time_diff_cost(trajectory, bp.T_goal) * weights[8];
  cost += s_diff_cost(trajectory, bp.s_goal) * weights[9];
  cost += d_diff_cost(trajectory, bp.d_goal) * weights[10];
  cost += speed_cost(trajectory, bp.target_vehicle_speed) * weights[11];

  return cost;
}
