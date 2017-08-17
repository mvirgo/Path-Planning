//
//  costfunctions.hpp
//  
//
//  Created by Michael on 8/13/17.
//
//

#ifndef costfunctions_hpp
#define costfunctions_hpp

#include <stdio.h>

double logistic(double x);

vector <double> differentiate(vector <double> coeffs);

double nearest_approach(vector<vector<double>> trajectory, double target_s, double target_speed);

double total_jerk_cost(vector<vector<double>> trajectory);

double exceeds_speed_limit(vector<vector<double>> traj);

double stays_on_road_cost(vector<vector<double>> traj);

double total_accel_cost(vector<vector<double>> trajectory);

double max_accel_cost(vector<vector<double>> trajectory);

double max_jerk_cost(vector<vector<double>> trajectory);

double total_jerk_cost(vector<vector<double>> trajectory);

double collision_cost(vector<vector<double>> trajectory, double target_s, double target_speed);

double buffer_cost(vector<vector<double>> trajectory, double target_s, double target_speed);

double time_diff_cost(vector<vector<double>> trajectory, double T_goal);

double s_diff_cost(vector<vector<double>> trajectory, vector<double> s_goal);

double d_diff_cost(vector<vector<double>> trajectory, vector<double> d_goal);

#endif /* costfunctions_hpp */
