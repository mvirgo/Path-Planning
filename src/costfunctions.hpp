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

double exceeds_speed_limit(vector<vector<double>> traj);

double stays_on_road_cost(vector<vector<double>> traj);

double total_accel_cost(vector<vector<double>> trajectory);

double max_accel_cost(vector<vector<double>> trajectory);

double max_jerk_cost(vector<vector<double>> trajectory);

double total_jerk_cost(vector<vector<double>> trajectory);

#endif /* costfunctions_hpp */
