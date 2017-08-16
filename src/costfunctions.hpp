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

#endif /* costfunctions_hpp */
