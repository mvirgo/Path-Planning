//
//  costfunctions.cpp
//  
//
//  Created by Michael on 8/13/17.
//
//

#include <math.h>
#include "costfunctions.hpp"
#include "constants.cpp"

double logistic(double x) {
  return 2 / (1 + exp(-x)) - 1;
}

vector <double> differentiate(vector <double> coeffs) {
  vector <double> derivative;
  for (int i = 1; i < coeffs.size(); i++) {
    derivative.push_back(coeffs[i] * i);
  }
  return derivative;
}

double exceeds_speed_limit(vector<vector<double>> trajectory) {
  double T = trajectory[3][0];
  double dt;
  double max_speed = 0;
  double curr_speed;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  
  for (double i = 0; i < T; i++) {
    dt = (T * i) / 100;
    curr_speed = s_dot[0] + (s_dot[1] * dt) + (s_dot[2] * pow(dt, 2)) + (s_dot[3] * pow(dt, 3)) + (s_dot[4] * pow(dt, 4));
    if (curr_speed > max_speed) {
      max_speed = curr_speed;
    }
  }
  if (max_speed > SPEED_LIMIT) {
    return 1;
  } else {
    return 0;
  }
}

double stays_on_road_cost(vector<vector<double>> trajectory) {
  double T = trajectory[3][0];
  double dt;
  double low_D = 0;
  double high_D = 12;
  double curr_D;
  vector <double> d = trajectory[1];
  
  for (double i = 0; i < T; i++) {
    dt = (T * i) / 100;
    curr_D = d[0] + (d[1] * dt) + (d[2] * pow(dt, 2)) + (d[3] * pow(dt, 3)) + (d[4] * pow(dt, 4)) + (d[5] * pow(dt,5));
    if (curr_D < low_D) {
      low_D = curr_D;
    } else if (curr_D < high_D) {
      high_D = curr_D;
    }
  }
  
  if (low_D < MIN_D or high_D > MAX_D) {
    return 1;
  } else {
    return 0;
  }
}
