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
  double T = trajectory[2][0];
  double dt;
  double max_speed = 0;
  double curr_speed;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  
  for (double i = 0; i < 100; i++) {
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
  double T = trajectory[2][0];
  double dt;
  double low_D = 0;
  double high_D = 12;
  double curr_D;
  vector <double> d = trajectory[1];
  
  for (double i = 0; i < 100; i++) {
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

double total_accel_cost(vector<vector<double>> trajectory) {
  double T = trajectory[2][0];
  double dt;
  double acc;
  double total_acc = 0;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  vector <double> s_d_dot = differentiate(s_dot);
  
  for (double i = 0; i < 100; i++) {
    dt = (T * i) / 100;
    acc = s_d_dot[0] + (s_d_dot[1] * dt) + (s_d_dot[2] * pow(dt, 2)) + (s_d_dot[3] * pow(dt, 3));
    total_acc += abs(acc*dt);
  }
  
  double acc_per_second = total_acc / T;
  
  return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double max_accel_cost(vector<vector<double>> trajectory) {
  double T = trajectory[2][0];
  double dt;
  double high_acc = 0;
  double curr_acc;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  vector <double> s_d_dot = differentiate(s_dot);
  
  for (double i = 0; i < 100; i++) {
    dt = (T * i) / 100;
    curr_acc = abs(s_d_dot[0] + (s_d_dot[1] * dt) + (s_d_dot[2] * pow(dt, 2)) + (s_d_dot[3] * pow(dt, 3)));
    if (curr_acc > high_acc) {
      high_acc = curr_acc;
    }
  }
  
  if (high_acc > MAX_ACCEL) {
    return 1;
  } else {
    return 0;
  }
}

double max_jerk_cost(vector<vector<double>> trajectory) {
  double T = trajectory[2][0];
  double dt;
  double high_jerk = 0;
  double curr_jerk;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  vector <double> s_d_dot = differentiate(s_dot);
  vector <double> jerk = differentiate(s_d_dot);
  
  for (double i = 0; i < 100; i++) {
    dt = (T * i) / 100;
    curr_jerk = abs(jerk[0] + (jerk[1] * dt) + (jerk[2] * pow(dt, 2)));
    if (curr_jerk > high_jerk) {
      high_jerk = curr_jerk;
    }
  }
  
  if (high_jerk > MAX_JERK) {
    return 1;
  } else {
    return 0;
  }
}

double total_jerk_cost(vector<vector<double>> trajectory) {
  double T = trajectory[2][0];
  double dt;
  double j;
  double total_jerk = 0;
  vector <double> s = trajectory[0];
  vector <double> s_dot = differentiate(s);
  vector <double> s_d_dot = differentiate(s_dot);
  vector <double> jerk = differentiate(s_d_dot);
  
  for (double i = 0; i < 100; i++) {
    dt = (T * i) / 100;
    j = jerk[0] + (jerk[1] * dt) + (jerk[2] * pow(dt, 2));
    total_jerk += abs(j*dt);
  }
  
  double jerk_per_second = total_jerk / T;
  
  return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}
