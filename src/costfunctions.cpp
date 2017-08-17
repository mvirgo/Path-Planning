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

double to_equation(vector<double> state, double dt) {
  double total = 0;
  
  for (int i = 0; i < state.size(); i++) {
    total += state[i] * pow(dt, i);
  }
  
  return total;
}

double nearest_approach(vector<vector<double>> trajectory, double target_s, double target_speed) {
  double T = trajectory[2][0];
  double dt;
  double curr_s;
  double nearest = 99999;
  double vehicle_s = target_s;
  vector <double> s = trajectory[0];
  
  for (double i = 0; i < 100; i++) {
    dt = (T * i) / 100;
    curr_s = to_equation(s, dt);
    vehicle_s += target_speed * dt;  // Simplify as if straight line, constant speed
    if (abs(curr_s - vehicle_s) < nearest) {
      nearest = abs(curr_s - vehicle_s);
    }
  }
  
  return nearest;
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
    curr_speed = to_equation(s_dot, dt);
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
    curr_D = to_equation(d, dt);
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
    acc = to_equation(s_d_dot, dt);
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
    curr_acc = abs(to_equation(s_d_dot, dt));
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
    curr_jerk = abs(to_equation(jerk, dt));
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
    j = to_equation(jerk, dt);
    total_jerk += abs(j*dt);
  }
  
  double jerk_per_second = total_jerk / T;
  
  return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}

double collision_cost(vector<vector<double>> trajectory, double target_s, double target_speed) {
  double nearest = nearest_approach(trajectory, target_s, target_speed);
  
  if (nearest < 2*VEHICLE_RADIUS) {
    return 1;
  } else {
    return 0;
  }
}

double buffer_cost(vector<vector<double>> trajectory, double target_s, double target_speed) {
  double nearest = nearest_approach(trajectory, target_s, target_speed);
  
  return logistic((2*VEHICLE_RADIUS) / nearest);
}
