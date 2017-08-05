//
//  behavior.hpp
//  
//
//  Created by Michael on 8/5/17.
//
//

#ifndef behavior_hpp
#define behavior_hpp

#include <vector>
#include <string>

using namespace std;

class BehaviorPlanner {
  public:
    string lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);
    int laneCalc(double d);
    double closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion);
    int laneScore(double s, double d, vector<vector<double>> sensor_fusion);
};

string BehaviorPlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = laneCalc(d);
  bool blocked = false;
  int closest = closestVehicle(s, lane, sensor_fusion);
  // check if blocked
  if (closest > 100) {
    return "Keep";
  } else if (lane == 0) {
    // ***Need to change to also score b/w right and keep***
    return "Prepare Right";
  } else if (lane == 2) {
    // ***Need to change to also score b/w left and keep***
    return "Prepare Left";
  } else {
    // ***Need to change to score, then select from all options***
    return "Keep";
  }
}

int BehaviorPlanner::laneCalc(double d) {
  // Check which lane the d-value comes from
  // Left is 0, middle is 1, right is 2
  int lane;
  if (d < 4) {
    lane = 0;
  } else if (d < 8) {
    lane = 1;
  } else {
    lane = 2;
  }
  return lane;
}

double BehaviorPlanner::closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion) {
  // *** Need to determine whether returning closest or dist
  double dist = 10000;
  int vehicle_lane;
  //int closest;
  double vehicle_s;
  double vehicle_d;
  
  for(int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
    vehicle_s = sensor_fusion[vehicle][5];
    vehicle_d = sensor_fusion[vehicle][6];
    vehicle_lane = laneCalc(vehicle_d);
    
    if (vehicle_lane == lane) { // if same lane
      if (vehicle_s > s) { // and ahead of vehicle
        if (vehicle_s - s < dist) {
          dist = vehicle_s - s;
          //closest = vehicle;
        }
      }
    }
  }
  return dist;
}

int BehaviorPlanner::laneScore(double s, double d, vector<vector<double>> sensor_fusion) {
  // *** To score lanes based on best plan ***
  return 0;
}

#endif /* behavior_hpp */
