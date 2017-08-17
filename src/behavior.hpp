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
    int curr_lane;
    vector<double> s_goal;
    vector<double> d_goal;
    double T_goal;
    double target_vehicle_speed;
    double target_vehicle_s;
  
    int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);
    int laneCalc(double d);
    vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion);
    int laneScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

#endif /* behavior_hpp */
