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
    double curr_lead_vehicle_speed = 22.352 - 0.5;
    double target_vehicle_speed;
    vector<double> avg_scores = {0,0,0};
  
    // Decides whether to go left, right, or stay in the same lane
    // Returns amount of meters left or right to move
    int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);
  
    // Calculates if d value corresponds to left, right, or center lane
    int laneCalc(double d);
  
    // Calculates the closest vehicle either in front or behind the car in a given lane
    // Returns distance and speed of that vehicle
    vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);
  
    // Scores each lane on factors such as distance to nearest vehicle & speed
    // Returns the lane with the best score (0 left, 1 middle, 2 right)
    int laneScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

#endif /* behavior_hpp */
