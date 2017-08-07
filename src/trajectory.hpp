//
//  trajectory.hpp
//  
//
//  Created by Michael on 8/6/17.
//
//

#ifndef trajectory_hpp
#define trajectory_hpp

#include <vector>

using namespace std;

/*
 Calculate the Jerk Minimizing Trajectory that connects the initial state
 to the final state in time T.
 
 INPUTS
 
 start - the vehicles start location given as a length three array
 corresponding to initial values of [s, s_dot, s_double_dot]
 
 end   - the desired end state for vehicle. Like "start" this is a
 length three array.
 
 T     - The duration, in seconds, over which this maneuver should occur.
 
 OUTPUT
 an array of length 6, each value corresponding to a coefficent in the polynomial
 s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
 
 */
vector<double> JMT(vector<double> start, vector <double> end, double T);

vector<double> trajectory(double s, double d, double speed, vector<vector<double>> sensor_fusion);

#endif /* trajectory_hpp */
