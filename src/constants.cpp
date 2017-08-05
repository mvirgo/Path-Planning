const int MAX_JERK = 10; // m/s/s/s
const int MAX_ACCEL= 10; // m/s/s

const int EXPECTED_JERK_IN_ONE_SEC = 2; // m/s/s
const int EXPECTED_ACC_IN_ONE_SEC = 1; // m/s

const double SPEED_LIMIT = 22.352; // 50 mph in m/s
// ** Need to check the below **
const double VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection

const int MIN_D = 0; // middle yellow lines
const int MAX_D = 12; // outside of far right lane
