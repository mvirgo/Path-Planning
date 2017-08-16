const int N_SAMPLES = 10;
const vector<double> SIGMA_S = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const vector<double> SIGMA_D = {1.0, 1.0, 1.0};
const double SIGMA_T = 1.0;

const int MAX_JERK = 10; // m/s/s/s
const int MAX_ACCEL= 10; // m/s/s

const int EXPECTED_JERK_IN_ONE_SEC = 2; // m/s/s
const int EXPECTED_ACC_IN_ONE_SEC = 1; // m/s

const double SPEED_LIMIT = 22.352; // 50 mph in m/s
const double SPEED_CONV = 2.23694; // convert speeds for easier calc
const double VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection

const int MIN_D = 0; // middle yellow lines
const int MAX_D = 12; // outside of far right lane
