#include <vector>
#include <math.h>

using namespace std;

bool lane_change_is_feaseble(int current_lane,
                             double car_s,
                             double car_vel,
                             int best_lane,
                             const vector<int>& car_ids_per_lane,
                             const vector<vector<double>>& sensor_data);
