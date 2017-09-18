#include <vector>
#include <math.h>

using namespace std;

struct pp_sensor_data {
  vector<double> distance_to_closest;
  vector<double> distance_to_closest_ahead;
  vector<double> vel_of_closest;
  vector<int> car_ids_closest;
  vector<int> car_ids_closest_ahead;
  vector<vector<int>> car_ids_per_line;
};

bool lane_change_is_feaseble(int current_lane,
                             double car_s,
                             double car_vel,
                             int best_lane,
                             const vector<int>& car_ids_per_lane,
                             const vector<vector<double>>& sensor_data);

pp_sensor_data preprocess_sensor_data(const vector<vector<double>>& sensor_fusion,
                                      size_t prev_path_size,
                                      double car_s);
