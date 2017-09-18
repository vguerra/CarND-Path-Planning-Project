
#include <iostream>

#include "cost-functions.h"
#include "paths.h"

using namespace std;

bool lane_change_is_feaseble(int current_lane,
                             double car_s,
                             double car_vel,
                             int best_lane,
                             const vector<int>& car_ids_per_lane,
                             const vector<vector<double>>& sensor_data) {

  // no change of lane
  if (current_lane == best_lane)
    return true;

  // no car that could cause a collision
  if (car_ids_per_lane.empty())
    return true;

  for (size_t car_index = 0; car_index < car_ids_per_lane.size(); ++car_index) {
    double this_car_s = car_s;

    int car_id = car_ids_per_lane[car_index];
    vector<double> car_data = sensor_data[car_id];

    int other_car_s = car_data[5];
    double vx = car_data[3];
    double vy = car_data[4];
    double other_car_vel = sqrt(vx*vx + vy*vy);

    // we check for collisions in the future
    for (int i = 0; i < 50; i++) {
      other_car_s += (double)i * 0.02 * other_car_vel;
      this_car_s += (double)i * 0.02 * car_vel;

      if (fabs(other_car_s - this_car_s) <  2 * Car_radius) {
        cout << "COLISSION IN THE FUTUREEE!!!!!! ... ABORTTT!!!! " << i << endl;
        return false;
      }
    }
  }

  cout << "THERE WAS A CAR BUT NO DANGER! ;) " << endl;

  return true;
}

pp_sensor_data preprocess_sensor_data(const vector<vector<double>>& sensor_fusion,
                                      size_t prev_path_size,
                                      double car_s) {

  vector<double> distance_to_closest(3, std::numeric_limits<double>::max());
  vector<double> distance_to_closest_ahead(3, std::numeric_limits<double>::max());
  vector<double> vel_of_closest(3, std::numeric_limits<double>::max());
  vector<int> car_ids_closest(3, -1);
  vector<int> car_ids_closest_ahead(3, -1);
  vector<vector<int>> car_ids_per_line(3, vector<int>());

  for (size_t i = 0; i < sensor_fusion.size(); ++i) {
    int other_car_id = sensor_fusion[i][0];
    float other_d = sensor_fusion[i][6];
    int other_car_lane = (int)(other_d / 4.0);

    if (other_car_lane < 0 || other_car_lane > 2) {
      continue;
    }

    car_ids_per_line[other_car_lane].push_back(other_car_id);

    int other_car_s = sensor_fusion[i][5];

    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double other_car_vel = sqrt(vx*vx + vy*vy);

    // update car position to current state
    other_car_s += (double)prev_path_size * 0.02 * other_car_vel;

    double diff_in_s = other_car_s - car_s;
    double diff_abs = fabs(diff_in_s);

    if (diff_abs < distance_to_closest[other_car_lane]) {
      distance_to_closest[other_car_lane] = diff_abs;
      car_ids_closest[other_car_lane] = other_car_id;
    }

    if (diff_in_s < 0.0) {
      continue;
    }

    if (diff_in_s < distance_to_closest_ahead[other_car_lane]) {
      distance_to_closest_ahead[other_car_lane] = diff_in_s;
      vel_of_closest[other_car_lane] = other_car_vel;
      car_ids_closest_ahead[other_car_lane] = other_car_id;
    }
  }

  return {
    distance_to_closest,
    distance_to_closest_ahead,
    vel_of_closest,
    car_ids_closest,
    car_ids_closest_ahead,
    car_ids_per_line,
  };
}

