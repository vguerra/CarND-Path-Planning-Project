
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

