#include <math.h>
#include <iostream>
#include <algorithm>

#include "cost-functions.h"

using namespace std;

// Constante weights for each of the cost functions
const double Cost_epsilon = 2;
const double Cost_change_of_lane = 4;
const double Cost_closest_car = 18;
const double Cost_slowest_car = 8;
const double Cost_colission = 100;

const double Max_distance = 100.0;
const double Max_velocity = 80.0;

// For each lane, the possible lanes we can transition to ( including the lane itself ).
const vector<vector<int>> LaneOptions = {
  {0, 1}, {0, 1, 2}, {1, 2}
};

// The further the lane the less costly.
double change_of_lane(int current_lane, int goal_lane) {
  return pow(current_lane - goal_lane, 2);
}

// The closest the car, the higher the cost
// returns value between 0 and 1.
double closest_car(double distance_to_car) {
  return (Max_distance - min(Max_distance, distance_to_car))/Max_distance;
}

// The slower the car, the higher the cost
// returns value between 0 and 1.
double slowest_car(double vel_of_car) {
  return (Max_velocity - min(Max_velocity, vel_of_car))/Max_velocity;
}

// Binary cost function: If there is a collision
// returns 0 or 1.
double colission(double absolute_distance_to_car) {
  bool colides = (absolute_distance_to_car < 2 * Car_radius);
  return (colides ? 1.0 : 0.0);
}

// Computes what's the best lane is depending on cost's of
// lane options.
// Returns number of lane to use: 0, 1 or 2.
int compute_best_lane(int current_lane,
                      const vector<double>& distance_to_closest,
                      const vector<double>& distance_to_closest_ahead,
                      const vector<double>& vel_of_closest) {

  auto& lane_options = LaneOptions[current_lane];
  vector<double> cost_per_lane(lane_options.size(), 0.0);
  double current_cost = 0.0;

  // cost of changing line
  for (size_t lane_index = 0; lane_index < lane_options.size(); ++lane_index) {
    int possible_lane = lane_options[lane_index];

    double cost_change_of_lane = Cost_change_of_lane * change_of_lane(current_lane, possible_lane);
    double cost_closes_car = Cost_closest_car * closest_car(distance_to_closest_ahead[possible_lane]);
    double cost_slowest_car = Cost_slowest_car * slowest_car(vel_of_closest[possible_lane]);
    double cost_colission = Cost_colission * colission(distance_to_closest[possible_lane]);

    cost_per_lane[lane_index] = cost_change_of_lane + cost_closes_car + cost_slowest_car + cost_colission;

    cout << possible_lane << " : " << cost_change_of_lane << " + " \
    << cost_closes_car << " + " \
    << cost_slowest_car << " + " \
    << cost_colission << " = " << cost_per_lane[lane_index] << ", ";

    if (current_lane == possible_lane) {
      current_cost = cost_per_lane[lane_index];
    }
  }
  cout << endl;

  auto best_lane_it = min_element(begin(cost_per_lane), end(cost_per_lane));
  size_t best_lane_index = distance(begin(cost_per_lane), best_lane_it);
  int best_lane = lane_options[best_lane_index];
  double best_cost = cost_per_lane[best_lane_index];

  int return_lane = fabs(current_cost - best_cost) < Cost_epsilon ? current_lane : best_lane;

  cout << "current_lane: " << current_lane << ", choosing lane: " << return_lane << endl;

  return return_lane;
}

