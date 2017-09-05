#include <vector>

using namespace std;

int best_lane(int current_lane,
              const vector<double>& distance_to_closest,
              const vector<double>& distance_to_closest_ahead,
              const vector<double>& vel_of_closest,
              const vector<int>& car_ids);
