#include <vector>

using namespace std;

const double Car_radius = 1.5;

int compute_best_lane(int current_lane,
                      const vector<double>& distance_to_closest,
                      const vector<double>& distance_to_closest_ahead,
                      const vector<double>& vel_of_closest);
