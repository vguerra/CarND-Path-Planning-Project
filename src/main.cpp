#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "cost-functions.h"
#include "helpers.h"
#include "paths.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0; //49.5; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
//          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          size_t prev_path_size = previous_path_x.size();

          if (prev_path_size > 0) {
            car_s = end_path_s;
          }

          // here we take sensor fussion data
          // to build up relevant information for computing cost of each line.
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

          int best_lane = compute_best_lane(lane,
                                            distance_to_closest,
                                            distance_to_closest_ahead,
                                            vel_of_closest);

          if (lane_change_is_feaseble(lane,
                                      car_s,
                                      ref_vel,
                                      best_lane,
                                      car_ids_per_line[best_lane],
                                      sensor_fusion)) {
            lane = best_lane;
          }


          // checking if we need to slow down or speed up.
          bool too_close = false;
          if (car_ids_closest_ahead[lane] != -1) {
            int other_car_id = car_ids_closest_ahead[lane];
            double check_speed = vel_of_closest[lane];
            double check_car_s = sensor_fusion[other_car_id][5];

            check_car_s += (double)prev_path_size * 0.02 * check_speed;

            if (check_car_s > car_s && (check_car_s - car_s) < 20) {
              too_close = true;
            }
          }

          if (too_close) {
            ref_vel -= 0.224;
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Taking into account points from previous
          // path ( 2 points )

          // In case we receive less than 2 points,
          // we infeer previous car position.
          if (prev_path_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          } else {
            // we have enough points, we take only the last 2.

            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // Generating extra 3 points that are the base for the spline
          for (int i = 0; i < 3; ++i) {
            vector<double> next_wp = getXY(car_s + 30*(i + 1), (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }

          // Converting points to local car coordinates
          for (size_t i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          for (size_t i = 0; i < prev_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Extracting points of the splice, using a distance
          // based on ref_vel that will define car's velocity on simulator.
          double target_x = 30.0;
          double target_y = s(target_x);

          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          double N = target_dist/(0.02 * ref_vel / 2.24);


          // Computing next points to put into our trajectory.
          for (size_t i = 1; i <= 30 - prev_path_size; i++) {
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            // converting back points to map's coordinates.
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































