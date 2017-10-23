#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>


#include "trajectory.h"
#include "json.hpp"
#include "spline.h"
#include "map.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  string map_file_ = "../data/highway_map.csv";
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  Map map;
  
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
    map.add_waypoint(x, y, s, d_x, d_y);
  }

  //start in lane 1;
  int lane = 1;
  int lane_change_wp = 0;

  h.onMessage([&map,&lane,&lane_change_wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


            double ref_vel = 49.5; //mph

	    int prev_size = previous_path_x.size();

	    if (prev_size > 0)
	    {
	      car_s = end_path_s;
	    }
	    
            int next_wp = -1;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);


            if(prev_size < 2)
            {
	      next_wp = map.NextWaypoint(ref_x, ref_y, ref_yaw);
            }
            else
            {
                ref_x = previous_path_x[prev_size-1];
                double ref_x_prev = previous_path_x[prev_size-2];
                ref_y = previous_path_y[prev_size-1];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
                next_wp = map.NextWaypoint(ref_x, ref_y, ref_yaw);

                car_s = end_path_s;

                car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
            }


            vector<double> ptsx;
            vector<double> ptsy;

            if (prev_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            else
            {
              ptsx.push_back(previous_path_x[prev_size-2]);
              ptsx.push_back(previous_path_x[prev_size-1]);

              ptsy.push_back(previous_path_y[prev_size-2]);
              ptsy.push_back(previous_path_y[prev_size-1]);

              ref_x = previous_path_x[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              ref_y = previous_path_y[prev_size-1];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

              car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
            }

            vector<double> next_wp0 = map.getXY(car_s + 30, 2 + 4*lane);
            vector<double> next_wp1 = map.getXY(car_s + 60, 2 + 4*lane);
            vector<double> next_wp2 = map.getXY(car_s + 90, 2 + 4*lane);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // transform the points into the car's local coordinate system
            for (int i = 0; i < ptsx.size(); i++ )
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
              ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);

            }

            tk::spline s;
            s.set_points(ptsx,ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_start = 0;

            for (int i = 0; i < 50 - prev_size; i++)
            {
              if(ref_vel > car_speed)
              {
                car_speed+=.224;
              }
              else if(ref_vel < car_speed)
              {
                car_speed-=.224;
              }

              double N = target_dist*2.24/(.02*car_speed);
              double x_point = x_start+(target_x)/N;
              double y_point = s(x_point);

              x_start = x_point;

              double tmp_x = x_point;
              double tmp_y = y_point;

              // transform the points back into real-world coordinate frame
              x_point = tmp_x * cos(ref_yaw) - tmp_y * sin(ref_yaw);
              y_point = tmp_x * sin(ref_yaw) + tmp_y * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;
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
