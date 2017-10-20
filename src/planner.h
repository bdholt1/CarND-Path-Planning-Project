#ifndef PLANNER_H
#define PLANNER_H

#include <chrono>
#include <map>
#include <vector>

#include "FSM.h"

class Planner
{
public:
  Planner(int num_lanes);

  ~Planner();

  void add_waypoint(double x, double y, double s, double d_x, double d_y);

  void update(std::vector<std::vector<double>> sensor_fusion, double x, double y, double s, double d, double yaw, double speed);

  void generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y);

  std::vector<double> next_x_values();
  std::vector<double> next_y_values();

private:
  FSM m_fsm;
  std::map<int, Vehicle> m_vehicles;
  std::map<int, std::vector<Prediction>> m_predictions;
  std::chrono::system_clock::time_point m_time;

  std::vector<double> maps_x;
  std::vector<double> maps_y;
  std::vector<double> maps_s;
  std::vector<double> maps_dx;
  std::vector<double> maps_dy;

  std::vector<double> m_next_x_vals;
  std::vector<double> m_next_y_vals;
};

#endif //PLANNER_H
