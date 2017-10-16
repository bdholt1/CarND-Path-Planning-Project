#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

class Planner
{
public:
  Planner();

  ~Planner();


  void add_waypoint(double x, double y, double s, double d_x, double d_y);

  void update(std::vector<std::vector<double>> sensor_fusion, double ego_s, double ego_d, double ego_speed);

  void generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y);

  std::vector<double> get_x_values();
  std::vector<double> get_y_values();

private:
  std::vector<double> maps_x;
  std::vector<double> maps_y;
  std::vector<double> maps_s;
  std::vector<double> maps_dx;
  std::vector<double> maps_dy;

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
};

#endif PLANNER_H
