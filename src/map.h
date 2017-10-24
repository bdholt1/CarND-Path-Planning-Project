#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
  void add_waypoint(double x, double y, double s, double d_x, double d_y);
  static double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y) const;
  int NextWaypoint(double x, double y, double theta) const;
  std::vector<double> getFrenet(double x, double y, double theta) const;
  std::vector<double> getXY(double s, double d) const;

private:
  std::vector<double> maps_x;
  std::vector<double> maps_y;
  std::vector<double> maps_s;
  std::vector<double> maps_dx;
  std::vector<double> maps_dy;
};

#endif //MAP_H
