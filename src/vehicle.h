#ifndef VEHICLE_H
#define VEHICLE_H

#include <ostream>

class Vehicle
{
public:
  Vehicle(int id, double s, double d, double speed, double a);

  virtual ~Vehicle();

  int get_current_lane() const { return m_lane;};
  double get_current_speed() const {return m_v;};
  void update(double s, double d, double speed, double a);

  bool is_behind(const Vehicle& other);
  bool is_ahead(const Vehicle& other);
  bool is_close(const Vehicle& other);

private:
  std::ostream& display(std::ostream& os);
  void increment(double dt);
  Vehicle state_at(double t);

  int m_id;
  double m_s;
  double m_d;
  double m_v;
  double m_a;
  int m_lane;
};

#endif
