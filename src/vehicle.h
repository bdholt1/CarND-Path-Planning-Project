#ifndef VEHICLE_H
#define VEHICLE_H

#include <ostream>
#include <vector>

class Vehicle
{
public:
  Vehicle(int id);
  virtual ~Vehicle();

  int get_lane() const { return m_lane;};
  void set_lane(int lane) {m_lane = lane;};
  double get_speed() const {return m_v;};
  double get_s() const {return m_s;};

  void update(double s, double d, double speed, double t);
  std::vector<Vehicle> generate_predictions(double interval, int horizon);

  void increment(double dt);
  Vehicle state_at(double t);
  std::ostream& display(std::ostream& os);

  bool is_behind(const Vehicle& other);
  bool is_ahead(const Vehicle& other);
  bool is_close(const Vehicle& other);

private:
  int m_id;
  double m_s;
  double m_d;
  double m_v;
  double m_a;
  int m_lane;
};

#endif //VEHICLE_H
