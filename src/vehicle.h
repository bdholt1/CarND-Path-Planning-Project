#ifndef VEHICLE_H
#define VEHICLE_H

#include <ostream>
#include <vector>

struct Prediction
{
  int s;
  int lane;
};

class Vehicle
{
public:
  Vehicle(int id);
  Vehicle(const Vehicle& rhs);
  Vehicle& operator=(const Vehicle& rhs);
  virtual ~Vehicle();

  double x() const {return m_x;};
  double y() const {return m_y;};
  double d() const {return m_d;};
  double s() const {return m_s;};
  double yaw() const {return m_yaw;};
  double speed() const {return m_speed;};

  int lane() const { return m_lane;};
  void set_lane(int lane) {m_lane = lane;};

  void update(double x, double y, double s, double d, double yaw, double speed, double t);
  std::vector<Prediction> generate_predictions(double interval, int horizon);

  void increment(double dt);
  Prediction state_at(double t);
  std::ostream& display(std::ostream& os);

  bool is_behind(const Vehicle& other);
  bool is_ahead(const Vehicle& other);
  bool is_close(const Vehicle& other);

private:
  int m_id;
  double m_x;
  double m_y;
  double m_s;
  double m_d;
  double m_yaw;
  double m_speed;
  double m_a;
  int m_lane;
};

#endif //VEHICLE_H
