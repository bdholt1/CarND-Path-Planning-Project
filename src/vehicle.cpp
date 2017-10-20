
#include "vehicle.h"

#include <iostream>

using namespace std;

const double BUFFER_DISTANCE = 20.0;

Vehicle::Vehicle(int id)
: m_id(id)
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::update(double x, double y, double s, double d, double yaw, double speed, double t)
{
  m_x = x;
  m_y = y;
  m_s = s;
  m_d = d;
  m_lane = d/4;
  m_yaw = yaw;
  m_a = (m_speed - speed) / t;
  m_speed = speed;
}

vector<Prediction> Vehicle::generate_predictions(double interval, int horizon)
{
  vector<Prediction> predictions;
    for (int i = 0; i < horizon; ++i)
    {
      predictions.push_back(state_at(i*interval));
    }
    return predictions;
}

bool Vehicle::is_behind(const Vehicle& other)
{
  if (m_lane == other.m_lane && other.m_s - m_s < BUFFER_DISTANCE)
  {
    return true;
  }
  return false;
}

bool Vehicle::is_ahead(const Vehicle& other)
{
  if (m_lane == other.m_lane && m_s - other.m_s < BUFFER_DISTANCE)
  {
    return true;
  }
  return false;
}

bool Vehicle::is_close(const Vehicle& other)
{
  return false;
}

std::ostream& Vehicle::display(std::ostream& os)
{
  os << "x:     " << m_x << "\n";
  os << "y:     " << m_y << "\n";
  os << "s:     " << m_s << "\n";
  os << "d:     " << m_d << "\n";
  os << "lane:  " << m_lane << "\n";
  os << "yaw: " << m_yaw << "\n";
  os << "speed: " << m_speed << "\n";
  os << "a:     " << m_a << "\n";
  return os;
}


void Vehicle::increment(double dt)
{
  m_s += m_speed * dt + m_a * dt * dt / 2;
  m_speed += m_a * dt;
}

Prediction Vehicle::state_at(double dt)
{

  int s = m_s + m_speed * dt + m_a * dt * dt / 2;
  Prediction prediction;
  prediction.s = s;
  prediction.lane = m_lane;
  return prediction;
}

