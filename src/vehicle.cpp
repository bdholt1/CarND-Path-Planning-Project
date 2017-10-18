
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

void Vehicle::update(double s, double d, double speed, double t)
{
  m_s = s;
  m_d = d;
  m_lane = d/4;
  m_a = (m_v - speed) / t;
  m_v = speed;
}

vector<Vehicle> Vehicle::generate_predictions(double interval, int horizon)
{
  vector<Vehicle> predictions;
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
  os << "s:     " << m_s << "\n";
  os << "d:     " << m_d << "\n";
  os << "lane:  " << m_lane << "\n";
  os << "speed: " << m_v << "\n";
  os << "a:     " << m_a << "\n";
  return os;
}


void Vehicle::increment(double dt)
{
  m_s += m_v * dt;
  m_v += m_a * dt;
}

Vehicle Vehicle::state_at(double t)
{

  int s = m_s + m_v * t + m_a * t * t / 2;
  int v = m_v + m_a * t;
  Vehicle vehicle(m_id);
  vehicle.update(s, m_d, v, t);
  return vehicle;
}

