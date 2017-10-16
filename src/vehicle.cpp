#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include <cassert>


const double BUFFER_DISTANCE = 20.0;

Vehicle::Vehicle(int id, double s, double d, double speed, double a)
: m_id(id)
, m_s(s)
, m_d(d)
, m_v(speed)
, m_a(a)
, m_lane(d/4)
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::update(double s, double d, double speed, double a)
{
  m_s = s;
  m_d = d;
  m_lane = d/4;
  m_v = speed;
  m_a = a;
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
  return Vehicle(m_id, s, m_d, v, m_a);
}

