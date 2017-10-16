#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include <cassert>


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a)
{
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = State::CS;
  max_acceleration = -1;
}

Vehicle::~Vehicle()
{
}





void Vehicle::configure(vector<int> road_data)
{
  /*
  Called by simulator before simulation begins. Sets various
  parameters which will impact the ego vehicle.
  */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

string Vehicle::display()
{
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt)
{
  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t)
{
  /*
  Predicts state of vehicle in t seconds (assuming constant acceleration)
  */
  int s = this->s + this->v * t + this->a * t * t / 2;
  int v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time)
{
  /*
  Simple collision detection.
  */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps)
{
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps+1; t++)
  {
    if( collides_with(other, t) )
    {
      collider_temp.collision = true;
      collider_temp.time = t;
      cout << "Collision detected with vehicle " << endl;
      return collider_temp;
    }
  }

  return collider_temp;
}




