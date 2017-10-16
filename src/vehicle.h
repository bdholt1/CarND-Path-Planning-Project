#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>


typedef std::map<int, std::vector <std::vector<int> > > predictions VehiclePredictions;

class Vehicle {

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };




  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();


  void configure(vector<int> road_data);

  int get_current_lane() const { return lane;};
  double get_current_speed() const {return v;};


private:

  std::string display();

  void increment(double dt=1);

  Predict state_at(double t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  int L = 1;
  int preferred_buffer = 50; // impacts "keep lane" behavior.
  int lane;
  int s;
  int v;
  int a;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;


};



#endif
