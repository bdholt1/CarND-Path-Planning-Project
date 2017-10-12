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

using namespace std;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  struct Snapshot {
    Snapshot(int lane, int s, int v, int a, string state)
    : _lane(lane)
    , _s(s)
    , _v(v)
    , _a(a)
    , _state(state)
    {}

    int _lane;
    int _s;
    int _v;
    int _a;
    string _state;
  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt=1);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int> > > predictions);

  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  vector<vector<int> > generate_predictions(int horizon);

private:
  string _get_next_state(map<int,vector < vector<int> > > predictions);
  vector< Snapshot > _trajectory_for_state(string state, map<int,vector < vector<int> > > predictions, int horizon=5);
  Snapshot _snapshot() const;
  void _restore_state_from_snapshot(const Snapshot& snapshot);
  double _calculate_cost(vector< Vehicle::Snapshot >& trajectory, map<int,vector < vector<int> > > predictions);
};


struct TrajectoryData {
  TrajectoryData(int proposed_lane,
                 double avg_speed,
                 double max_acceleration,
                 double rms_acceleration,
                 double closest_approach,
                 double end_distance_to_goal,
                 int end_lanes_from_goal,
                 int collides)
  : _proposed_lane(proposed_lane)
  , _avg_speed(avg_speed)
  , _max_acceleration(max_acceleration)
  , _rms_acceleration(rms_acceleration)
  , _closest_approach(closest_approach)
  , _end_distance_to_goal(end_distance_to_goal)
  , _end_lanes_from_goal(end_lanes_from_goal)
  , _collides(collides)
  {}

  int _proposed_lane;
  double _avg_speed;
  double _max_acceleration;
  double _rms_acceleration;
  double _closest_approach;
  double _end_distance_to_goal;
  int _end_lanes_from_goal;
  int _collides;
};

double change_lane_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data);
double distance_from_goal_lane(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data);
double inefficiency_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data);
double collision_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data);
double buffer_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data);


TrajectoryData get_helper_data(const Vehicle& vehicle, vector< Vehicle::Snapshot >& trajectory, map<int,vector < vector<int> > > predictions);

bool check_collision(Vehicle::Snapshot& snapshot, int s_previous, int s_now);

map<int,vector < vector<int> > > filter_predictions_by_lane(map<int,vector < vector<int> > > predictions, int lane);

#endif
