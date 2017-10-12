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
  state = "CS";
  max_acceleration = -1;
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_state(map<int,vector < vector<int> > > predictions)
{
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }
    */

    state = _get_next_state(predictions);
}

string Vehicle::_get_next_state(map<int,vector < vector<int> > > predictions)
{
  vector<string> states = {"KL", "LCL", "LCR"};
  if  (lane == 0)
  {
    auto itr = std::find(states.begin(), states.end(), string("LCR"));
    if (itr != states.end()) states.erase(itr);
  }
  if (lane == lanes_available -1)
  {
    auto itr = std::find(states.begin(), states.end(), string("LCL"));
    if (itr != states.end()) states.erase(itr);
  }

  typedef std::pair<std::string, double> StateCostPair;
  vector<StateCostPair> costs;
  for (auto state : states)
  {
    map<int,vector < vector<int> > > predictions_copy(predictions);
    vector< Snapshot > trajectory = _trajectory_for_state(state, predictions_copy);
    double cost = _calculate_cost(trajectory, predictions);
    //cout << "cost for state " << state << " = " << cost << endl;
    costs.push_back(make_pair(state, cost));
  }

  struct CompareSecond
  {
    bool operator()(const StateCostPair& left, const StateCostPair& right) const
    {
       return left.second < right.second;
    }
  };
  StateCostPair best = *min_element(costs.begin(), costs.end(), CompareSecond());

  return best.first;
}

vector< Vehicle::Snapshot > Vehicle::_trajectory_for_state(string state, map<int,vector < vector<int> > > predictions, int horizon)
{
  // remember current state
  Snapshot s = _snapshot();

  // pretend to be in new proposed state
  this->state = state;
  vector< Snapshot > trajectory;
  trajectory.push_back(s);

  for (int i = 0; i < horizon; ++i)
  {
    _restore_state_from_snapshot(s);
    this->state = state;
    realize_state(predictions);
    assert(0 <= lane);
    assert(lane < lanes_available);
    increment();
    trajectory.push_back(_snapshot());

    // need to remove first prediction for each vehicle.
    for (auto it = predictions.begin(); it != predictions.end(); ++it)
    {
      auto vec = it->second;
      vec.erase(vec.begin());
    }
  }

  // restore state from snapshot
  _restore_state_from_snapshot(s);
  return trajectory;
}

Vehicle::Snapshot Vehicle::_snapshot() const
{
  return Snapshot(this->lane, this->s, this->v, this->a, this->state);
}

void Vehicle::_restore_state_from_snapshot(const Vehicle::Snapshot& snapshot)
{
  this->lane = snapshot._lane;
  this->s = snapshot._s;
  this->v = snapshot._v;
  this->a = snapshot._a;
  this->state = snapshot._state;
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
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions)
{
  /*
  Given a state, realize it by adjusting acceleration and lane.
  Note - lane changes happen instantaneously.
  */
  string state = this->state;
  if(state.compare("CS") == 0)
  {
    realize_constant_speed();
  }
  else if(state.compare("KL") == 0)
  {
    realize_keep_lane(predictions);
  }
  else if(state.compare("LCL") == 0)
  {
    realize_lane_change(predictions, "L");
  }
  else if(state.compare("LCR") == 0)
  {
    realize_lane_change(predictions, "R");
  }
  else if(state.compare("PLCL") == 0)
  {
    realize_prep_lane_change(predictions, "L");
  }
  else if(state.compare("PLCR") == 0)
  {
    realize_prep_lane_change(predictions, "R");
  }
}

void Vehicle::realize_constant_speed()
{
  a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s)
{
  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while(it != predictions.end())
  {
    int v_id = it->first;

    vector<vector<int> > v = it->second;

    if((v[0][0] == lane) && (v[0][1] > s))
    {
      in_front.push_back(v);
    }
    it++;
  }

  if(in_front.size() > 0)
  {
    int min_s = 1000;
    vector<vector<int>> leading = {};
    for(int i = 0; i < in_front.size(); i++)
    {
      if((in_front[i][0][1]-s) < min_s)
      {
        min_s = (in_front[i][0][1]-s);
        leading = in_front[i];
      }
    }
    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;
}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions)
{
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction)
{
  int delta = -1;
  if (direction.compare("L") == 0)
  {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction)
{
  int delta = -1;
  if (direction.compare("L") == 0)
  {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while(it != predictions.end())
  {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if((v[0][0] == lane) && (v[0][1] <= this->s))
    {
      at_behind.push_back(v);
    }
    it++;
  }
  if(at_behind.size() > 0)
  {
    int max_s = -1000;
    vector<vector<int> > nearest_behind = {};
    for(int i = 0; i < at_behind.size(); i++)
    {
      if((at_behind[i][0][1]) > max_s)
      {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if(delta_v != 0)
    {
      int time = -2 * delta_s/delta_v;
      int a;
      if (time == 0)
      {
        a = this->a;
      }
      else
      {
        a = delta_v/time;
      }
      if(a > this->max_acceleration)
      {
        a = this->max_acceleration;
      }
      if(a < -this->max_acceleration)
      {
        a = -this->max_acceleration;
      }
      this->a = a;
    }
    else
    {
      int my_min_acc = max(-this->max_acceleration,-delta_s);
      this->a = my_min_acc;
    }
  }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10)
{
  vector<vector<int> > predictions;
  for( int i = 0; i < horizon; i++)
  {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;

}

double Vehicle::_calculate_cost(vector< Vehicle::Snapshot >& trajectory, map<int,vector < vector<int> > > predictions)
{
  TrajectoryData trajectory_data = get_helper_data(*this, trajectory, predictions);
  double cost = 0.0;
  cost += change_lane_cost(*this, trajectory, predictions, trajectory_data);
  cost += distance_from_goal_lane(*this, trajectory, predictions, trajectory_data);
  cost += inefficiency_cost(*this, trajectory, predictions, trajectory_data);
  cost += collision_cost(*this, trajectory, predictions, trajectory_data);
  cost += buffer_cost(*this, trajectory, predictions, trajectory_data);

  return cost;
}


// priority levels for costs
const double COLLISION  = 10e6;
const double DANGER     = 10e5;
const double REACH_GOAL = 10e5;
const double COMFORT    = 10e4;
const double EFFICIENCY = 10e2;

const double DESIRED_BUFFER = 1.5;  // timesteps
const double PLANNING_HORIZON = 2;


double change_lane_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data)
{
  /**
  Penalizes lane changes AWAY from the goal lane and rewards
  lane changes TOWARDS the goal lane.
  */
  int proposed_lanes = data._end_lanes_from_goal;
  int cur_lanes = trajectory[0]._lane;
  double cost = 0.0;
  if (proposed_lanes > cur_lanes)
    cost = COMFORT;
  if (proposed_lanes < cur_lanes)
    cost = -COMFORT;
  return cost;
}

double distance_from_goal_lane(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data)
{
  double distance = std::abs(data._end_distance_to_goal);
  distance = std::max(distance, 1.0);
  double time_to_goal = distance / data._avg_speed;
  int lanes = data._end_lanes_from_goal;
  double multiplier = 5 * lanes / time_to_goal;
  double cost = multiplier * REACH_GOAL;

  return cost;
}

double inefficiency_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data)
{
  double speed = data._avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = float(diff) / target_speed;
  double multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}

double collision_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data)
{
  if (data._collides != -1)
  {
    double time_til_collision = data._collides;
    double exponent = std::pow(time_til_collision, 2);
    double multiplier = std::exp(-exponent);
    return multiplier * COLLISION;
  }
  return 0;
}

double buffer_cost(const Vehicle &vehicle, const vector<Vehicle::Snapshot> &trajectory, const map<int,vector < vector<int> > > &predictions, const TrajectoryData &data)
{
  double closest = data._closest_approach;
  if (std::abs(closest) < 1e-6)
    return 10 * DANGER;

  double timesteps_away = closest / data._avg_speed;
  if (timesteps_away > DESIRED_BUFFER)
    return 0.0;

  double multiplier = 1.0 - std::pow(timesteps_away / DESIRED_BUFFER, 2);
  return multiplier * DANGER;
}

TrajectoryData get_helper_data(const Vehicle& vehicle, vector< Vehicle::Snapshot >& trajectory, map<int,vector < vector<int> > > predictions)
{
  Vehicle::Snapshot current_snapshot = trajectory[0];
  Vehicle::Snapshot first = trajectory[1];
  Vehicle::Snapshot last = trajectory[trajectory.size()-1];
  double end_distance_to_goal = vehicle.goal_s - last._s;
  int end_lanes_from_goal = abs(vehicle.goal_lane - last._lane);
  double dt = 1.0 * trajectory.size();
  int proposed_lane = first._lane;
  double avg_speed = (last._s - current_snapshot._s) / dt;

  // initialize a bunch of variables
  vector<double> accels;
  double closest_approach = 999999;
  int collides = -1;
  Vehicle::Snapshot last_snap = trajectory[0];
  map<int,vector < vector<int> > > filtered = filter_predictions_by_lane(predictions, proposed_lane);
  double max_accel = 0.0;
  double rms_acceleration = 0.0;

  for (std::size_t i = 1; i < PLANNING_HORIZON+1; ++i)
  {
    Vehicle::Snapshot snapshot = trajectory[i];
    int lane = snapshot._lane;
    int s = snapshot._s;
    int v = snapshot._v;
    int a = snapshot._a;

    accels.push_back(a);
    for (auto& kv : filtered)
    {
      int v_id = kv.first;
      vector < vector<int> > predicted_traj = kv.second;
      vector<int> state = predicted_traj[i];
      vector<int> last_state = predicted_traj[i-1];
      bool vehicle_collides = check_collision(snapshot, last_state[1], state[1]);
      if (vehicle_collides)
        collides = i;
      int dist = std::abs(state[1] - s);
      if (dist < closest_approach)
        closest_approach = dist;
      last_snap = snapshot;
    }

    for (auto& a : accels)
    {
      max_accel = std::max(max_accel, std::abs(a));
      rms_acceleration += a*a;
    }
    rms_acceleration /= accels.size();
  }

  return TrajectoryData(proposed_lane,
                        avg_speed,
                        max_accel,
                        rms_acceleration,
                        closest_approach,
                        end_distance_to_goal,
                        end_lanes_from_goal,
                        collides);
}


bool check_collision(Vehicle::Snapshot& snapshot, int s_previous, int s_now)
{
  int s = snapshot._s;
  int v = snapshot._v;
  int v_target = s_now - s_previous;
  if (s_previous < s)
  {
    if (s_now >= s)
      return true;
    else
      return false;
  }

  if (s_previous > s)
  {
    if (s_now <= s)
      return true;
    else
      return false;
  }

  if (s_previous == s)
  {
    if (v_target > v)
      return false;
    else
      return true;
  }
}


map<int,vector < vector<int> > > filter_predictions_by_lane(map<int,vector < vector<int> > > predictions, int lane)
{
  map<int,vector < vector<int> > > filtered;

  for (auto& kv : predictions)
  {
    int v_id = kv.first;
    vector<vector<int>> predicted_traj = kv.second;
    if (predicted_traj[0][0] == lane and v_id != -1)
    {
      filtered[v_id] = predicted_traj;
    }
  }
  return filtered;
}

