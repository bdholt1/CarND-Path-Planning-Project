
#include "CostFunction.h"
#include "FSM.h"

using namespace std;

FSM::FSM(const Vehicle& ego, int num_lanes)
: m_ego(ego)
, m_num_lanes(num_lanes)
, m_lane(1)
{
  m_state = State::CS;
}

void FSM::update_state(VehiclePredictions predictions)
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

    m_state = get_next_state(predictions);
}

void FSM::realize_state(VehiclePredictions predictions)
{
  /*
  Given a state, realize it by adjusting acceleration and lane.
  Note - lane changes happen instantaneously.
  */
  switch(m_state)
  {
    case State::CS  :
      realize_constant_speed();
      break;
    case State::KL  :
      realize_keep_lane(predictions);
      break;
    case State::LCL  :
      realize_lane_change(predictions, "L");
      break;
    case State::LCR  :
      realize_lane_change(predictions, "R");
      break;
    case State::PLCL  :
      realize_prep_lane_change(predictions, "L");
      break;
    case State::PLCR  :
      realize_prep_lane_change(predictions, "R");
      break;
  }
}

State FSM::get_next_state(VehiclePredictions predictions)
{
  vector<State> states;
  if (m_state == State::PLCR)
  {
    // Prepare Lane Change Right can keep lane, continue preparing or change right
    states.push_back(State::KL);
    states.push_back(State::PLCR);
    states.push_back(State::LCR);
  }
  else if (m_state == State::PLCL)
  {
    // Prepare Lane Change Left can keep lane, continue preparing or change left
    states.push_back(State::KL);
    states.push_back(State::PLCL);
    states.push_back(State::LCL);
  }
  else if (m_state == State::LCR)
  {
    // Once we're changing lane we have to go to keep lane
    states.push_back(State::KL);
  }
  else if (m_state == State::LCL)
  {
    // Once we're changing lane we have to go to keep lane
    states.push_back(State::KL);
  }
  else {
    // Keep lane can continue ...
    states.push_back(State::KL);
    if (m_lane < m_lanes_available - 1)
    {
      // ... or prepare to change right
      states.push_back(State::PLCR);
    }
    if (m_lane > 0)
    {
      // ... or prepare to change left
      states.push_back(State::PLCL);
    }
  }

  // If there is only 1 state in the list of possible states
  // then transition to it without computing costs.
  if (states.size() == 1) {
    return states[0];
  }

  typedef std::pair<State, double> StateCostPair;
  vector<StateCostPair> costs;
  for (auto state : states)
  {
    VehiclePredictions predictions_copy(predictions);
    vector<Snapshot> trajectory = _trajectory_for_state(state, predictions_copy);
    CostFunction cf;
    double cost = cf.calculate_cost(trajectory, predictions);
    cout << "cost for state " << state << " = " << cost << endl;
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

int FSM::max_accel_for_lane(VehiclePredictions predictions, int lane, int s)
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

void FSM::realize_constant_speed()
{
}

void FSM::realize_keep_lane(VehiclePredictions predictions)
{
  this->a = _max_accel_for_lane(predictions, this->m_lane, this->s);
}

void FSM::realize_lane_change(VehiclePredictions predictions, string direction)
{
  int delta = 1;
  if (direction.compare("L") == 0)
  {
    delta = -1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void FSM::realize_prep_lane_change(VehiclePredictions predictions, string direction)
{
  int delta = 1;
  if (direction.compare("L") == 0)
  {
    delta = -1;
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

vector<vector<int> > FSM::generate_predictions(int horizon = 10)
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

vector< FSM::Snapshot > FSM::_trajectory_for_state(State state, VehiclePredictions predictions, int horizon)
{
  // remember current state
  Snapshot s = _snapshot();

  vector< Snapshot > trajectory;
  trajectory.push_back(s);

  for (int i = 0; i < horizon; ++i)
  {
    _restore_state_from_snapshot(s);
    // pretend to be in new proposed state
    this->state = state;
    realize_state(predictions);
    assert(0 <= lane);
    assert(lane < lanes_available);
    increment();
    trajectory.push_back(_snapshot());

    // need to remove first prediction for each vehicle.
    for (auto pair: predictions)
    {
      auto vec = pair.second;
      vec.erase(vec.begin());
    }
  }

  // restore state from snapshot
  _restore_state_from_snapshot(s);
  return trajectory;
}

FSM::Snapshot FSM::_snapshot() const
{
  return Snapshot(this->lane, this->s, this->v, this->a, this->m_state);
}

void FSM::_restore_state_from_snapshot(const FSM::Snapshot& snapshot)
{
  this->lane = snapshot._lane;
  this->s = snapshot._s;
  this->v = snapshot._v;
  this->a = snapshot._a;
  this->state = snapshot._state;
}
