
#include "CostFunction.h"
#include "FSM.h"

#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;

FSM::FSM(const Vehicle& ego, int lanes_available)
: m_ego(ego)
, m_lanes_available(lanes_available)
{
  m_state = State::CS;
}

void FSM::update_ego(double x, double y, double s, double d, double yaw, double speed, double t)
{
  m_ego.update(x, y, s, d, yaw, speed, t);
}

void FSM::update_state(const std::map<int, std::vector<Prediction>> &predictions)
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

void FSM::realize_state(const std::map<int, std::vector<Prediction>> &predictions)
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

State FSM::get_next_state(const std::map<int, std::vector<Prediction>> &predictions)
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
    if (ego().lane() < m_lanes_available - 1)
    {
      // ... or prepare to change right
      states.push_back(State::PLCR);
    }
    if (ego().lane() > 0)
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

  typedef pair<State, double> StateCostPair;
  vector<StateCostPair> costs;
  for (auto state : states)
  {
    std::map<int, std::vector<Prediction>> predictions_copy(predictions);
    vector<Snapshot> trajectory = trajectory_for_state(state, predictions_copy, 20);
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

void FSM::realize_constant_speed()
{
}

void FSM::realize_keep_lane(const std::map<int, std::vector<Prediction>> &predictions)
{
  // stay in the same lane
  m_new_lane = ego().lane();


}

void FSM::realize_lane_change(const std::map<int, std::vector<Prediction>> &predictions, string direction)
{
  int delta = 1;
  if (direction.compare("L") == 0)
  {
    delta = -1;
  }
  int lane = m_ego.lane() + delta;
  // realise the lane change
  m_ego.set_lane(lane);
  int m_new_lane = lane;


}

void FSM::realize_prep_lane_change(const std::map<int, std::vector<Prediction>> &predictions, string direction)
{
  int delta = 1;
  if (direction.compare("L") == 0)
  {
    delta = -1;
  }
  int lane = m_ego.lane() + delta;
  // prepare for lane change
  int m_new_lane = lane;

}


vector< Snapshot > FSM::trajectory_for_state(State state, const std::map<int, std::vector<Prediction>> &predictions, int horizon)
{
  // remember current state
  Snapshot s = take_snapshot();

  vector< Snapshot > trajectory;
  trajectory.push_back(s);

  for (int i = 0; i < horizon; ++i)
  {
    restore_state_from_snapshot(s);
    // pretend to be in new proposed state
    m_state = state;
    realize_state(predictions);
    assert(0 <= ego().lane());
    assert(ego().lane() < m_lanes_available);
    m_ego.increment(i * INTERVAL);
    trajectory.push_back(take_snapshot());

    // need to remove first prediction for each vehicle.
    for (auto pair: predictions)
    {
      auto vec = pair.second;
      vec.erase(vec.begin());
    }
  }

  // restore state from snapshot
  restore_state_from_snapshot(s);
  return trajectory;
}

Snapshot FSM::take_snapshot() const
{
  return Snapshot(m_ego, m_lanes_available, m_new_lane, m_target_speed, m_state);
}

void FSM::restore_state_from_snapshot(const Snapshot& snapshot)
{
  m_ego = snapshot.m_ego;
  m_lanes_available = snapshot.m_lanes_available;
  m_new_lane = snapshot.m_new_lane;
  m_target_speed = snapshot.m_target_speed;
  m_state = snapshot.m_state;
}
