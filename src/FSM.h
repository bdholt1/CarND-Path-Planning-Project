#ifndef FSM_H
#define FSM_H

#include <map>
#include <string>
#include <vector>

#include "CostFunction.h"
#include "Vehicle.h"

enum State {
 CS = 0,
 KL,
 PLCL,
 LCL,
 PLCR,
 LCR
};

struct Snapshot {
  Snapshot(int lane, int s, int v, int a, State state)
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
  State _state;
};

double const SPEED_INCREMENT = .224;
double const MAX_SPEED = 49.96;
double const TIME_INTERVAL = 0.02;
double const PREDICTIONS_COUNT = 5;
double const PREDICTION_INTERVAL = 0.15;

class FSM
{
public:

  FSM(const Vehicle& ego, int num_lanes, int lane);

  void update_state(VehiclePredictions predictions);
  void realize_state(VehiclePredictions predictions);

private:

  State get_next_state(VehiclePredictions predictions);
  int _max_accel_for_lane(VehiclePredictions predictions, int lane, int s);
  void realize_constant_speed();
  void realize_keep_lane(VehiclePredictions predictions);
  void realize_lane_change(VehiclePredictions predictions, std::string direction);
  void realize_prep_lane_change(VehiclePredictions predictions, std::string direction);
  std::vector<std::vector<int> > generate_predictions(int horizon);
  std::vector<Snapshot> trajectory_for_state(State state, VehiclePredictions predictions, int horizon);
  Snapshot _snapshot() const;
  void _restore_state_from_snapshot(const Snapshot& snapshot);

  Vehicle m_ego;
  int m_num_lanes;
  int m_lane;
  State m_state;

  CostFunction m_cf;
};

#endif FSM_H
