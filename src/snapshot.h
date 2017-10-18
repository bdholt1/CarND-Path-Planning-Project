#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <map>
#include <vector>

#include "vehicle.h"

using VehiclePredictions = std::map<int, std::vector <std::vector<int> > >;

enum State {
 CS = 0,
 KL,
 PLCL,
 LCL,
 PLCR,
 LCR
};

struct Snapshot {
  Snapshot(Vehicle ego, int lanes_available, int new_lane, double target_speed, State state)
  : m_ego(ego)
  , m_lanes_available(lanes_available)
  , m_new_lane(new_lane)
  , m_target_speed(target_speed)
  , m_state(state)
  {}

  Vehicle m_ego;
  int m_lanes_available;
  int m_new_lane;
  double m_target_speed;
  State m_state;
};

#endif SNAPSHOT_H