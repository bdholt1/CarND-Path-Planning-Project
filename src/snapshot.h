#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <map>
#include <vector>

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

#endif SNAPSHOT_H