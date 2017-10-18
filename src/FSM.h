#ifndef FSM_H
#define FSM_H

#include "snapshot.h"
#include "vehicle.h"

double const SPEED_CHANGE = .224;
double const SPEED_LIMIT = 49.5;
double const NUM_PREDICTIONS = 5; // Number of predictions in the future
double const INTERVAL = 0.02; // Interval between iterations of the simulator

class FSM
{
public:

  FSM(const Vehicle& ego, int num_lanes);

  void update_ego(double ego_s, double ego_d, double ego_speed, double t);
  void update_state(VehiclePredictions predictions);
  void realize_state(VehiclePredictions predictions);

  int ego_lane() const {return m_ego.get_lane();};
  double ego_speed() const {return m_ego.get_speed();};

private:

  State get_next_state(VehiclePredictions predictions);
  void realize_constant_speed();
  void realize_keep_lane(VehiclePredictions predictions);
  void realize_lane_change(VehiclePredictions predictions, std::string direction);
  void realize_prep_lane_change(VehiclePredictions predictions, std::string direction);
  std::vector<std::vector<int> > generate_predictions(int horizon);
  std::vector<Snapshot> trajectory_for_state(State state, VehiclePredictions predictions, int horizon);

  Snapshot take_snapshot() const;
  void restore_state_from_snapshot(const Snapshot& snapshot);

  Vehicle m_ego;
  int m_lanes_available;
  int m_new_lane;
  double m_target_speed;
  State m_state;
};

#endif FSM_H