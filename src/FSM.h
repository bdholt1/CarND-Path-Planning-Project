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

  void update_ego(double x, double y, double s, double d, double yaw, double speed, double t);
  void update_state(const std::map<int, std::vector<Prediction>> &predictions);
  void realize_state(const std::map<int, std::vector<Prediction>> &predictions);

  Vehicle& ego() {return m_ego;};
  const Vehicle& ego() const {return m_ego;};

private:
  State get_next_state(const std::map<int, std::vector<Prediction>> &predictions);
  void realize_constant_speed();
  void realize_keep_lane(const std::map<int, std::vector<Prediction>> &predictions);
  void realize_lane_change(const std::map<int, std::vector<Prediction>> &predictions, std::string direction);
  void realize_prep_lane_change(const std::map<int, std::vector<Prediction>> &predictions, std::string direction);
  std::map<int, std::vector<Prediction>> generate_predictions(int horizon);
  std::vector<Snapshot> trajectory_for_state(State state, const std::map<int, std::vector<Prediction>> &predictions, int horizon);

  Snapshot take_snapshot() const;
  void restore_state_from_snapshot(const Snapshot& snapshot);

  Vehicle m_ego;
  int m_lanes_available;
  int m_new_lane;
  double m_target_speed;
  State m_state;
};

#endif //FSM_H
