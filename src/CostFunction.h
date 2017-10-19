#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <vector>

#include "snapshot.h"
#include "vehicle.h"


struct TrajectoryData {
  TrajectoryData(int current_lane,
                 int target_lane,
                 double average_speed,
                 double target_speed,
                 double closest_approach,
                 int collides)
  : _current_lane(current_lane)
  , _target_lane(target_lane)
  , _average_speed(average_speed)
  , _target_speed(target_speed)
  , _closest_approach(closest_approach)
  , _collides(collides)
  {}

  int _current_lane;
  int _target_lane;
  double _average_speed;
  double _target_speed;
  double _closest_approach;
  int _collides;
};

class CostFunction
{
public:
  double calculate_cost(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions);

private:
  double change_lane_cost(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions, const TrajectoryData &data);
  double distance_from_goal_lane(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions, const TrajectoryData &data);
  double inefficiency_cost(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions, const TrajectoryData &data);
  double collision_cost(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions, const TrajectoryData &data);
  double buffer_cost(const std::vector<Snapshot> &trajectory, const VehiclePredictions &predictions, const TrajectoryData &data);

  TrajectoryData get_helper_data(const std::vector<Snapshot >& trajectory, const VehiclePredictions &predictions);

  bool check_collision(Snapshot& snapshot, int s_previous, int s_now);
  VehiclePredictions filter_predictions_by_lane(const VehiclePredictions &predictions, int lane);
};

#endif //COST_FUNCTION_H
