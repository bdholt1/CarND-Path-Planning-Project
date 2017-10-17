#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <vector>

#include "snapshot.h"
#include "vehicle.h"


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

  TrajectoryData get_helper_data(const Vehicle& vehicle, std::vector<Snapshot >& trajectory, VehiclePredictions predictions);

  VehiclePredictions filter_predictions_by_lane(VehiclePredictions predictions, int lane);
};

#endif COST_FUNCTION_H