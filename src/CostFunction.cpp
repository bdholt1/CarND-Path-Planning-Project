
#include "CostFunction.h"

#include <cmath>
#include <iostream>

// priority levels for costs
const double COLLISION  = 10e6;
const double DANGER     = 10e5;
const double REACH_GOAL = 10e5;
const double COMFORT    = 10e4;
const double EFFICIENCY = 10e2;

const double DESIRED_BUFFER = 1.5;  // timesteps
const double PLANNING_HORIZON = 5;
const double INTERVAL = 0.02;

using namespace std;

double CostFunction::calculate_cost(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions)
{
  TrajectoryData trajectory_data = get_helper_data(trajectory, predictions);
  double cost = 0.0;
  cost += change_lane_cost(trajectory, predictions, trajectory_data);
  cost += distance_from_goal_lane(trajectory, predictions, trajectory_data);
  cost += inefficiency_cost(trajectory, predictions, trajectory_data);
  cost += collision_cost(trajectory, predictions, trajectory_data);
  cost += buffer_cost(trajectory, predictions, trajectory_data);

  return cost;
}

double CostFunction::change_lane_cost(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions, const TrajectoryData &data)
{
  // Penalises being in the wrong lane
  double cost = 0.0;
  if (data._target_lane != data._current_lane)
  {
    cost = COMFORT;
  }
  cout << "lane change cost = " << cost << endl;
  return cost;
}

double CostFunction::inefficiency_cost(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions, const TrajectoryData &data)
{
  double speed = data._average_speed;
  double target_speed = data._target_speed;
  double diff = target_speed - speed;
  double pct = float(diff) / target_speed;
  double multiplier = pct * pct;
  double cost = multiplier * EFFICIENCY;
  cout << "inefficiency cost = " << cost << endl;
  return cost;
}

double CostFunction::collision_cost(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions, const TrajectoryData &data)
{
  double cost = 0.0;
  if (data._collides != -1)
  {
    double time_til_collision = data._collides;
    double exponent = std::pow(time_til_collision, 2);
    double multiplier = std::exp(-exponent);
    cost = multiplier * COLLISION;
    cout << "collision cost = " << cost << endl;
  }
  return cost;
}

double CostFunction::buffer_cost(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions, const TrajectoryData &data)
{
  double closest = data._closest_approach;
  double cost = 0.0;
  if (std::abs(closest) < 5)
  {
    cost = 10 * DANGER;
    cout << "buffer cost = " << cost << endl;
    return cost;
  }

  double timesteps_away = closest / data._average_speed;
  if (timesteps_away > DESIRED_BUFFER)
  {
    cout << "buffer cost = " << cost << endl;
    return cost;
  }

  double multiplier = 1.0 - std::pow(timesteps_away / DESIRED_BUFFER, 2);
  cost = multiplier * DANGER;
  cout << "buffer cost = " << cost << endl;
  return cost;
}

TrajectoryData CostFunction::get_helper_data(const std::vector<Snapshot> &trajectory, const std::map<int, std::vector<Prediction>> &predictions)
{
  Snapshot current_snapshot = trajectory[0];
  Snapshot first = trajectory[1];
  Snapshot last = trajectory[trajectory.size()-1];

  double dt = trajectory.size() * INTERVAL;
  int current_lane = first.m_new_lane;
  int target_lane = last.m_new_lane;
  double target_speed = last.m_target_speed;
  double average_speed = (last.m_ego.speed()*dt - current_snapshot.m_ego.speed()) / dt;

  // initialize a bunch of variables
  double closest_approach = 999999;
  int collides = -1;
  std::map<int, std::vector<Prediction>> current_lane_filtered = filter_predictions_by_lane(predictions, current_lane);
  std::map<int, std::vector<Prediction>> target_lane_filtered = filter_predictions_by_lane(predictions, target_lane);


  for (std::size_t i = 1; i < PLANNING_HORIZON+1; ++i)
  {
    Snapshot snapshot = trajectory[i];
    int lane = snapshot.m_new_lane;

    for (auto& kv : target_lane_filtered)
    {
      int v_id = kv.first;
      vector<Prediction> predicted_traj = kv.second;
      Prediction pred = predicted_traj[i];
      Prediction prev_pred = predicted_traj[i-1];
      bool vehicle_collides = check_collision(snapshot, prev_pred.s, pred.s);
      if (vehicle_collides)
        collides = i;
      int dist = std::abs(pred.s - current_snapshot.m_ego.s());
      if (dist < closest_approach)
        closest_approach = dist;
    }


  }

  return TrajectoryData(current_lane,
                        target_lane,
                        average_speed,
                        target_speed,
                        closest_approach,
                        collides);
}


bool CostFunction::check_collision(Snapshot& snapshot, int s_previous, int s_now)
{
  int s = snapshot.m_ego.s();
  int v = snapshot.m_ego.speed();
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


std::map<int, std::vector<Prediction>> CostFunction::filter_predictions_by_lane(const std::map<int, std::vector<Prediction>> &predictions, int lane)
{
  std::map<int, std::vector<Prediction>> filtered;

  for (auto& kv : predictions)
  {
    int v_id = kv.first;
    vector<Prediction> predicted_traj = kv.second;
    if (predicted_traj[0].lane == lane and v_id != -1)
    {
      filtered[v_id] = predicted_traj;
    }
  }
  return filtered;
}

