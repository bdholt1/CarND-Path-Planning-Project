
#include "CostFunction.h"

// priority levels for costs
const double COLLISION  = 10e6;
const double DANGER     = 10e5;
const double REACH_GOAL = 10e5;
const double COMFORT    = 10e4;
const double EFFICIENCY = 10e2;

const double DESIRED_BUFFER = 1.5;  // timesteps
const double PLANNING_HORIZON = 5;


double CostFunction::calculate_cost(const std::vector<Snapshot> &trajectory, const VehiclePrediction &predictions)
{
  TrajectoryData trajectory_data = get_helper_data(*this, trajectory, predictions);
  double cost = 0.0;
  //cost += change_lane_cost(*this, trajectory, predictions, trajectory_data);
  cost += distance_from_goal_lane(trajectory, predictions, trajectory_data);
  cost += inefficiency_cost(trajectory, predictions, trajectory_data);
  cost += collision_cost(trajectory, predictions, trajectory_data);
  cost += buffer_cost(trajectory, predictions, trajectory_data);

  return cost;
}

double CostFunction::change_lane_cost(const vector<Snapshot> &trajectory, const VehiclePrediction &predictions, const TrajectoryData &data)
{
  /**
  Penalizes lane changes AWAY from the goal lane and rewards
  lane changes TOWARDS the goal lane.
  */
  int proposed_lanes = data._end_lanes_from_goal;
  int cur_lanes = trajectory[0]._lane;
  double cost = 0.0;
  if (proposed_lanes > cur_lanes)
    cost = COMFORT;
  if (proposed_lanes < cur_lanes)
    cost = -COMFORT;
  cout << "lane change cost = " << cost << endl;
  return cost;
}

double CostFunction::distance_from_goal_lane(const vector<Snapshot> &trajectory, const VehiclePrediction &predictions, const TrajectoryData &data)
{
  double distance = std::abs(data._end_distance_to_goal);
  distance = std::max(distance, 1.0);
  double time_to_goal = distance / data._avg_speed;
  int lanes = data._end_lanes_from_goal;
  double multiplier = 5 * lanes / time_to_goal;
  double cost = multiplier * REACH_GOAL;
  cout << "distance to goal cost = " << cost << endl;
  return cost;
}

double CostFunction::inefficiency_cost(const vector<Snapshot> &trajectory, const VehiclePrediction &predictions, const TrajectoryData &data)
{
  double speed = data._avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = float(diff) / target_speed;
  double multiplier = pct * pct;
  double cost = multiplier * EFFICIENCY;
  cout << "inefficiency cost = " << cost << endl;
  return cost;
}

double CostFunction::collision_cost(const vector<Vehicle::Snapshot> &trajectory, const VehiclePrediction &predictions, const TrajectoryData &data)
{
  if (data._collides != -1)
  {
    double time_til_collision = data._collides;
    double exponent = std::pow(time_til_collision, 2);
    double multiplier = std::exp(-exponent);
    return multiplier * COLLISION;
    double cost = multiplier * COLLISION;
    cout << "collision cost = " << cost << endl;
  }
  return 0;
}

double CostFunction::buffer_cost(const vector<Snapshot> &trajectory, const VehiclePrediction &predictions, const TrajectoryData &data)
{
  double closest = data._closest_approach;
  double cost = 0.0;
  if (std::abs(closest) < 1e-6)
  {
    cost = 10 * DANGER;
    cout << "buffer cost = " << cost << endl;
    return cost;
  }

  double timesteps_away = closest / data._avg_speed;
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

TrajectoryData get_helper_data(const std::vector<Snapshot> &trajectory, const VehiclePrediction &predictions)
{
  Vehicle::Snapshot current_snapshot = trajectory[0];
  Vehicle::Snapshot first = trajectory[1];
  Vehicle::Snapshot last = trajectory[trajectory.size()-1];
  double end_distance_to_goal = vehicle.goal_s - last._s;
  int end_lanes_from_goal = abs(vehicle.goal_lane - last._lane);
  double dt = 1.0 * trajectory.size();
  int proposed_lane = first._lane;
  double avg_speed = (last._s - current_snapshot._s) / dt;

  // initialize a bunch of variables
  vector<double> accels;
  double closest_approach = 999999;
  int collides = -1;
  Vehicle::Snapshot last_snap = trajectory[0];
  map<int,vector < vector<int> > > filtered = filter_predictions_by_lane(predictions, proposed_lane);
  double max_accel = 0.0;
  double rms_acceleration = 0.0;

  for (std::size_t i = 1; i < PLANNING_HORIZON+1; ++i)
  {
    Vehicle::Snapshot snapshot = trajectory[i];
    int lane = snapshot._lane;
    int s = snapshot._s;
    int v = snapshot._v;
    int a = snapshot._a;

    accels.push_back(a);
    for (auto& kv : filtered)
    {
      int v_id = kv.first;
      vector < vector<int> > predicted_traj = kv.second;
      vector<int> state = predicted_traj[i];
      vector<int> last_state = predicted_traj[i-1];
      bool vehicle_collides = check_collision(snapshot, last_state[1], state[1]);
      if (vehicle_collides)
        collides = i;
      int dist = std::abs(state[1] - s);
      if (dist < closest_approach)
        closest_approach = dist;
      last_snap = snapshot;
    }

    for (auto& a : accels)
    {
      max_accel = std::max(max_accel, std::abs(a));
      rms_acceleration += a*a;
    }
    rms_acceleration /= accels.size();
  }

  return TrajectoryData(proposed_lane,
                        avg_speed,
                        max_accel,
                        rms_acceleration,
                        closest_approach,
                        end_distance_to_goal,
                        end_lanes_from_goal,
                        collides);
}


bool check_collision(Snapshot& snapshot, int s_previous, int s_now)
{
  int s = snapshot._s;
  int v = snapshot._v;
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


VehiclePrediction filter_predictions_by_lane(const VehiclePrediction &predictions, int lane)
{
  VehiclePrediction filtered;

  for (auto& kv : predictions)
  {
    int v_id = kv.first;
    vector<vector<int>> predicted_traj = kv.second;
    if (predicted_traj[0][0] == lane and v_id != -1)
    {
      filtered[v_id] = predicted_traj;
    }
  }
  return filtered;
}

