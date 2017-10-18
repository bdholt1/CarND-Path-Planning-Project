
#include "planner.h"
#include "spline.h"
#include "trajectory.h"

using namespace std;

Planner::Planner(int num_lanes)
{
  Vehicle ego_vehicle(-1);
  m_fsm = FSM{ego_vehicle, num_lanes};
  m_time = std::chrono::high_resolution_clock::now();
}

void Planner::add_waypoint(double x, double y, double s, double d_x, double d_y)
{
  maps_x.push_back(x);
  maps_y.push_back(y);
  maps_s.push_back(s);
  maps_dx.push_back(d_x);
  maps_dy.push_back(d_y);
}

void Planner::update(std::vector<std::vector<double>> sensor_fusion, double ego_s, double ego_d, double ego_speed)
{
  chrono::system_clock::time_point new_time = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::milli> fp_ms = new_time - m_time;

  double t = fp_ms.count() / 1000;
  m_time = new_time;

  m_fsm.update_ego(ego_s, ego_d, ego_speed, t);

  predictions.clear();
  for (auto vec : sensor_fusion)
  {
    double id = vec[0];
    double x = vec[1];
    double y = vec[2];
    double vx = vec[3];
    double vy = vec[4];
    double s = vec[5];
    double d = vec[6];

    if (vehicles.find(id) == vehicles.end())
    {
      Vehicle v(id);
      v.update(s, d, sqrt(vx*vx + vy*vy), t);
      vehicles[id] = v;
    }
    else
    {
      vehicles[id].update(s, d, sqrt(vx*vx + vy*vy), t);
    }

    std::vector <std::vector<int> > preds = vehicle[id].generate_predictions(INTERVAL);
    predictions[id] = preds;
    }
  }

}

void Planner::generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y);
{
  fsm.update_state(vehicle_predictions);
  fsm.realize_state(vehicle_predictions);

  double target_lane = fsm.ego_lane();
  double target_speed = fsm.ego_speed();

  int next_wp = -1;
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  if(prev_size < 2)
  {
    next_wp = NextWaypoint(ref_x, ref_y, ref_yaw);
  }
  else
  {
    ref_x = previous_path_x[prev_size-1];
    double ref_x_prev = previous_path_x[prev_size-2];
    ref_y = previous_path_y[prev_size-1];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
    next_wp = NextWaypoint(ref_x,ref_y,ref_yaw);

    car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
  }


  vector<double> ptsx;
  vector<double> ptsy;

  if (prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    ptsx.push_back(previous_path_x[prev_size-2]);
    ptsx.push_back(previous_path_x[prev_size-1]);

    ptsy.push_back(previous_path_y[prev_size-2]);
    ptsy.push_back(previous_path_y[prev_size-1]);

    ref_x = previous_path_x[prev_size-1];
    double ref_x_prev = previous_path_x[prev_size-2];
    ref_y = previous_path_y[prev_size-1];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

    car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
  }

  vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, maps_s, maps_x, maps_y);
  vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, maps_s, maps_x, maps_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, maps_s, maps_x, maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // transform the points into the car's local coordinate system
  for (int i = 0; i < ptsx.size(); i++ )
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);

  }

  tk::spline s;
  s.set_points(ptsx,ptsy);

  m_next_x_vals.clear();
  m_next_y_vals.clear();

  for(int i = 0; i < prev_size; i++)
  {
    m_next_x_vals.push_back(previous_path_x[i]);
    m_next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_start = 0;

  for (int i = 0; i < 50 - prev_size; i++)
  {
    if(ref_vel > car_speed)
    {
    car_speed+=.224;
    }
    else if(ref_vel < car_speed)
    {
    car_speed-=.224;
    }

    double N = target_dist*2.24/(.02*car_speed);
    double x_point = x_start+(target_x)/N;
    double y_point = s(x_point);

    x_start = x_point;

    double tmp_x = x_point;
    double tmp_y = y_point;

    // transform the points back into real-world coordinate frame
    x_point = tmp_x * cos(ref_yaw) - tmp_y * sin(ref_yaw);
    y_point = tmp_x * sin(ref_yaw) + tmp_y * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    m_next_x_vals.push_back(x_point);
    m_next_y_vals.push_back(y_point);
  }

  int prev_size = previous_path_x.size();

  if (prev_size > 0)
  {
    car_s = end_path_s;
  }

}

double distance(double x1, double y1, double x2, double y2) const
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Planner::ClosestWaypoint(double x, double y) const
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int Planner::NextWaypoint(double x, double y, double theta) const
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    //heading vector
    double hx = map_x-x;
    double hy = map_y-y;

    //Normal vector:
    double nx = maps_dx[closestWaypoint];
    double ny = maps_dy[closestWaypoint];

    //Vector into the direction of the road (perpendicular to the normal vector)
    double vx = -ny;
    double vy = nx;

    //If the inner product of v and h is positive then we are behind the waypoint so we do not need to
    //increment closestWaypoint, otherwise we are beyond the waypoint and we need to increment closestWaypoint.

    double inner = hx*vx+hy*vy;
    if (inner<0.0) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Planner::getFrenet(double x, double y, double theta) const
{
    int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y, maps_dx, maps_dy);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Planner::getXY(double s, double d) const
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}
