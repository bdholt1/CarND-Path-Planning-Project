
#include "map.h"

#include <cmath>

using namespace std;

void Map::add_waypoint(double x, double y, double s, double d_x, double d_y)
{
  maps_x.push_back(x);
  maps_y.push_back(y);
  maps_s.push_back(s);
  maps_dx.push_back(d_x);
  maps_dy.push_back(d_y);
}

double Map::distance(double x1, double y1, double x2, double y2) const
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::ClosestWaypoint(double x, double y) const
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

int Map::NextWaypoint(double x, double y, double theta) const
{

    int closestWaypoint = ClosestWaypoint(x,y);

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
vector<double> Map::getFrenet(double x, double y, double theta) const
{
    int next_wp = NextWaypoint(x,y, theta);

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
vector<double> Map::getXY(double s, double d) const
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

    double perp_heading = heading-M_PI/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}
