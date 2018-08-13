#include "utils.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}


// theta is car_yaw in rad
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);
  
    if(angle > pi()/4)
    {
      closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
    }
  
    return closestWaypoint;
}


// return lane number
int get_lane(double d){
    return floor(d / LANE_WIDTH);
}


// return d (frenet coordinate) for lane center
double get_lane_center(int lane){
    return lane * LANE_WIDTH + LANE_WIDTH / 2;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// theta is car_yaw in rad
vector<double> map_to_frenet_coordinates(double x, double y, double theta,
    const vector<double> &maps_x, const vector<double> &maps_y){
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

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
vector<double> frenet_to_map_coordinates(double s, double d, const vector<double> &maps_s,
    const vector<double> &maps_x, const vector<double> &maps_y)
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

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};
}


// car_origin_x and car_origin_y are in map coordinate
vector<double> map_to_car_coordinates(double x, double y, double car_origin_x, double car_origin_y, double yaw){
    double shift_x = x - car_origin_x;
    double shift_y = y - car_origin_y;
    double car_x = shift_x * cos(yaw) + shift_y * sin(yaw);
    double car_y = -shift_x * sin(yaw) + shift_y * cos(yaw);
    return {car_x, car_y};
}


// car_origin_x and car_origin_y are in map coordinate
vector<double> frenet_to_car_coordinates(double s, double d, const vector<double> &maps_s,
    const vector<double> &maps_x, const vector<double> &maps_y,
    double car_origin_x, double car_origin_y, double yaw){
    vector<double> map_point = frenet_to_map_coordinates(s, d, maps_s, maps_x, maps_y);
    return map_to_car_coordinates(map_point[0], map_point[1], car_origin_x, car_origin_y, yaw);
}


/* Transform from car coordinate to map coordinates
   car_origin_x and car_origin_y are in map coordinate
*/
vector<double> car_to_map_coordinates(double car_x, double car_y, double car_origin_x,
    double car_origin_y, double yaw){
    double map_x = car_x * cos(yaw) - car_y * sin(yaw) + car_origin_x;
    double map_y = car_x * sin(yaw) + car_y * cos(yaw) + car_origin_y;
    return {map_x, map_y};
}


/* return car id number for the car right front of me
   return -1 at the end of my previously planned path, there is no car that will be below
   safe following distance in front of me
*/
int get_car_in_front(double previous_path_end_velocity, double previous_path_end_s, const MeasurementPackage &m){
    int car_id = -1;
    double closest_gap = previous_path_end_velocity * REACTION_SECONDS;  // safe distance
    for (int i = 0; i < m.sensor_fusion.size(); i++){
        int their_lane = get_lane(m.sensor_fusion[i].d);
        // in my lane
        if (their_lane == m.car_lane){
            double their_speed = sqrt(m.sensor_fusion[i].vx * m.sensor_fusion[i].vx +
                m.sensor_fusion[i].vy * m.sensor_fusion[i].vy);
            // where they will be at the end of my planned path
            double their_future_s = m.sensor_fusion[i].s + NUM_WAYPOINTS * WAYPOINT_INTERVAL * their_speed;
            // currently in front me, and in the future will be too close
            if ((m.sensor_fusion[i].s > m.car_s) && (their_future_s - previous_path_end_s < closest_gap)){
                closest_gap = their_future_s - previous_path_end_s;
                car_id = i;
            }
        }
    }
    return car_id;
}


/* when driving with maximum acceleration within comfortable level of jerk and acceleration limit
  return distance spacing between every WAYPOINT INTERVAL
*/
PlannedPath jerk_constrained_spacings(double current_velocity, double current_acceleration, 
    double target_velocity, int n){
    PlannedPath planned_path;
    double delta_speed_from_zero_acc_to_max_acc = 1. / 2 * EXPECTED_ACCELERATION * EXPECTED_ACCELERATION / EXPECTED_JERK;

    cout << "jerk_constrained_spacings: " << endl;
    cout << "current_velocity: " << current_velocity << ", " <<
        "target_velocity: " << target_velocity << endl;
    for (int i = 0; i < n; i++){
        if (target_velocity - current_velocity > delta_speed_from_zero_acc_to_max_acc){
            if (current_acceleration < 0){
                current_acceleration = 0;
            }
            else if (current_acceleration + EXPECTED_JERK * WAYPOINT_INTERVAL < EXPECTED_ACCELERATION){
                // acceleration with expected jerk
                current_acceleration += EXPECTED_JERK * WAYPOINT_INTERVAL;
            }
        }
        else if (target_velocity - current_velocity > 0) {
            current_acceleration = std::max(current_acceleration - EXPECTED_JERK * WAYPOINT_INTERVAL, 0.);
        }
        // exceed speed limit
        else {
            if (current_acceleration > 0){
                current_acceleration = 0;
            }
            else if (current_acceleration - EXPECTED_JERK * WAYPOINT_INTERVAL >
                - EXPECTED_ACCELERATION){
                current_acceleration -= EXPECTED_JERK * WAYPOINT_INTERVAL;
            }
        }
        current_velocity += current_acceleration * WAYPOINT_INTERVAL;
        planned_path.spacings.push_back(current_velocity * WAYPOINT_INTERVAL);

        // cout << "current acceleration: " << current_acceleration << ", " 
        // << "current velocity: " << current_velocity << endl;
    }

    planned_path.end_acceleration = current_acceleration;
    planned_path.end_velocity = current_velocity;
    return planned_path;
}
