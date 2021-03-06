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
double get_car_in_front_speed(double previous_path_end_velocity, double previous_path_end_s,
    const MeasurementPackage &m){
    double speed = -1;
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
                speed = their_speed;
            }
        }
    }
    return speed;
}


/* return highest cost if it's not safe to switch lane
 * else, cost is determined by the speed of the car ahead 
 * in the lane I am about to switch into
 */
double get_lane_switch_cost(int delta_lane, const MeasurementPackage &m){
    double safe_front_follow_distance = m.car_speed * REACTION_SECONDS;
    double closest_gap = -1;
    double fixed_cost = (
        delta_lane < -1)? LEFT_LANE_SWITCH_FIXED_COST : RIGHT_LANE_SWITCH_FIXED_COST;
    double cost = fixed_cost;
    for (int i = 0; i < m.sensor_fusion.size(); i++){
        int their_lane = get_lane(m.sensor_fusion[i].d);
        // in the lane I am going to switch into
        if (their_lane == m.car_lane + delta_lane){

            double their_speed = sqrt(m.sensor_fusion[i].vx * m.sensor_fusion[i].vx +
                m.sensor_fusion[i].vy * m.sensor_fusion[i].vy);

            // no car within safe following distance in front of me
            if (m.sensor_fusion[i].s > m.car_s){
                if (m.sensor_fusion[i].s - m.car_s < safe_front_follow_distance){
                    return HIGHEST_COST;
                }
                else{
                    if ((closest_gap < 0) || (m.sensor_fusion[i].s - m.car_s < closest_gap)){
                        closest_gap = m.sensor_fusion[i].s - m.car_s;
                        cost = SPEED_LIMIT - their_speed + fixed_cost;
                    }
                }
            }

            // no car behind me within safe following distance
            double safe_back_follow_distance = their_speed * REACTION_SECONDS;
            if ((m.sensor_fusion[i].s < m.car_s) &&
                (m.car_s - m.sensor_fusion[i].s < safe_back_follow_distance)){
                return HIGHEST_COST;
            }
        }
    }
    return cost;

}


/* return no cost if there is no car in front
 * else cost is determined by the speed of the car in front of me
 */  
double get_keep_lane_cost(const MeasurementPackage &m){
    double car_in_front_speed = get_car_in_front_speed(SPEED_LIMIT,
        m.car_s + NUM_WAYPOINTS * WAYPOINT_INTERVAL * SPEED_LIMIT, m);
    // default: no car in front, then stay in lane
    double cost = -1;

    if (car_in_front_speed > 0){
        cost = SPEED_LIMIT - car_in_front_speed;
    }
    return cost;
}


/*
 * reduce speed limit at road segment with high curvature
 * because otherwise the centripetal acceleration can be too high
 */ 
double get_speed_limit(const MeasurementPackage &m){
    // we find curvature by locating 2 points X distance apart in frenet s coordinate
    // and also measure their cartesian distance. 
    // the smaller cartesian distance : X ratio is, the curvier the segment of the road is

    double spacing = 10;
    int segments = 10;
    vector<vector<double>> anchor_points;
    double min_dist = spacing;


    for (int i = 0; i < segments; i++){
        anchor_points.push_back(frenet_to_map_coordinates(
            m.car_s + spacing * i, m.car_d, m.map_waypoints_s,
            m.map_waypoints_x, m.map_waypoints_y));
    }

    // check for maximum curvature (i.e. min cartesian distance) in all the segments
    for (int i = 1; i < segments; i++){
        double dist = sqrt((anchor_points[i][0] - anchor_points[i - 1][0]) * 
            (anchor_points[i][0] - anchor_points[i - 1][0]) +
            (anchor_points[i][1] - anchor_points[i - 1][1]) * 
            (anchor_points[i][1] - anchor_points[i - 1][1]));
        if (dist < min_dist){
            min_dist = dist;
        }
    }

    // lower speed limit proportional to (1 - min_distance / spacing ratio)
    // clip between 5 and SPEED_LIMIT
    double speed_limit = std::min(std::max(SPEED_LIMIT * (
        1 - (1 - min_dist / spacing) * 5.), 5.), SPEED_LIMIT);
    cout << "speed_limit: " << speed_limit << endl;
    return speed_limit;
}
