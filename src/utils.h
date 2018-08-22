#ifndef UTILS_H_
#define UTILS_H_
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "measurement_package.h"

using namespace std;

/*
 * define constants
 */
const double SPEED_LIMIT = 20; // 22 meter / second = 49.2 miles per hour
const double MPH_TO_MPS_CONVERSION = 0.44; // 1 mile per hour = 0.44 meter / s
const double LANE_WIDTH = 4; // meter
const double MAX_LANE = 2; // max lane number
const double MAX_ACCELERATION = 2.6; // meter / second ^2
const double MAX_JERK = 2; // m/s/s/s
const double REACTION_SECONDS = 1; // must be 1 second away from the car in front

const double WAYPOINT_INTERVAL = 0.02; // second. paths are made up of (x,y) points that the car will visit sequentially every .02 seconds
const int NUM_WAYPOINTS = 50; // how many way points to generate each time
const int BUFFER_POINTS = 10; // start changing course of actions only after 10 points (0.2 seconds)
const double HIGHEST_COST = 99;
const double LEFT_LANE_SWITCH_FIXED_COST = 2; // fixed overhead cost for switching lanes
const double RIGHT_LANE_SWITCH_FIXED_COST = 3; // fixed overhead cost for switching lanes


/*
 *  helper functions
 */
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);



/*
 * coordinates transformations
 */ 


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// return vector {frenet_s,frenet_d}
vector<double> map_to_frenet_coordinates(double x, double y, double theta,
	const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> frenet_to_map_coordinates(double s, double d, const vector<double> &maps_s,
	const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from map (cartesian) coordinates to car coordinates
vector<double> map_to_car_coordinates(double x, double y, double car_origin_x,
	double car_origin_y, double yaw);
// Transform from Frenet to car coordinates
vector<double> frenet_to_car_coordinates(double s, double d, const vector<double> &maps_s,
	const vector<double> &maps_x, const vector<double> &maps_y,
	double car_origin_x, double car_origin_y, double yaw);
// Transform from car coordinate to map coordinates
vector<double> car_to_map_coordinates(double car_x, double car_y, double car_origin_x,
	double car_origin_y, double yaw);


/*
 * other helper functions
 */

// return lane number
int get_lane(double d);
// return d (frenet coordinate) for lane center
double get_lane_center(int lane);

/*
return speed of the car closest to me that is in front of me and is in my lane
return -1 if no car is in front within safe distance
*/
double get_car_in_front_speed(double previous_path_end_velocity, double previous_path_end_s, const MeasurementPackage &m);

/*
 * reduce speed limit at road segment with high curvature
 * because otherwise the centripetal acceleration can be too high
 */ 
double get_speed_limit(const MeasurementPackage &m);

/* return highest cost if it's not safe to switch lane
 * else, cost is determined by the speed of the car ahead 
 * in the lane I am about to switch into
 */
double get_lane_switch_cost(int delta_lane, const MeasurementPackage &m);

double get_keep_lane_cost(const MeasurementPackage &m);


#endif /* UTILS_H_ */
