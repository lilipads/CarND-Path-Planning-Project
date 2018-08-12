#ifndef UTILS_H_
#define UTILS_H_
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// define constants
const double SPEED_LIMIT = 22.3; // 22.3 meter / second = 50 miles per hour
const double MPH_TO_MPS_CONVERSION = 0.44; // 1 mile per hour = 0.44 meter / s
const double LANE_WIDTH = 4; // meter
const double MAXIMUM_ACCELERATION = 9; // meter / second ^2
const double EXPECTED_ACCELERATION= 2.6; // meter / second ^2
const double MAX_JERK = 10; // m/s/s/s
const double EXPECTED_JERK = 2; // m/s/s/s

const double WAYPOINT_INTERVAL = 0.02; // second. paths are made up of (x,y) points that the car will visit sequentially every .02 seconds
const int NUM_WAYPOINTS = 100; // how many way points to generate each time


/*
 *  helper functions
 */
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// return lane number
int get_lane(double d);
// return d (frenet coordinate) for lane center
double get_lane_center(int lane);

// coordinates transformations
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
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

// when driving with maximum acceleration within comfortable level of jerk and acceleration limit
// return distance spacing between every WAYPOINT INTERVAL
// vector<double> jerk_constrained_spacings(double current_velocity, double current_acceleration, double target_velocity, int n);

// double tangent_from_spline(tk::spline s, double x);
#endif /* UTILS_H_ */
