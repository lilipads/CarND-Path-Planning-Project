#ifndef PATH_H_
#define PATH_H_
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "measurement_package.h"
#include "utils.h"


using namespace std;

struct PlannedPath {
    double end_velocity;
    double end_acceleration;
    vector<double> spacings; // distance spacing between every WAYPOINT INTERVAL
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    double cost;
};

/*
 * waypoints when driving straight
 * the trajectory knows to slow down when there is car ahead and speed up when necessary

 - bool extend_trajectory: when set to true, force the planning to continue with
 	previously planned way points, and only add new way points in the end
 */ 
PlannedPath get_straight_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	double speed_limit, bool extend_trajectory);


/*
 * waypoints when switching lanes
 * the trajectory does NOT check whether it's safe to switch lane
 * (that's the job of the cost function)

 - int delta_lane: -1 when switching one lane to the left, and +1 when right
 */ 

PlannedPath get_lane_switch_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	int delta_lane, double speed_limit);


/* helper functions */

// when driving with maximum acceleration within comfortable level of jerk and acceleration limit
PlannedPath _jerk_constrained_spacings(double current_velocity, 
	double current_acceleration, double target_velocity, int n);

// fill in the next_x_vals and next_y_vals for the planned path 
void _get_trajectory(vector<double> anchor_x, vector<double> anchor_y,
	double starting_x_in_car_coordinates, PlannedPath & planned_path,
	int points_to_produce, const MeasurementPackage &m);

// anchor points for spline
void _get_anchor_points(bool append_to_current_path, vector<double> & anchor_x,
	vector<double> & anchor_y, vector<double> anchor_points_in_d, const MeasurementPackage &m,
	double anchor_point_spacing);

// return vector: {velocity at the end of the buffer, acceleration at the end of the buffer}
vector<double> _get_buffer_end_state(const MeasurementPackage &m);

#endif /* UTILS_H_ */