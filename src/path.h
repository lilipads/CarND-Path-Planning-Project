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



// when driving with maximum acceleration within comfortable level of jerk and acceleration limit
PlannedPath jerk_constrained_spacings(double current_velocity, 
	double current_acceleration, double target_velocity, int n);

// when driving with maximum acceleration within comfortable level of jerk and acceleration limit
PlannedPath jerk_constrained_spacings(double current_velocity, double current_acceleration, double target_velocity, int n);

PlannedPath get_straight_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	double speed_limit);

PlannedPath extend_straight_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	double speed_limit);

PlannedPath get_lane_switch_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	int delta_lane, double speed_limit);


#endif /* UTILS_H_ */