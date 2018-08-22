#include "path.h"


/* speed up to the target velocity within the jerk and acceleration limit
 * return distance spacing between every WAYPOINT INTERVAL
 */
PlannedPath _jerk_constrained_spacings(double current_velocity, double current_acceleration, 
    double target_velocity, int n){
    PlannedPath planned_path;
    // this is the increase in speed when the car goes from 0 acceleration to maximal acceleration
    // with maximal jerk. This is useful because when car is about to approach target velocity,
    // we should stop acceleration
    double delta_speed_from_zero_acc_to_max_acc = 1. / 2 *
    	MAX_ACCELERATION * MAX_ACCELERATION / MAX_JERK;

    for (int i = 0; i < n; i++){
        if (target_velocity - current_velocity > delta_speed_from_zero_acc_to_max_acc){
            if (current_acceleration < 0){
                current_acceleration = 0;
            }
            else{
                // accelerate with max jerk
                current_acceleration += std::min(MAX_ACCELERATION - current_acceleration,
                    MAX_JERK * WAYPOINT_INTERVAL);
            }
        }
        // approaching target velocity, lower acceleration
        else if (target_velocity - current_velocity >= 0) {
            current_acceleration = std::max(current_acceleration - MAX_JERK * WAYPOINT_INTERVAL, 0.);
        }
        // exceed speed limit
        else {
            if (current_acceleration > 0){
                current_acceleration = 0;
            }
            else {
                current_acceleration -= std::min(MAX_ACCELERATION + current_acceleration,
                    MAX_JERK * WAYPOINT_INTERVAL);
            }
        }
        current_velocity += current_acceleration * WAYPOINT_INTERVAL;
        planned_path.spacings.push_back(current_velocity * WAYPOINT_INTERVAL);
    }

    planned_path.end_acceleration = current_acceleration;
    planned_path.end_velocity = current_velocity;
    return planned_path;
}


PlannedPath get_lane_switch_trajectory(const MeasurementPackage &m,
    double previous_path_end_velocity, double previous_path_end_acceleration,
    int delta_lane, double speed_limit){


	/* 
	 * anchor points for the spline
	 */

    vector<double> anchor_x;
    vector<double> anchor_y;
    int anchor_point_spacing = 30;
    bool append_to_current_path = false;

    // for the additional points: first point is straight ahead in the current lane
    // then anchor a few points in the center of the target lane we want to switch into
    vector<double> anchor_points_in_d;
    double center_d = get_lane_center(m.car_lane);
    double target_lane_d = get_lane_center(m.car_lane + delta_lane);
    anchor_points_in_d.push_back(center_d);
    for (int i = 0; i < 3; i++){
    	anchor_points_in_d.push_back(target_lane_d);
    }

 	// retain the buffer points. Then add the additional points
    _get_anchor_points(append_to_current_path, anchor_x, anchor_y,
    	anchor_points_in_d, m, anchor_point_spacing);


    /*
     * get the spacings between way points
     */

    // calculate car state by buffer end time
    double buffer_end_acceleration = 0.;
    double buffer_end_speed = m.car_speed;

    if (m.previous_path_x.size() >= 2){
        double distance2 = sqrt(pow((m.previous_path_x[BUFFER_POINTS] -
            m.previous_path_x[BUFFER_POINTS - 1]), 2) +
            pow((m.previous_path_y[BUFFER_POINTS] -
                m.previous_path_y[BUFFER_POINTS - 1]), 2));
        double distance1 = sqrt(pow((m.previous_path_x[BUFFER_POINTS - 1] -
            m.previous_path_x[BUFFER_POINTS - 2]), 2) +
            pow((m.previous_path_y[BUFFER_POINTS - 1] -
                m.previous_path_y[BUFFER_POINTS - 2]), 2));
        buffer_end_acceleration = (distance2 - distance1) / WAYPOINT_INTERVAL /
            WAYPOINT_INTERVAL;
        buffer_end_speed = distance1 / WAYPOINT_INTERVAL;
    }

    // get the spacings: keep constant speed when switching lane
    double starting_x_in_car_coordinates = anchor_x[BUFFER_POINTS - 1];
    int points_to_produce = NUM_WAYPOINTS * 4;
    PlannedPath planned_path = _jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration,
    	buffer_end_speed, points_to_produce);


    /*
     * get the trajectory: fill in the next_x_vals and next_y_vals for the planned path
     */

    // retain the first <BUFFER_POINTS> way points from the previous path
    for (int i = 0; i < BUFFER_POINTS; i++){
        planned_path.next_x_vals.push_back(m.previous_path_x[i]);
        planned_path.next_y_vals.push_back(m.previous_path_y[i]);
    }
    _get_trajectory(anchor_x, anchor_y, starting_x_in_car_coordinates, planned_path,
		points_to_produce, m);

    return planned_path;
}


PlannedPath get_straight_trajectory(const MeasurementPackage &m,
    double previous_path_end_velocity, double previous_path_end_acceleration,
    double speed_limit){

    /*
     * create a spline bsaed on some anchort points (in car coordinates)
     * anchored on: previous planned path (or if empty, current car position)
     * and 20 meters out from the last point of the previous path
     */
    tk::spline s; 

    vector<double> anchor_x;
    vector<double> anchor_y;

    // starting anchor points: previous planned path. translate it to car coordinates
    for (int i = 0; i < m.previous_path_x.size(); i++){
        vector<double> anchor_point = map_to_car_coordinates(
            m.previous_path_x[i], m.previous_path_y[i], m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    // additional anchor points: extend anchor_point_spacing * 2 meters out from the last planned waypoint 
    double center_d = get_lane_center(get_lane(m.end_path_d));
    int anchor_point_spacing = 30;

    for (int i = 1; i < 3; i++){
        vector<double> anchor_point = frenet_to_car_coordinates(
            m.end_path_s + i * anchor_point_spacing, 
            center_d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y,
            m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    s.set_points(anchor_x, anchor_y);


    /*
     * read points from created spline
     * define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
     */

    // only retain the first <BUFFER_POINTS> way points from the previous path
    vector <double> next_x_vals;
    vector <double> next_y_vals;
    for (int i = 0; i < BUFFER_POINTS; i++){
        next_x_vals.push_back(m.previous_path_x[i]);
        next_y_vals.push_back(m.previous_path_y[i]);
    }

    // calculate car state by buffer end time
    double buffer_end_acceleration = 0.;
    double buffer_end_speed = m.car_speed;

    if (m.previous_path_x.size() >= 2){
        double distance2 = sqrt(pow((m.previous_path_x[BUFFER_POINTS] -
            m.previous_path_x[BUFFER_POINTS - 1]), 2) +
            pow((m.previous_path_y[BUFFER_POINTS] -
                m.previous_path_y[BUFFER_POINTS - 1]), 2));
        double distance1 = sqrt(pow((m.previous_path_x[BUFFER_POINTS - 1] -
            m.previous_path_x[BUFFER_POINTS - 2]), 2) +
            pow((m.previous_path_y[BUFFER_POINTS - 1] -
                m.previous_path_y[BUFFER_POINTS - 2]), 2));
        buffer_end_acceleration = (distance2 - distance1) / WAYPOINT_INTERVAL /
            WAYPOINT_INTERVAL;
        buffer_end_speed = distance1 / WAYPOINT_INTERVAL;
    }

    PlannedPath planned_path;
    int points_to_produce;
    double starting_x_in_car_coordinates;
    double car_in_front_speed = get_car_in_front_speed(previous_path_end_velocity, m.end_path_s, 
        m);
    
    // if there is no car in front
    if (car_in_front_speed < 0) {
        // if no car in front and car is not deccelerating, just append onto previously planned waypoints
        if (buffer_end_acceleration >= 0){
            next_x_vals = m.previous_path_x;
            next_y_vals = m.previous_path_y;
            points_to_produce = NUM_WAYPOINTS - m.previous_path_x.size();
            planned_path = _jerk_constrained_spacings(previous_path_end_velocity,
                previous_path_end_acceleration, speed_limit,
                points_to_produce);
            starting_x_in_car_coordinates = anchor_x[m.previous_path_x.size() - 1];
        }
        // if no car in front but car is deccelerating, redo the path beyond buffer
        else{
            points_to_produce = NUM_WAYPOINTS - BUFFER_POINTS;
            planned_path = _jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration, speed_limit, points_to_produce);
            starting_x_in_car_coordinates = anchor_x[BUFFER_POINTS - 1];
        }
    }
    // if there is car in front, clear previously planned path and reduce to their car's speed
    else{
        points_to_produce = NUM_WAYPOINTS - BUFFER_POINTS;

        planned_path = _jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration,
            car_in_front_speed, points_to_produce);
        starting_x_in_car_coordinates = anchor_x[BUFFER_POINTS - 1];
    }


    double x = starting_x_in_car_coordinates;
    vector<double> map_point;
    for (int i = 0; i < points_to_produce; i++){
        // calculate angle at spline
        double delta_x = 5;
        double tangent_angle = atan2(s(x + delta_x) - s(x), delta_x);
        x += planned_path.spacings[i] * cos(tangent_angle);
        map_point = car_to_map_coordinates(x, s(x), m.car_x, m.car_y, m.car_yaw);
        next_x_vals.push_back(map_point[0]);
        next_y_vals.push_back(map_point[1]);
    }

    /*
     * set end state of this planned path
     */
    planned_path.next_x_vals = next_x_vals;
    planned_path.next_y_vals = next_y_vals;

    return planned_path;
}


PlannedPath extend_straight_trajectory(const MeasurementPackage &m,
    double previous_path_end_velocity, double previous_path_end_acceleration,
    double speed_limit){

    /*
     * create a spline bsaed on some anchort points (in car coordinates)
     * anchored on: previous planned path (or if empty, current car position)
     * and 20 meters out from the last point of the previous path
     */
    tk::spline s; 

    vector<double> anchor_x;
    vector<double> anchor_y;

    // starting anchor points: previous planned path. translate it to car coordinates
    for (int i = 0; i < m.previous_path_x.size(); i++){
        vector<double> anchor_point = map_to_car_coordinates(
            m.previous_path_x[i], m.previous_path_y[i], m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    // additional anchor points: extend anchor_point_spacing * 2 meters out from the last planned waypoint 
    double center_d = get_lane_center(get_lane(m.end_path_d));
    int anchor_point_spacing = 10;

    for (int i = 1; i < 3; i++){
        vector<double> anchor_point = frenet_to_car_coordinates(
            m.end_path_s + i * anchor_point_spacing, 
            center_d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y,
            m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    s.set_points(anchor_x, anchor_y);


    /*
     * read points from created spline
     * define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
     */

    // only retain the first <BUFFER_POINTS> way points from the previous path
    vector <double> next_x_vals;
    vector <double> next_y_vals;

    PlannedPath planned_path;
    int points_to_produce;
    double starting_x_in_car_coordinates;
    

    next_x_vals = m.previous_path_x;
    next_y_vals = m.previous_path_y;
    points_to_produce = NUM_WAYPOINTS - m.previous_path_x.size();
    planned_path = _jerk_constrained_spacings(previous_path_end_velocity,
        previous_path_end_acceleration, speed_limit,
        points_to_produce);
    starting_x_in_car_coordinates = anchor_x[m.previous_path_x.size() - 1];


    double x = starting_x_in_car_coordinates;
    vector<double> map_point;
    for (int i = 0; i < points_to_produce; i++){
        // calculate angle at spline
        double delta_x = 5;
        double tangent_angle = atan2(s(x + delta_x) - s(x), delta_x);
        x += planned_path.spacings[i] * cos(tangent_angle);
        map_point = car_to_map_coordinates(x, s(x), m.car_x, m.car_y, m.car_yaw);
        next_x_vals.push_back(map_point[0]);
        next_y_vals.push_back(map_point[1]);
    }

    /*
     * set end state of this planned path
     */
    planned_path.next_x_vals = next_x_vals;
    planned_path.next_y_vals = next_y_vals;

    return planned_path;
}


/*
 * fill in the next_x_vals and next_y_vals for the planned path 
 * by first creating a spline, and then translate the spacings onto points
 * on the spline
 args:
 - anchor_x, anchor_y: points to create the spline
 - planned_path: result of jerk_constrained_spacings,
 		contain a vector for the distances between each adjacent way point pairs
 - starting_x_in_car_coordinates: x coordinate of the start of the trajectory
 - points_to_produce: how many way points to add to the planned path
 */ 
void _get_trajectory(vector<double> anchor_x, vector<double> anchor_y,
	double starting_x_in_car_coordinates, PlannedPath & planned_path,
	int points_to_produce, const MeasurementPackage &m){

    /*
     * create a spline bsaed on some anchort points (in car coordinates)
     * anchored on: previous planned path (or if empty, current car position)
     * and 20 meters out from the last point of the previous path
     */
    tk::spline s; 
    s.set_points(anchor_x, anchor_y);


    /*
     * read points from created spline
     * define a path made up of (x,y) points that the car will visit sequentially
     * every .02 seconds
     */

    double x = starting_x_in_car_coordinates;

    // to translate the distance into cartesian coordinate, find the tangent at the
    // current point in the spline, and locate the x coordinate in relation to the
    // current x coordinate using cosine of that angle
    for (int i = 0; i < points_to_produce; i++){
        // calculate angle at spline
        double delta_x = 5;
        double tangent_angle = atan2(s(x + delta_x) - s(x), delta_x);
        x += planned_path.spacings[i] * cos(tangent_angle);
        vector<double> map_point = car_to_map_coordinates(
        	x, s(x), m.car_x, m.car_y, m.car_yaw);
        planned_path.next_x_vals.push_back(map_point[0]);
        planned_path.next_y_vals.push_back(map_point[1]);
    }
}


/*
 * return a list of anchor points to fit a spline for the trajectory
 - append to current_path: true if adding points on to the end of the already planned path
 		false means we will only keep the buffering points, and erase everything else
 - anchor_x, anchor_y: results will be stored here
 - anchor_points_in_d: vector of the d coodinate (frenet) of the new anchor points
 - anchor_point_spacing: distance in s (frenet) of two adjacent anchor points
 */
void _get_anchor_points(bool append_to_current_path, vector<double> & anchor_x,
	vector<double> & anchor_y, vector<double> anchor_points_in_d, const MeasurementPackage &m,
	double anchor_point_spacing){

	double starting_s;
	int num_previous_points;

	if (append_to_current_path){
		num_previous_points = m.previous_path_x.size();
		starting_s = m.end_path_s;
	}
	else{
		num_previous_points = BUFFER_POINTS;
		// s of the end of the buffer
		starting_s = map_to_frenet_coordinates(m.previous_path_x[BUFFER_POINTS - 1],
	        m.previous_path_y[BUFFER_POINTS - 1], m.car_yaw, m.map_waypoints_x,
	        m.map_waypoints_y)[0];
	}
	

	// add points from previous path to the anchor points
	for (int i = 0; i < num_previous_points; i++){
        vector<double> anchor_point = map_to_car_coordinates(
            m.previous_path_x[i], m.previous_path_y[i], m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }
    
    // add new anchor points: each <anchor_point_spacing> apart,
    // and their d (frenet) is determined by <anchor_points_in_d>
    for (int i = 0; i < anchor_points_in_d.size(); i++){
        vector<double> anchor_point = frenet_to_car_coordinates(
            starting_s + (i + 1) * anchor_point_spacing, 
            anchor_points_in_d[i], m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y,
            m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }
}

