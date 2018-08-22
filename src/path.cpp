#include "path.h"


/* when driving with maximum acceleration within comfortable level of jerk and acceleration limit
  return distance spacing between every WAYPOINT INTERVAL
*/
PlannedPath jerk_constrained_spacings(double current_velocity, double current_acceleration, 
    double target_velocity, int n){
    PlannedPath planned_path;
    double delta_speed_from_zero_acc_to_max_acc = 1. / 2 * EXPECTED_ACCELERATION * EXPECTED_ACCELERATION / EXPECTED_JERK;

    for (int i = 0; i < n; i++){
        if (target_velocity - current_velocity > delta_speed_from_zero_acc_to_max_acc){
            if (current_acceleration < 0){
                current_acceleration = 0;
            }
            else{
                // acceleration with expected jerk
                current_acceleration += std::min(EXPECTED_ACCELERATION - current_acceleration,
                    EXPECTED_JERK * WAYPOINT_INTERVAL);
            }
        }
        // approaching target velocity, slow down acceleration
        else if (target_velocity - current_velocity >= 0) {
            current_acceleration = std::max(current_acceleration - EXPECTED_JERK * WAYPOINT_INTERVAL, 0.);
        }
        // exceed speed limit
        else {
            if (current_acceleration > 0){
                current_acceleration = 0;
            }
            else {
                current_acceleration -= std::min(EXPECTED_ACCELERATION + current_acceleration,
                    EXPECTED_JERK * WAYPOINT_INTERVAL);
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
     * create a spline bsaed on some anchort points (in car coordinates)
     * anchored on: previous planned path (or if empty, current car position)
     * and 20 meters out from the last point of the previous path
     */
    tk::spline s; 

    // start from car origin
    vector<double> anchor_x;
    vector<double> anchor_y;

    // keep the buffer points from the previous path 
    // TODO: previous path can be small
    for (int i = 0; i < BUFFER_POINTS; i++){
        vector<double> anchor_point = map_to_car_coordinates(
            m.previous_path_x[i], m.previous_path_y[i], m.car_x, m.car_y, m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    double center_d = get_lane_center(m.car_lane);
    int anchor_point_spacing = 30;
    double buffer_end_s = map_to_frenet_coordinates(m.previous_path_x[BUFFER_POINTS - 1],
        m.previous_path_y[BUFFER_POINTS - 1], m.car_yaw, m.map_waypoints_x,
        m.map_waypoints_y)[0];

    // first point is straight ahead 10 meters
    // next two anchor points is 20 meters and 30 meters ahead in the target lane
    for (int i = 1; i < 5; i++){
        if (i > 1){
            center_d = get_lane_center(m.car_lane + delta_lane);
        }
        vector<double> anchor_point = frenet_to_car_coordinates(
            buffer_end_s + i * anchor_point_spacing, 
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
    
    points_to_produce = NUM_WAYPOINTS * 4; // - BUFFER_POINTS;
    // planned_path = jerk_constrained_spacings(m.car_speed, 0, SPEED_LIMIT, points_to_produce);
    // starting_x_in_car_coordinates = 0;
    planned_path = jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration, buffer_end_speed, points_to_produce);
    starting_x_in_car_coordinates = anchor_x[BUFFER_POINTS - 1];

    
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
            planned_path = jerk_constrained_spacings(previous_path_end_velocity,
                previous_path_end_acceleration, speed_limit,
                points_to_produce);
            starting_x_in_car_coordinates = anchor_x[m.previous_path_x.size() - 1];
        }
        // if no car in front but car is deccelerating, redo the path beyond buffer
        else{
            points_to_produce = NUM_WAYPOINTS - BUFFER_POINTS;
            planned_path = jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration, speed_limit, points_to_produce);
            starting_x_in_car_coordinates = anchor_x[BUFFER_POINTS - 1];
        }
    }
    // if there is car in front, clear previously planned path and reduce to their car's speed
    else{
        points_to_produce = NUM_WAYPOINTS - BUFFER_POINTS;

        planned_path = jerk_constrained_spacings(buffer_end_speed, buffer_end_acceleration,
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
    planned_path = jerk_constrained_spacings(previous_path_end_velocity,
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