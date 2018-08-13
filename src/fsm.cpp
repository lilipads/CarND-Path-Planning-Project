#include "fsm.h"

#include <iostream>
using namespace std;

FSM::FSM() {}

FSM::~FSM() {}

// State FSM::get_current_state(){ return current_state;}

json FSM::next_state(const MeasurementPackage &m){
    // set to the state with minimum cost
    // json best_trajectory; // TODO
    // state next_state;
    // min_cost = -1;
    // for (StateName target_s : current_state.next_possible_states){
    //     trajectory, cost = current_state.get_trajectory(target_s, environment); // TODO
    //     if (min_cost == -1 or cost < min_cost){
    //         best_trajectory = trajectory;
    //         next_state = target_s;
    //         min_cost = cost;

    //         // already found optimal solution. no need to compute the rest
    //         if (cost == 0){
    //             break;
    //         }
    //     }
    // }

    // current_state = create_state(target_s); // TODO
    // return trajectory;

    cout << endl << endl << endl;
    cout << "================new cycle================= ";

    /*
     * create a spline bsaed on some anchort points (in car coordinates)
     * anchored on: previous planned path (or if empty, current car position)
     * and 20 meters out from the last point of the previous path
     */
    tk::spline s; 

    vector<double> anchor_x;
    vector<double> anchor_y;

    vector<double> car_origin = frenet_to_map_coordinates(
    m.car_s, m.car_d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y);
    
    double previous_path_end_s = map_to_frenet_coordinates(m.previous_path_x.back(),
        m.previous_path_y.back(), m.car_yaw, m.map_waypoints_x, m.map_waypoints_y)[0];

    // starting anchor points: previous planned path. translate it to car coordinates
    for (int i = 0; i < m.previous_path_x.size(); i++){
        vector<double> anchor_point = map_to_car_coordinates(
            m.previous_path_x[i], m.previous_path_y[i], car_origin[0],
            car_origin[1], m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    // additional anchor points: extend anchor_point_spacing * 2 meters out from the last planned waypoint 
    double center_d = get_lane_center(m.car_lane);
    int anchor_point_spacing = 10;

    for (int i = 1; i < 3; i++){
        vector<double> anchor_point = frenet_to_car_coordinates(
            previous_path_end_s + i * anchor_point_spacing, 
            center_d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y,
            car_origin[0], car_origin[1], m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }

    // cout << "anchor_x: ";
    // for (int i = 0; i < anchor_x.size(); i++){ cout << anchor_x[i] << ", ";}
    // cout << endl;

    // cout << "anchor_y: ";
    // for (int i = 0; i < anchor_y.size(); i++){ cout << anchor_y[i] << ", ";}
    // cout << endl;

    // fit a line to the anchor points
    s.set_points(anchor_x, anchor_y);
    cout << "car speed (measured): " << m.car_speed << endl;
    cout << "car_x: " << m.car_x << ", car_y: " << m.car_y << endl;

    cout << "previous_path_x: ";
    for (int i = 0; i < m.previous_path_x.size(); i++){ cout << m.previous_path_x[i] << ", ";}
    cout << endl;

    cout << "previous_path_y: ";
    for (int i = 0; i < m.previous_path_x.size(); i++){ cout << m.previous_path_y[i] << ", ";}
    cout << endl;


    /*
     * read points from created spline
     * define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
     */
    vector<double> next_x_vals = m.previous_path_x;
    vector<double> next_y_vals = m.previous_path_y;

    PlannedPath planned_path;
    int points_to_produce;
    double starting_x_in_car_coordinates;
    int car_in_front_id = get_car_in_front(previous_path_end_velocity, previous_path_end_s, m);
    
    if (car_in_front_id == -1) {
        cout << "car NOT in front!" << endl;
        // if no car in front, append onto previously planned waypoints
        points_to_produce = NUM_WAYPOINTS - m.previous_path_x.size();
        planned_path = jerk_constrained_spacings(previous_path_end_velocity,
            previous_path_end_acceleration, SPEED_LIMIT,
            points_to_produce);
        starting_x_in_car_coordinates = anchor_x[m.previous_path_x.size() - 1];
    }
    else{
        // if there is car in front, clear previously planned path and reduce to their car's speed
        cout << "car in front!" << endl;

        // buffer time: start changing course of action only after 10 points (0.2 seconds)
        int buffer_points = std::min(10, (int) m.previous_path_x.size()); 
        points_to_produce = NUM_WAYPOINTS - buffer_points;

        // only retain the first <buffer_points> way points from the previous path
        next_x_vals = {};
        next_y_vals = {};
        for (int i = 0; i < buffer_points; i++){
            next_x_vals.push_back(m.previous_path_x[i]);
            next_y_vals.push_back(m.previous_path_y[i]);
        }

        // speed of the car in front of us
        double their_speed = sqrt(m.sensor_fusion[car_in_front_id].vx * 
            m.sensor_fusion[car_in_front_id].vx + m.sensor_fusion[car_in_front_id].vy *
            m.sensor_fusion[car_in_front_id].vy);

        double future_acceleration = 0.;
        double future_speed = m.car_speed;

        if (m.previous_path_x.size() >= 2){
            double distance2 = sqrt(pow((m.previous_path_x[buffer_points] -
                m.previous_path_x[buffer_points - 1]), 2) +
                pow((m.previous_path_y[buffer_points] -
                    m.previous_path_y[buffer_points - 1]), 2));
            double distance1 = sqrt(pow((m.previous_path_x[buffer_points - 1] -
                m.previous_path_x[buffer_points - 2]), 2) +
                pow((m.previous_path_y[buffer_points - 1] -
                    m.previous_path_y[buffer_points - 2]), 2));
            future_acceleration = (distance2 - distance1) / WAYPOINT_INTERVAL /
                WAYPOINT_INTERVAL;
            future_speed = distance1 / WAYPOINT_INTERVAL;
        }

        cout << "future_acceleration: " << future_acceleration << endl;
        cout << "their speed: " << their_speed << endl;
        cout << "future speed (calculated): " << future_speed << endl;
        planned_path = jerk_constrained_spacings(future_speed, future_acceleration, their_speed,
            points_to_produce);

        starting_x_in_car_coordinates = anchor_x[buffer_points - 1];
    }

    cout << "spacings: ";
    for (int i = 0; i < planned_path.spacings.size(); i++){ cout << planned_path.spacings[i] << ", ";}
    cout << endl;

    double x = starting_x_in_car_coordinates;
    vector<double> map_point;
    for (int i = 0; i < points_to_produce; i++){
        // calculate angle at spline
        double delta_x = 5;
        double tangent_angle = atan2(s(x + delta_x) - s(x), delta_x);
        x += planned_path.spacings[i] * cos(tangent_angle);
        // cout << "planned_path.spacings[i]" << planned_path.spacings[i] << endl;
        // cout << "planned_path.spacings[i] * cos(tangent_angle): " << planned_path.spacings[i] * cos(tangent_angle) << endl;
        map_point = car_to_map_coordinates(x, s(x), car_origin[0],
            car_origin[1], m.car_yaw);
        next_x_vals.push_back(map_point[0]);
        next_y_vals.push_back(map_point[1]);
    }

    /*
     * set end state of this planned path
     */
    previous_path_end_velocity = planned_path.end_velocity;
    previous_path_end_acceleration = planned_path.end_acceleration;

    /* 
     * set return values in json
     */
    json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    cout << "msgJson: " << msgJson["next_x"] << endl;
    cout << "msgJson: " << msgJson["next_y"] << endl;

    return msgJson;
}