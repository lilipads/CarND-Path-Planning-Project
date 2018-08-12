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


    /*
     * create a spline
     */
    tk::spline s; 
    // create anchor point that are along the center of the lane
    vector<double> car_origin = frenet_to_map_coordinates(
        m.car_s, m.car_d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y);
    
    vector<double> anchor_x;
    vector<double> anchor_y;

    int lane = get_lane(m.car_d);

    for (int i = 0; i < 3; i++){
        int anchor_point_spacing = 10;
        double d = (i == 0) ? m.car_d : get_lane_center(lane);
        vector<double> anchor_point = frenet_to_car_coordinates(m.car_s + i * anchor_point_spacing, 
            d, m.map_waypoints_s, m.map_waypoints_x, m.map_waypoints_y,
            car_origin[0], car_origin[1], m.car_yaw);
        anchor_x.push_back(anchor_point[0]);
        anchor_y.push_back(anchor_point[1]);
    }
    s.set_points(anchor_x, anchor_y);

    // vector<double> spacings = jerk_constrained_spacings(m.car_speed, 0, SPEED_LIMIT, NUM_WAYPOINTS);


    /*
     * read points from created spline
     */
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double x = 0;
    // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    for (int i = 0; i < NUM_WAYPOINTS; i++){
        // double tangent_angle = tangent_from_spline(s, x);
        // x += spacings[i] * cos(tangent_angle);
        // vector<double> map_point = car_to_map_coordinates(x, s(x), car_origin[0],
        //     car_origin[1], m.car_yaw);
        // next_x_vals.push_back(map_point[0]);
        // next_y_vals.push_back(map_point[1]);
        x = x + 0.4;
        vector<double> map_point = car_to_map_coordinates(x, s(x), car_origin[0],
            car_origin[1], m.car_yaw);
        next_x_vals.push_back(map_point[0]);
        next_y_vals.push_back(map_point[1]);
    }

    json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    return msgJson;
}