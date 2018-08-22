#include "state.h"

State * get_state(StateName state, double end_velocity, double end_acceleration,
    double speed_limit){
    State * new_state;
    if (state == keep_lane){
        new_state = new KeepLaneState(end_velocity, end_acceleration, speed_limit);
    }
    else if (state == lane_change_left){
        new_state = new LaneChangeLeft(end_velocity, end_acceleration, speed_limit);
    }
    else if (state == lane_change_right){
        new_state = new LaneChangeRight(end_velocity, end_acceleration, speed_limit);
    }
    return new_state;
}


State::State() {}
StateName State::get_name(){return name;}
vector<StateName> State::get_next_possible_states(){return next_possible_states;}

StartState::StartState() {
    name = keep_lane;
    next_possible_states = {keep_lane};
}


KeepLaneState::KeepLaneState(
    double s_previous_path_end_velocity, double s_previous_path_end_acceleration,
    double s_speed_limit) :
    previous_path_end_velocity(s_previous_path_end_velocity),
    previous_path_end_acceleration(s_previous_path_end_acceleration),
    speed_limit(s_speed_limit) {
    name = keep_lane;
    next_possible_states = {keep_lane, lane_change_left, lane_change_right};
}

LaneChangeLeft::LaneChangeLeft(
    double s_previous_path_end_velocity, double s_previous_path_end_acceleration,
    double s_speed_limit) :
    previous_path_end_velocity(s_previous_path_end_velocity),
    previous_path_end_acceleration(s_previous_path_end_acceleration),
    speed_limit(s_speed_limit) {
    name = lane_change_left;
    next_possible_states = {keep_lane, lane_change_left};
}


LaneChangeRight::LaneChangeRight(
    double s_previous_path_end_velocity, double s_previous_path_end_acceleration,
    double s_speed_limit) :
    previous_path_end_velocity(s_previous_path_end_velocity),
    previous_path_end_acceleration(s_previous_path_end_acceleration),
    speed_limit(s_speed_limit) {
    name = lane_change_right;
    next_possible_states = {keep_lane, lane_change_right};
}


PlannedPath StartState::get_trajectory(StateName statename, const MeasurementPackage &m){
    PlannedPath p = get_straight_trajectory(m, 0, 0, SPEED_LIMIT, false);
    return p;
}


PlannedPath KeepLaneState::get_trajectory(StateName statename, const MeasurementPackage &m){
    PlannedPath p;
    switch(statename) {
        case keep_lane:
            p = get_straight_trajectory(m, previous_path_end_velocity,
                previous_path_end_acceleration, speed_limit, false);
            p.cost = get_keep_lane_cost(m);
            if (p.cost > 0){
                cout << "keep lane cost: " << p.cost;
            }
            break;
        case lane_change_left:
            // cannot switch left when already in left most lane
            if (m.car_lane == 0){
                p.cost = HIGHEST_COST;
                break;
            }
            else{
                p = get_lane_switch_trajectory(m, previous_path_end_velocity,
                previous_path_end_acceleration, -1, speed_limit);
                p.cost = get_lane_switch_cost(-1, m);
            }
            cout <<" switch left cost: " << p.cost;
            break;
        case lane_change_right:
            // cannot switch right when already in right most lane
            if (m.car_lane == MAX_LANE){
                p.cost = HIGHEST_COST;
            }
            else{
                p = get_lane_switch_trajectory(m, previous_path_end_velocity,
                previous_path_end_acceleration, 1, speed_limit);
                p.cost = get_lane_switch_cost(1, m);
            }
            cout <<" switch right cost: " << p.cost << endl;
            break;
        default:
            throw "invalid state";
    }
    return p;
}


PlannedPath LaneChangeLeft::get_trajectory(StateName statename, const MeasurementPackage &m){
    PlannedPath p;
    p = get_straight_trajectory(m, previous_path_end_velocity,
        previous_path_end_acceleration, speed_limit, true);

    // if car is already near the center of the target lane, we consider it has finshed the lane
    // switch and thus force it to transtion to keep_lane state (by giving it a low cost).
    // otherwise, stay in lane change state (so it cannot do another lane change yet)
    switch(statename) {
        // finished switching lane
        case keep_lane: 
            if (abs(m.car_d - m.end_path_d) < LANE_WIDTH / 10.){
                p.cost = -1;
            }
            else{
                p.cost = HIGHEST_COST;
            }
            break;
        // still in the process of lane change 
        case lane_change_left:
            if (abs(m.car_d - m.end_path_d) < LANE_WIDTH / 10.){
                p.cost = HIGHEST_COST;
            }
            else{
                p.cost = -1;
            }
            break;
        default:
            throw "invalid state";
    }
    return p;
}


PlannedPath LaneChangeRight::get_trajectory(StateName statename, const MeasurementPackage &m){
    PlannedPath p;
    p = get_straight_trajectory(m, previous_path_end_velocity,
        previous_path_end_acceleration, speed_limit, true);

    // if car is already near the center of the target lane, we consider it has finshed the lane
    // switch and thus force it to transtion to keep_lane state (by giving it a low cost).
    // otherwise, stay in lane change state (so it cannot do another lane change yet)
    switch(statename) {
        // finished switching lane
        case keep_lane: 
            if (abs(m.car_d - m.end_path_d) < LANE_WIDTH / 10.){
                p.cost = -1;
            }
            else{
                p.cost = HIGHEST_COST;
            }
            break;
        // still in the process of lane change
        case lane_change_right:
            if (abs(m.car_d - m.end_path_d) < LANE_WIDTH / 10.){
                p.cost = HIGHEST_COST;
            }
            else{
                p.cost = -1;
            }
            break;
        default:
            throw "invalid state";
    }
    return p;
}
