#include "fsm.h"

#include <iostream>
using namespace std;

FSM::FSM() {}

FSM::~FSM() {}

json FSM::next_state(const MeasurementPackage &m){
    State * next_state;
    double min_cost = 100;
    PlannedPath best_p;
    StateName new_state_name;

    // find the state with minimum cost
    for (StateName target_state : current_state -> get_next_possible_states()){
        PlannedPath p = current_state -> get_trajectory(target_state, m);
        if (p.cost < min_cost){
            best_p = p;
            next_state = get_state(target_state, p.end_velocity, p.end_acceleration);
            min_cost = p.cost;
            new_state_name = target_state;
        }
    }

    cout << current_state -> get_name() << endl;

    // if (new_state_name == lane_change_left){
    //     cout << "switch left!" << endl;
    //     cout << "cost: " << best_p.cost << endl;
    //     cout << "m.car_lane: " << m.car_lane << ", get_lane(m.end_path_d): " << get_lane(m.end_path_d) << endl;
    // }

   
    current_state = next_state;

    json msgJson;
    msgJson["next_x"] = best_p.next_x_vals;
    msgJson["next_y"] = best_p.next_y_vals;

    return msgJson;
}
