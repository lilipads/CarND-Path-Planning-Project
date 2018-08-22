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

    double speed_limit = get_speed_limit(m);

    // find the state with minimum cost
    for (StateName target_state : current_state -> get_next_possible_states()){
        PlannedPath p = current_state -> get_trajectory(target_state, m);
        if (p.cost < min_cost){
            best_p = p;
            next_state = get_state(target_state, p.end_velocity, p.end_acceleration, speed_limit);
            min_cost = p.cost;
            new_state_name = target_state;
        }
        // already found optimal path, no need to explore other states
        if (p.cost < 0){
            break;
        }
    }

   
    current_state = next_state;

    json msgJson;
    msgJson["next_x"] = best_p.next_x_vals;
    msgJson["next_y"] = best_p.next_y_vals;

    return msgJson;
}
