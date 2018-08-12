#include <vector>

enum StateName = {start, keep_lane, lane_change_left};

class State {
public:
	State();
	virtual State();
	vector<StateName> next_possible_states;

	trajectory get_trajectory(StateName target_state_name, environment){};
}


class StartState : State{
public:
	StartState();
	next_possible_states = {StateName.keep_lane};
	trajectory get_trajectory(StateName target_state_name, environment){
		switch(target_state_name) {
		    case StateName.keep_lane: 
		    	// accelerate with expected acceleration
		    	break;
		    default:
		    	throw "invalid state";
	    }
	};
}


class KeepLaneState : State{
public:
	KeepLaneState();
	next_possible_states = {StateName.keep_lane, StateName.lane_change_left};
	trajectory get_trajectory(StateName target_state_name, environment){
		switch(target_state_name) {
		    case StateName.keep_lane: 
		    	// if no car in front, accelerate to speed limit, cost 0
		    	// if car in front, reduce to that car's speed with JMT before safe distance
		    	break;
		    case StateName.lane_change_left:
		    	// if car on the left, do nothing, prohibitive cost
		    	// else, JMT to target state on the left
		    	break;
		    default:
		    	throw "invalid state";
		}
	};
}


class LaneChangeLeftState : State{
public:
	KeepLaneState();
	next_possible_states = {StateName.lane_change_left, StateName.keep_lane};
	trajectory get_trajectory(StateName target_state_name, environment){
		// append a few more points to the end of the trajectory
		switch(target_state_name) {
		    case StateName.lane_change_left: 
		    	// if still in the middle of changing lane, must stick to this state
		    	break;
		    case StateName.keep_lane:
		    	// if finished changing lane, must switch to this state
		    	break;
		    default:
		    	throw "invalid state";
		}
	};
}


State create_state(StateName name){
	// TODO
}