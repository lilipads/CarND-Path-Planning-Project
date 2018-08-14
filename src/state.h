#ifndef STATE_H_
#define STATE_H_
#include <vector>
#include "utils.h"
#include "measurement_package.h"
#include "spline.h"


enum StateName {start, keep_lane, lane_change_left};

class State {
public:
	StateName name;
	vector<StateName> next_possible_states;

	State();
	StateName get_name();
	vector<StateName> get_next_possible_states();
	virtual PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m) = 0;
};


class StartState : public State{
public:
	StartState();
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);
};


class KeepLaneState : public State{
public:
	KeepLaneState();
	KeepLaneState(double s_previous_path_end_velocity, double s_previous_path_end_acceleration);
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);

private:
	double previous_path_end_acceleration;
	double previous_path_end_velocity;
};

class LaneChangeLeft : public State{
public:
	LaneChangeLeft();
	LaneChangeLeft(double s_previous_path_end_velocity, double s_previous_path_end_acceleration);
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);

private:
	double previous_path_end_acceleration;
	double previous_path_end_velocity;
};


/*
return index of the car in sensor_fusion vector
find the car closest to me that is in front of me and is in my lane
return -1 if no car is in front within safe distance
*/
int get_car_in_front(double previous_path_end_velocity, const MeasurementPackage &m);


// when driving with maximum acceleration within comfortable level of jerk and acceleration limit
PlannedPath jerk_constrained_spacings(double current_velocity, double current_acceleration, double target_velocity, int n);

PlannedPath get_straight_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration);

PlannedPath get_lane_switch_trajectory(const MeasurementPackage &m,
	double previous_path_end_velocity, double previous_path_end_acceleration,
	int delta_lane);

#endif /* STATE_H_ */