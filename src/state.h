#ifndef STATE_H_
#define STATE_H_
#include <vector>
#include <algorithm>
#include "utils.h"
#include "measurement_package.h"
#include "path.h"

using namespace std;


enum StateName {start, keep_lane, lane_change_left, lane_change_right};

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
	KeepLaneState(double s_previous_path_end_velocity,
		double s_previous_path_end_acceleration, double s_speed_limit);
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);

private:
	double previous_path_end_acceleration;
	double previous_path_end_velocity;
	double speed_limit;
};

class LaneChangeLeft : public State{
public:
	LaneChangeLeft();
	LaneChangeLeft(double s_previous_path_end_velocity,
		double s_previous_path_end_acceleration, double s_speed_limit);
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);

private:
	double previous_path_end_acceleration;
	double previous_path_end_velocity;
	double speed_limit;
};


class LaneChangeRight : public State{
public:
	LaneChangeRight();
	LaneChangeRight(double s_previous_path_end_velocity,
		double s_previous_path_end_acceleration, double s_speed_limit);
	PlannedPath get_trajectory(StateName target_state_name, const MeasurementPackage &m);

private:
	double previous_path_end_acceleration;
	double previous_path_end_velocity;
	double speed_limit;
};


State * get_state(StateName state, double end_velocity, double end_acceleration,
	double speed_limit);

#endif /* STATE_H_ */