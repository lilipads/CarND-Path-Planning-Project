#ifndef FSM_H_
#define FSM_H_
#include <math.h>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include "measurement_package.h"
// #include "state.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// finite state machine
class FSM {
public:
	/**
	* Constructor.
	*/
	FSM();

	/**
	* Destructor.
	*/
	virtual ~FSM();

	// State current_state = start;

	// State get_current_state();

    json next_state(const MeasurementPackage &m);

private:
	// double previous_path_end_x;
	// double previous_path_end_y;
	double previous_path_end_velocity = 0;
	double previous_path_end_acceleration = 0;
	// double previous_path_end_yaw;
};

#endif /* FSM_H_ */
