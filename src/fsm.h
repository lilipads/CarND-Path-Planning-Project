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
#include "state.h"

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

	State * current_state = new StartState;

	// State get_current_state();

    json next_state(const MeasurementPackage &m);

};

#endif /* FSM_H_ */
