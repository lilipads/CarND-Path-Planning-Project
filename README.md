# CarND-Path-Planning-Project


## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Solution Overview

I use a finite state machine (FSM), with 3 defined states, such as keep_lane, change_lane_left, and change_lane_right. At every cycle, the fsm evaluates all the possible transitions. Each transition is associated with a trajectory planned out for the car (in the form of waypoints) and a cost. The FSM transitions into the state with the lowest transition cost. See fsm.cpp/next_state().

### Trajectory Generation
There are a few options to generate trajectories. In the course, we went over the jerk-minimizing-trajectory method. I didn't adopt that approach because it requires us to specify the end state location. In the case of straight highway driving, the ego car does not have a defined location it must reach in a given time. But rather, it just needs to speed up / slow down when necessary.

Here I propose an original method. See details at path.cpp/_jerk_constrained_spacings(). The basic idea is we acclerate with the maximal acceleration and jerk that the passenger is comfortable with to get to the target speed. I set it to 2.6 m/s^2 for accleration and 2 m/s^3 for jerk. If the car is about to approach target speed, we lower the acceleration  and smoothly transition to a constant movement with 0 accleration. If the car is over speed limit, we gently start braking with a comfortable level of jerkiness. So this method will give us a list of spacings between way points: how far the car should travel every 0.02 seconds.

Separately, I fit a spline to a few anchor points. In the case of straight driving, I anchor on a few points straight ahead in the current lane (same d in frenet coordinate, and a few meters apart in s). See path.cpp/get_straight_trajectory().

The spacings control for how fast the car should drive (and thus tell us how far apart the waypoints should be), and the spline tells us the trajectory the car should drive on (and thus tells us which curve the waypoints are drawn from). To map the spacings onto the trajectory, I start at the last way point, find the tangent at that point in the spline, and locate the next waypoint's x coordinate in relation to the last waypoint's x coordinate using cosine of that angle. Repeat this process and we will get all the waypoints.


### How to slow down when there is a car ahead
When generating a straight trajectory, we check whether there will is any car in front of us within our safe following distance (see utils.cpp/get_car_in_front_speed()). If there isn't one, to save computation, we keep the previously generated waypoints and just append a few new points to the end. If there is a car in front, then we need to slow down to that car's speed as soon as possible. To avoid too sudden a transition, instead of scratching off the previous path completely, we still retain a few points (buffer points) from the previous way points, and repalce the rest of the waypoints with a new plan to adapt to the situation. If that car is now out of our way, we will speed back up to the speed limit. All the logic is handled in path.cpp/get_straight_trajectory().


### Cost functions: knows to switch lane when the car ahead is too slow
We use a simple cost function here: how far away is the car from the speed limit at the end of the currently planned trajectory. If we switch lanes, we add a fixed overhead cost (because if all else equal, we prefer not to change lanes frequently!) -- a lower cost to switch left and a higher cost to switch lane right (because passing on the right is frowned upon).

In addition, there are a few hard constraints for impossible transitions: when in the leftmost lane, the cost of switching further left is at the highest (99). Similarly, if we are switching left, but see cars on the left that will collide with us, the cost is also at the highest.

See utils.cpp/get_keep_lane_cost() and utils.cpp/get_lane_switch_cost().

### Adjustable speed limit
With the above implementation, the car intelligently plans its path, but at times it will exceed maximal acceleration. Upon further investigation, when this happened, I saw it was usually exceeding normal accleration while having a 0 tangential acceleration. This means that, even though the car is driving within the speed limit, part of the highway is too windy that we need to slow down to reduce the centripetal acceleration (perhaps the highway should have better speed limit!). To account for for that, I added a logic to adjust the speed limit depending on how curvy the road is. See utils.cpp/get_speed_limit().

Essentially, we quantify how windy the road is by locating 2 points X distance apart in frenet s coordinate and then draw a straight line between them and measure their cartesian distance. The smaller cartesian distance : X ratio is, the curvier the segment of the road is. To see this, for a straight road, cartesian distance : X = 1. I subtract from the speed limit based on how far away this ratio is from 1.

## Future work
I started with a simple fsm with 3 states. To better swtich lanes, in future, we can add states such as prepare_to_switch_left and prepare_to_switch_right. This way, it can also know when to slow down first and then switch lane.

We are using a simple cost function now and it's worth improving. For example, it can incorporate jerkiness, accleration, etc. all as part of the cost in future.

The adjustable speed limit is very much based on heuristic right now to avoid the "exceed max accelration" error. Someitmes, we still exceed max acceleration and other times, it is driving too slow. Ideally, we can read the nromal acceleration data directly and control for that.

Lastly, we currently don't predict other cars' behavior, and assume that they will all drive straight ahead at their current speed. In highway driving this is generally not a problem. But prediction can help foresee situations where the car on the right is about to switch into our lane. Then we can slow down before the car switches into our lane.
