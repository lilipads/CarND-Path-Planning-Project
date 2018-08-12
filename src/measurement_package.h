#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

class MeasurementPackage {
public:
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path data given to the Planner
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  // Previous path's end s and d values 
  double end_path_s;
  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  // auto sensor_fusion = j[1]["sensor_fusion"]; //  [ id, x, y, vx, vy, s, d]

  // map data
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
