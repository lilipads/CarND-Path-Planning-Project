#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "fsm.h"
#include "util.h"
using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  FSM fsm;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&fsm, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, 
    char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

            /*
             * STEP 1: load everything into measurment package
             */ 
            
            MeasurementPackage measurement_package;

            // j[1] is the data JSON object            
        	// Main car's localization Data
            measurement_package.car_x = j[1]["x"];
            measurement_package.car_y = j[1]["y"];
            measurement_package.car_s = j[1]["s"];
            measurement_package.car_d = j[1]["d"];
            measurement_package.car_yaw = deg2rad(j[1]["yaw"]);
            measurement_package.car_speed = j[1]["speed"];
            measurement_package.car_speed *= MPH_TO_MPS_CONVERSION;
            measurement_package.car_lane = get_lane(measurement_package.car_d);
  
            // Previous path data given to the Planner
            for (auto x : j[1]["previous_path_x"]){
                measurement_package.previous_path_x.push_back(x);
            }
            for (auto y : j[1]["previous_path_y"]){
                measurement_package.previous_path_y.push_back(y);
            }

            // Previous path's end s and d values 
            measurement_package.end_path_s = j[1]["end_path_s"];
            measurement_package.end_path_d = j[1]["end_path_d"];

            // for convenience of calculation later on, if there is no previous path
            // add the car's current position
            if (measurement_package.previous_path_x.size() == 0){
                measurement_package.previous_path_x.push_back(measurement_package.car_x);
                measurement_package.previous_path_y.push_back(measurement_package.car_y);
                measurement_package.end_path_s = measurement_package.car_s;
                measurement_package.end_path_d = measurement_package.car_d;
            }
            
            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            for (int i = 0; i < j[1]["sensor_fusion"].size(); i++){
                Car car;
                car.x = j[1]["sensor_fusion"][i][1];
                car.y = j[1]["sensor_fusion"][i][2];
                car.vx = j[1]["sensor_fusion"][i][3];
                car.vx *= MPH_TO_MPS_CONVERSION;
                car.vy = j[1]["sensor_fusion"][i][4];
                car.vy *= MPH_TO_MPS_CONVERSION;
                car.s = j[1]["sensor_fusion"][i][5];
                car.d = j[1]["sensor_fusion"][i][6];
                measurement_package.sensor_fusion.push_back(car);
            }

            measurement_package.map_waypoints_x = map_waypoints_x;
            measurement_package.map_waypoints_y = map_waypoints_y;
            measurement_package.map_waypoints_s = map_waypoints_s;
            measurement_package.map_waypoints_dx = map_waypoints_dx;
            measurement_package.map_waypoints_dy = map_waypoints_dy;



            /*
             * STEP 2: get next state for the finite state machine
             */ 
            
            json msgJson = fsm.next_state(measurement_package);
            auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
