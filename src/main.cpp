#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

bool wayToSort(vector<double> i, vector<double> j) { return i[0] < j[0]; }

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          vector<double> next_x_vals, next_y_vals, spline_x_vals, spline_y_vals;
          vector <vector<double>> spline_xy;
          double angle, pos_x, pos_y, pos_x2, pos_y2, spline_x, spline_y, next_x, next_y;
          double dist_inc;
          double max_speed = 22.0; //m/s which is slightly lower than 50MPH
          //double speed_hysteresis = 0.1; //m/s
          double ref_speed;
          double delta_time = 0.02; //s = 20ms
          double max_accel = 9.0; // m/s^2
          double max_jerk = 9.0; // m/s^3
          //double dist_ref = 30.0;
          tk::spline s;   
          int path_size = previous_path_x.size();
          int map_size = map_waypoints_x.size();
          int lane_id = 1;
          double lane_width = 4.0;
          int start_point;
          
          // transfer previous path's points to new path
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //cmakestd::cout<<"previous path size is = "<<path_size<<std::endl;
          
          // get previous path's end points and angle
          // if path empty or too short, get the car's current position and angle
          if (path_size == 0 || path_size == 1) {
            // get car's current position
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            //go back one distance increment using angle
            //pos_x2 = pos_x - dist_inc * cos(angle);
            //pos_y2 = pos_y - dist_inc * sin(angle);
            // add last two points to spline list for better transition trajectory
            //spline_xy.push_back({pos_x2,pos_y2});
            spline_xy.push_back({pos_x,pos_y});
            // if there are no previous path points it is likely that this is the first cycle
            // start with zero reference speed value, to incrementally ramp up the car's velocity 
            ref_speed = 0.0;
            std::cout<<"initial speed = "<<ref_speed<<std::endl;
          } else {
            // get last point from previous path
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            //get second to last point from previous path
            pos_x2 = previous_path_x[path_size-2];
            pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            // add last two points to spline list for better transition trajectory
            spline_xy.push_back({pos_x2,pos_y2});
            spline_xy.push_back({pos_x,pos_y});
            ref_speed = car_speed * 0.44704; //conversion from MPH to m/s
            std::cout<<"car speed = "<<ref_speed<<std::endl;
          }
          
          // calculate spline points
          // starting point is the closest map waypoint to the end of previous path's end
          start_point = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          // add 10 more points to the spline list
          // points should be aligned to the waypoints provided in the map and along the middle of the intended lane_id
          for (int i = 0; i < 10; ++i) {
            spline_x = map_waypoints_x[(start_point+i)%map_size] + ( (lane_width/2 + double(lane_id*lane_width)) * map_waypoints_dx[(start_point+i)%map_size] );
            spline_y = map_waypoints_y[(start_point+i)%map_size] + ( (lane_width/2 + double(lane_id*lane_width)) * map_waypoints_dy[(start_point+i)%map_size] );
            spline_xy.push_back({spline_x,spline_y});
          }
          // sort spline_xy vector by the x value as expected by set_points in spline header function
          std::sort(spline_xy.begin(), spline_xy.end(), wayToSort);
          // split x and y values from spline points
          for (int i = 0; i < spline_xy.size(); ++i) {
            spline_x_vals.push_back(spline_xy[i][0]);
            spline_y_vals.push_back(spline_xy[i][1]);
          }
          s.set_points(spline_x_vals,spline_y_vals);
          
          // calculate dist_inc to smoothly ramp up the speed
          std::cout<<"ref speed before comp "<<ref_speed<< " to max speed "<<max_speed<<std::endl;
          if (ref_speed < max_speed) {
            ref_speed += max_accel * delta_time;
            std::cout<<"new ref speed "<<ref_speed<<std::endl;
            std::cout<<"velocity increment =  "<<max_accel * delta_time<<std::endl;
          }
          //std::cout<<"ref speed = "<<ref_speed<<std::endl;
          dist_inc = ref_speed * delta_time;
          
          //calculate car trajectory points
          //next_x = pos_x;
          for (int i = 0; i < 10-path_size; ++i) {    
            // advance dist_inc meters down the road
            //next_s = end_path_s + dist_inc*(i+1);
            //next_d = 6;
            //next_x = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)[0];
            //next_y = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)[1];
            
            // use spline to calculate y
            next_x = pos_x + dist_inc*(i+1);
            next_y = s(next_x);
            
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
          //std::cout<<"path size is = "<<next_x_vals.size()<<std::endl;
          /*
          for (int i = 0; i<next_x_vals.size(); ++i){
            std::cout<<"x = "<<next_x_vals[i]<<"y = "<<next_y_vals[i]<<std::endl;
          }
            */       
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
    //debugfile.close();
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