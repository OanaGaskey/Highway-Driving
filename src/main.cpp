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
#include "math.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


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
   
  double ctrl_speed = 0.0; // speed control for vehicle
  bool change_lane = 0; // 0 = keep lane; 1 = change lane;
  int lane_id = 1;
  
  h.onMessage([&ctrl_speed,&change_lane,&lane_id,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          double car_speed = j[1]["speed"]; //MPH

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
          //vector <vector<double>> spline_xy;
          vector<double> next_x_vals, next_y_vals, spline_x_vals, spline_y_vals, next_point;   
          vector<double> speed_per_lane = {50.00, 50.00, 50.00};
          const double MPH_mps = 0.447; //conversion factor from MPH to m/s 
          const double safety_dist = 20.0;
          const double spline_dist = 35.0;
          const double speed_hyst = 0.5; // speed hysteresis m/s
          const double lane_width = 4.0; //m
          const double delta_time = 0.02; //s = 20ms
          const double max_speed = 21.5; //m/s which is slightly lower than 50MPH
          const double max_accel = 10.0; // m/s^2
          //const double max_jerk = 10.0; // m/s^3  
          double angle, pos_x, pos_y, pos_x2, pos_y2, shift_x, shift_y, spline_x, spline_y, next_x, next_y;
          double ref_speed = max_speed;
          double dist_inc = 0.2;
          double front_car_speed, delta_speed, x_further;
          int path_size = previous_path_x.size();
          int map_size = map_waypoints_x.size();
          int front_car_slower = 0;
          int start_point;
          bool left_clear = 1;
          bool right_clear = 1;
          tk::spline s;
          
          // analyze traffic situation
          // take the absolute value since orientation doesn't matter as long as all cars in one lane go in one direction
          car_speed = fabs(car_speed * MPH_mps); // convert own car speed from MPH to m/s
          
          // loop over all detected cars from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); ++i){
            // check to see if there are other cars in my lane going slower than me
            // check if the other's car d Frenet component is within the boundaries of my lane
            if ( ( lane_id*lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < (lane_id+1)*lane_width) ){
              // check if other car is in fromt of me, closer than the end of my previous path plus safety distance               
              if ((sensor_fusion[i][5] < (end_path_s + safety_dist)) && 
                  (sensor_fusion[i][5] > (car_s - 5.0))                 ){           
                // calculate other's car absolute speed value in m/s              
                front_car_speed = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) ); 
                // calculate the diference in speed between the two cars
                delta_speed = car_speed - front_car_speed;
                // if car going slower than me, match its speed
                if (delta_speed > 0.0){
                  ref_speed = front_car_speed;
                  front_car_slower = 1;
                  std::cout<<"CAR IN MY LANE GOING SLOWER!!!"<<std::endl;
                } 
              }
            }
            
            // check for cars at my left in the eventuality of a lane change to the left
            // make sure car is not in the left most lane
            if ( (lane_id > 0) &&
                 ((lane_id - 1) * lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < lane_id * lane_width)         ){
              // look for available space in the left lane
              if ( (sensor_fusion[i][5] < car_s + 2*safety_dist) && 
                   (sensor_fusion[i][5] > car_s - safety_dist/2)   ){       
                left_clear = 0;
              }
              // check for traffic speed in the left lane, if any
              if (sensor_fusion[i][5] > car_s - 5.0){       
                speed_per_lane[lane_id - 1] = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) );
              }              
            }
            
            // check for cars at my right in the eventuality of a lane change to the right
            // make sure car is not in the right most lane            
            if ( (lane_id < 2) &&
                 ((lane_id+1) * lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < (lane_id+2) * lane_width) ){
              // look for available space in the right lane
              if ( (sensor_fusion[i][5] < car_s + 2*safety_dist) && 
                   (sensor_fusion[i][5] > car_s - safety_dist/2)   ){
                right_clear = 0;
              }
              // check for traffic speed in the right lane, if any
              if (sensor_fusion[i][5] > car_s - 5.0){       
                speed_per_lane[lane_id + 1] = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) );
              }  
            }            
          }// for over all traffic objects          
          
          // if the car in front of me slowed me down too much, change lanes if a lane chane is not already in progress
          if ( (front_car_slower == 1) && 
               (car_speed < 0.9 * max_speed) &&
               (change_lane == 0)               ) {
            change_lane = 1;
          }
          
          if (change_lane == 1){
            //try to change lanes to the left
            std::cout<<"trying to change lanes from "<<lane_id<<std::endl;
            if ( (lane_id > 0) && 
                 (left_clear == 1) /*&&
                 (speed_per_lane[lane_id - 1] > car_speed + speed_hyst)*/ ){
              std::cout<<"left clear"<<std::endl;
              lane_id = lane_id - 1;
              std::cout<<"new lane "<<lane_id<<std::endl;
              change_lane = 0;
            }else if ( (lane_id < 2) && 
                       (right_clear == 1) /*&&
                       (speed_per_lane[lane_id + 1] > car_speed + speed_hyst)*/ ){
              std::cout<<"right clear"<<std::endl;
              lane_id = lane_id + 1;
              std::cout<<"new lane "<<lane_id<<std::endl;
              change_lane = 0;
            }        
          }
                     
          // calculate path
          // transfer previous path's points to new path
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // get previous path's end points and angle
          // if path empty or too short, get the car's current position and angle
          if (path_size == 0 || path_size == 1) {
            // get car's current position
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            //go back one distance increment using angle
            pos_x2 = pos_x - dist_inc * cos(angle);
            pos_y2 = pos_y - dist_inc * sin(angle);
            // add last two points to spline list for better transition trajectory
            spline_x_vals.push_back(pos_x2);
            spline_y_vals.push_back(pos_y2);
            spline_x_vals.push_back(pos_x);
            spline_y_vals.push_back(pos_y);
          } else {
            // get last point from previous path
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            //get second to last point from previous path
            pos_x2 = previous_path_x[path_size-2];
            pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            // add last two points to spline list for better transition trajectory
            spline_x_vals.push_back(pos_x2);
            spline_y_vals.push_back(pos_y2);
            spline_x_vals.push_back(pos_x);
            spline_y_vals.push_back(pos_y);
          }          
              
          // calculate spline points ahead of ego vehicle, spaced by spline_dist
          next_point = getXY(car_s +   spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 2*spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 3*spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          
          // transform spline points from map coordinates to car coordinates, from the perspective of the end point of the previous path 
          for (int i = 0; i < spline_x_vals.size(); ++i) {
            //std::cout<<"spline x map coord = "<<spline_x_vals[i]<<std::endl;
            shift_x = spline_x_vals[i] - pos_x;
            shift_y = spline_y_vals[i] - pos_y;
            spline_x_vals[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_y_vals[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
            //std::cout<<"spline x car coord = "<<spline_x_vals[i]<<std::endl;
          }         
          // compute spine
          s.set_points(spline_x_vals,spline_y_vals);       
               
       
          // gradually approach reference speed
          // use of hysteresis to avoid unstable speed variation
          if ((car_speed > ref_speed /*+ speed_hyst*/) && (car_speed > 0.0)){
            ctrl_speed -= max_accel * delta_time;
            //std::cout<<"speed is decreasing "<<ctrl_speed<<std::endl;
          } else if((car_speed < ref_speed - speed_hyst) && (ref_speed <= max_speed)){
            ctrl_speed += 0.5 * max_accel * delta_time;            
            //std::cout<<"speed is increasing "<<ctrl_speed<<std::endl;
          } else{
            //std::cout<<"speed is kept "<<ctrl_speed<<std::endl;
          }
          
          // calculate x distance increments based on desired velocity
          dist_inc = ctrl_speed * delta_time;  
          
          shift_x = 0.0;
          //calculate car trajectory points
          for (int i = 0; i < 30-path_size; ++i) {    
            // advance dist_inc meters down the road        
            shift_x += dist_inc;
            // use spline to calculate y
            shift_y = s(shift_x);
            // transform back to map coordinates
            next_x = shift_x * cos(angle) - shift_y * sin(angle);
            next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x += pos_x;
            next_y += pos_y;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

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