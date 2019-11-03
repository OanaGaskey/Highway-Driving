# Highway-Driving
Path planning algorithm to safely navigate a vehicle around highway while changing lanes and limiting acceleration and jerk.
   
![FutureGIF](images/FutureGIF.JPG)

Highway driving is a topic of high interest in the self driving cars era. Driving long distance is time consuming and highway accidents are common due to fatigue and lane drifting. Premium car manufactures and trucking companies are heavily involved in implementing autonomous highway driving solutions.

In this project I implemented a path planning algorithm that drives a vehicle autonomously around a simulated highway track. The goal is to drive as close as possibe to the 50 MPH speed limit and to guarantee the passengers comfort by limiting the jerk value. The algorithm optimizies the vehicle's average speed by changing lanes when behind a slower vehicle, and checks for another faster available lane.

This project is implemented in C++ the source code can be found in the *src* folder above. The path planning logic is implemented in main.cpp under h.onMessage function.
The starting code for this project is provided by Udacity and can be found [here](https://github.com/udacity/CarND-Path-Planning-Project).


## The Highway Map, Frenet v.s Cartesian Coordinates

In path planning, the goal is to tell the car where to move next and to generate its trajectory. To drive smoothly along the road, the trajectory needs to be along the center of the lane and take into account the road's curvature.
The road geometry can induce some particularly complicated math when computing the trajectory in [cartesian](https://en.wikipedia.org/wiki/Cartesian_coordinate_system) coordinates.

![FrenetCartesian](images/FrenetCartesian.JPG)

This created the need for a simplified coordinate system called [Frenet](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas)

In the Frenet approach, it is implied that the geometry of the road is already known through a detailed map. In this case it is much easier to work using the s value which moves along with the road and the d value which indicates the displacement from the center of the road.

For this project the map is provided in *highway_map.csv* file under the *data* folder, in the following format:
```
784.6001 1135.571 0 -0.02359831 -0.9997216
815.2679 1134.93 30.6744785308838 -0.01099479 -0.9999396
844.6398 1134.911 60.0463714599609 -0.002048373 -0.9999979
875.0436 1134.808 90.4504146575928 -0.001847863 -0.9999983
905.283 1134.799 120.689735412598 0.004131136 -0.9999915
934.9677 1135.055 150.375551223755 0.05904382 -0.9982554
```

The above are points along the yellow line marking the center of the road. The first two columns represent the *(x,y)* position of the map points. In the third column, the *s* value is given. The *s* value starts with 0 and goes around the road in increments of about 30 meters. The track is a loop of about 7 km.
The last two columns give the *d* value as a unit normal vector perpendicular to the road in the direction of the right-hand side (split up into the x component, and the y component).
The d vector in this form is easily used to calculate lane positions. *d* is then multiplied by the lane width of 4 meters and the desired position in the lane.


## Traffic Analysis

Once trajectories can be generated along the road, the next challenge is to avoid collision with other vehicles and to generate trajectories to pass them if possible.

Sensor Fusion object list is provided with the following data: 
- car's unique ID, 
- car's x position in map coordinates, 
- car's y position in map coordinates, 
- car's x velocity in m/s, 
- car's y velocity in m/s, 
- car's s position in frenet coordinates, 
- car's d position in frenet coordinates.

To avoid rear end collision, all cars from sensor fusion are checked to determine if there is any in the same lane as ego car. If a slower front car is identyfied, than the ego car shall match the car's speed.

```
// loop over all detected cars from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); ++i){
            // check if the car's d Frenet is within the boundaries of my lane
            if ( ( lane_id*lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < (lane_id+1)*lane_width) ){
              // check if the car is in fromt of me, closer than the end of my previous path plus safety distance               
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
                  //std::cout<<"CAR IN MY LANE GOING SLOWER!!!"<<std::endl;
                } 
              }
            }
          }
```

In case the ego car is driving behind a slower vehicle, the possibility of a lane change is evaluated. For example, for a left lane change there needs to be a gap in traffic in the left lane of at least 10 meters ahead of ego car and another 10 meters behind it. Checking for this gap assures the ego car won't colide laterraly into another vehicle driving by its side and also that it won't cut too short in front of another vehicle.

```
// check for cars at my left in the eventuality of a lane change to the left
            // make sure car is not in the left most lane
            if ( (lane_id > 0) &&
                 ((lane_id - 1) * lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < lane_id * lane_width)         ){
              // look for available space in the left lane
              if ( (sensor_fusion[i][5] < car_s + 10.0) && 
                   (sensor_fusion[i][5] > car_s - 10.0)   ){       
                left_clear = 0;
              }
            }
```

Finally the lane change decision is made when if driving behind a slower vehicle, there is room to change lanes either to the left or to the right lane and the traffic in the destination lane is moving faster. There is no need to get behind a slow vehicle in another lane.

```
// if the car in front of me slowed me down too much, change lanes if a lane chane is not already in progress
          if ( (front_car_slower == 1) && 
               (car_speed < 0.9 * max_speed) &&
               (change_lane == 0)               ) {
            change_lane = 1;
          }
          
          if (change_lane == 1){
            //try to change lanes to the left
            if ( (lane_id > 0) && 
                 (left_clear == 1) &&
                 (speed_per_lane[lane_id - 1] > car_speed + speed_hyst) ){
              lane_id = lane_id - 1;
              change_lane = 0;
            }else if ( (lane_id < 2) && 
                       (right_clear == 1) &&
                       (speed_per_lane[lane_id + 1] > car_speed + speed_hyst) ){
              lane_id = lane_id + 1;
              change_lane = 0;
            }        
          }
```

## Path Planning Model

The path is calculated as a sequence of (x,y) points in the map coordinate system that the ego car will visit one by one. The vechicle's speed is determinated by the spacing in between the (x,y) points given that each 20 ms cycle the car is goint to the next available point in the sequence.

The car speed is controlled in increments of `max_accel * delta_time` to assure a smooth change in velocity from one cycle to another. The reference speed is the desired velocity. This is set to 50 MPH every cycle unless the ego car is trying to match the speed of the vehicle in front.

A hysteresis is used to avoid undesired changes in velocity around the reference value and to allow for smooth driving when keeping a constant speed. 

```
          // gradually approach reference speed
          // use of hysteresis to avoid unstable speed variation
          if ((car_speed > ref_speed) && (car_speed > 0.0)){
            ctrl_speed -= max_accel * delta_time;
            //std::cout<<"speed is decreasing "<<ctrl_speed<<std::endl;
          } else if((car_speed < ref_speed - speed_hyst) && (ref_speed <= max_speed)){
            ctrl_speed += 0.5 * max_accel * delta_time;            
            //std::cout<<"speed is increasing "<<ctrl_speed<<std::endl;
          } else{
            //std::cout<<"speed is kept "<<ctrl_speed<<std::endl;
          }
```

The sequence of points can be calculated to go down the road in the middle of the selected lane or it can be processed to lay the path for a lane change.

The sequence takes into account the current position of the ego vehicle followed by the next point in the map. The map points are about 30 meters from one another. A few following map points are taken and shifted in the desired lane by the use of dx and dy components of the d unit normal vector. The dx and dy are multiplied by the lane index times the lane width to get in the corresponding lane, and half a lane width is added to place the ego car in the middle of the lane.

When a lane change is planned, the lane index is changed by one unit and these points are generated in the same manner.

```
          // calculate spline points
          // starting point is the next map waypoint from the end of the previous path
          start_point = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          // add more points to the spline list, this means looking at the road ahead to calculate shape
          // points should be aligned to the waypoints provided in the map and along the middle of the intended lane_id
          for (int i = 0; i < 3; ++i) {
            spline_x = map_waypoints_x[(start_point+i+1)%map_size] + ( (lane_width/2 + double(lane_id*lane_width)) * map_waypoints_dx[(start_point+i)%map_size] );
            spline_y = map_waypoints_y[(start_point+i+1)%map_size] + ( (lane_width/2 + double(lane_id*lane_width)) * map_waypoints_dy[(start_point+i)%map_size] );
            spline_x_vals.push_back(spline_x);
            spline_y_vals.push_back(spline_y);
          } 
```

The trajectory needs to be smooth since abrupt changes in path will lead to an undesired jerk. If the car was following the exact points from above, a lane change would imply jumping directly in an adjacent lane with infinite acceleration. To avoid this an interpolation is done to allow a progressive change while generating intermediate points.

[Cubic spline interpolation](https://kluge.in-chemnitz.de/opensource/spline/) is a great method for generating second degree polinominal trajectories that pass through each provided point. spline.h is used from this open source library.

The map points are transforned into car coordinates and the spline is fitted to these points. The transformation into car coordinates is particularly usefull for cases when the road is going straight North and multiple y values correspond to the same x. It also makes things easier when the car is going South West and the x values are decreasing by the fact that it eliminates the need to sort points by x.
The transformation is done through a [translation](https://en.wikipedia.org/wiki/Translation_of_axes) and a [rotation](https://en.wikipedia.org/wiki/Rotation_of_axes).

```
          // transform spline points from map coordinates to car coordinates, from the perspective of the end point of the previous path 
          for (int i = 0; i < spline_x_vals.size(); ++i) {
            std::cout<<"spline x map coord = "<<spline_x_vals[i]<<std::endl;
            shift_x = spline_x_vals[i] - pos_x;
            shift_y = spline_y_vals[i] - pos_y;
            spline_x_vals[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_y_vals[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
            std::cout<<"spline x car coord = "<<spline_x_vals[i]<<std::endl;
          }         
          // compute spine
          s.set_points(spline_x_vals,spline_y_vals);
```

The spline is used to generate the trajectory points. The distance ahead is increased by increments given by the driving speed. x is incremented directly since it's along the driving direction. y is computed using the spine function.
The points are transformed back into the map coordinates and represent the future trajectory of the ego car. 

```
          // calculate x distance increments based on desired velocity
          dist_inc = std::max(ctrl_speed * delta_time, 0.0001);    
          
          shift_x = 0.0;
          //calculate car trajectory points
          for (int i = 0; i < 10-path_size; ++i) {    
            // advance dist_inc meters down the road        
            shift_x += dist_inc;
            // use spline to calculate y
            shift_y = s(shift_x);
            // transform back to map coordinates
            next_x = shift_x * cos(angle) - shift_y * sin(angle);
            next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x += pos_x;
            next_y += pos_y;
            //shift_x = x_further + pos_x;
            //shift_y = next_y + pos_y;
            //next_x = shift_x * cos(angle) - shift_y * sin(angle);
            //next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
```

At each cycle a new trajectory of points is generated. For a smoother driving experience each new trajectory is built as a continuation of the previous one. All the trajectory points that were not consumed from the previous cycle are copied into the new trajectory and more points are added to it.