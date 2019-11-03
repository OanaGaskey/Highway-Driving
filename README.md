# Highway-Driving
Path planning algorithm to safely navigate a vehicle around highway while changing lanes and limiting acceleration and jerk.
   
![FutureGIF](images/FutureGIF.JPG)

Highway driving is a topic of high interest in the self driving cars era. Driving long distance is time consuming and highway accidents are common due to fatigue and lane drifting. Premium car manufactures and trucking companies are heavily involved in implementing autonomous highway driving solutions.

In this project I implemented a path planning algorithm that drives a vehicle autonomously around a simulated highway track. The goal is to drive as close as possibe to the 50 MPH speed limit and to guarantee the passengers comfort by limiting the jerk value. The algorithm optimizies the vehicle's average speed by changing lanes when behind a slower vehicle, and checks for another faster available lane.

This project is implemented in C++ the source code can be found in the *src* folder above. The path planning logic is implemented in main.cpp under h.onMessage function.
The starting code for this project is provided by Udacity and can be found [here](https://github.com/udacity/CarND-Path-Planning-Project).


## Frenet v.s Cartesian Coordinates

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
The last two columns give the *d* value as a unit normal vector (split up into the x component, and the y component).

## Traffic Analysis

## Path Planning Model

