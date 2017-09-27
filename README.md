# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## 1. Modify the way points
It is found that the original way points are sparse. Therefore, the generated way points leads to non-smooth mapping between [s,d] and [x,y]. This would further leads to speed, acceleration, and jerk conflict.
![Sparse way point problem](./img/sparse_way_point_problem.png)
### 1.1. interpolate original way points using [spline tool](http://kluge.in-chemnitz.de/opensource/spline/)
```
// Modify the way points
tk::spline s_x, s_y, s_dx, s_dy;
s_x.set_points(map_waypoints_s_raw,map_waypoints_x_raw);
s_y.set_points(map_waypoints_s_raw,map_waypoints_y_raw);
s_dx.set_points(map_waypoints_s_raw,map_waypoints_dx_raw);
s_dy.set_points(map_waypoints_s_raw,map_waypoints_dy_raw);

double s_max = *max_element(begin(map_waypoints_s_raw),end(map_waypoints_s_raw));
for(int i=0;i<s_max;i+=10) {
    map_waypoints_s.push_back((double)i);
    map_waypoints_x.push_back(s_x((double)i));
    map_waypoints_y.push_back(s_y((double)i));
    map_waypoints_dx.push_back(s_dx((double)i));
    map_waypoints_dy.push_back(s_dy((double)i));
}
```
The path is much smoother than before. However, it seems that the way point is not evenly distributed. This would cause speed fluctuation.
![Speed fluctuation problem](./img/speed_fluctuation_problem.png)

### 1.2. rescale way points
To tackle the above problem, I will recalculate map_waypoint_s according to map_waypoint_x and map_waypoint_y.
```
map_waypoints_s.clear();
for(int i=0;i<map_waypoints_x.size();i++) {
    if(i==0) {
        map_waypoints_s.push_back(0.0);
    }
    else {
        double delta_s = distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i-1],map_waypoints_y[i-1]);
        map_waypoints_s.push_back(map_waypoints_s[i-1]+delta_s);
    }
}
```
The problem is alleviated but not yst solved. This is mainly due to the function 'getXY'. This function provides no smoothing mechanism. Therefore, I have to realize this mechanism before entering an array of [x,y] into 'next_points'.

### 1.3 Smoothing before sending commands


## Other instructions
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```