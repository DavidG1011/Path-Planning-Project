# Path-Planning-Project:
Self-Driving Car Engineer Nanodegree Program.

Rubric for this project is [Here](https://review.udacity.com/#!/rubrics/1020/view).

### Basic Build Instructions:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies:

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

### Goals:

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Details:

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---

### General Project Flow:

- Receive input from simulator.
- Declare paramaters.
- Calculate 3 waypoint spline.
- Calculate which lanes need to be checked.
- Check lanes and determine which cars are in range of interest.
- Calculate the cost for in-range cars.
- Check ego lane and determine if cars are close. If close, set desired speed to speed of other car.
- Determine ego speed. Will either be 50mph or the speed of an impeding car.
- Calculate cost from ego speed.
- Determine lowest cost.
- If lane change is lowest cost, determine if safe and execute.
- If stay in lane is lowest cost, execute at desired speed set by same lane check. 
- Divide spline into appropriate distancing for desired speed.
- Send x and y points to the simulator to execute.

---

### Detailed Project Flow and Explanation:

- Tunable Paramaters: Lines [267 - 304]:

[270] Debug: Toggle whether or not to output cost and best action data to terminal.

[273] Absolute max speed: The top speed the car can travel at. The specifications for this project say that the car should stay at at or less than 50 mph. 49.5 is a good speed to meet that requirement.

[277] Costs: A vector of doubles that represents the default cost for staying in your lane, as well as the costs for changing lanes. Index 0 represents the cost of staying in your lane, 1 represents a left change cost, and 2 represents a right change cost. The way my code is currently, it assumes an equal default cost for lane changes. This could be changed in the case of left-passing only laws. Since this is a simulator, I think we should be safe from the virtual police for now. 

[280 - 281] Forward distance, Rear distance: The range in frenet S ahead and behind of the ego car the lane checking algorithm will detect other cars. The default values are quite small, but in some cases, the forward distance is modified later to search further ahead.

[288] Same lane distance: How far ahead in the ego car's lane other cars will be detected. This is used for collision avoidance and general speed control.

[291 - 292] Acceleration and Deceleration increment: Controls how fast the car accelerates or decelerates. My current values were carefully picked to minimize jerk, while still being responsive enough to avoid collision and maintain speed.

[295] Spline waypoint distance: This program creates a spline from 3 incremental waypoints, and then samples the spline for 30 points to get a smooth driveable line. This paramater determines how far apart each of those 3 increments are from each other. For example: if this paramater was set to 30, it would fit a spline from origin - 30, origin - 60, origin - 90. The result of this paramater is that if it's high, the lines will be smoother and more gradual. The inverse is also true. If the waypoints are closer together, the spline will have sharper curves. I set my value to 32, as this provides a nice smooth line that is quick enough for lane changing, while also staying within my jerk limit.  

[299] Safe to accelerate: Prevents the ego car from accelerating if that acceleration would likely result in a collision with another car.

[303] Safe to change: Prevents the ego car from changing lanes if the lane change would result in a collision with another car. This was introduced due to the way my speed cost is calculated. If another car was quickly decelerating in front of the ego car, causing ego to decelerate, that cost could outweight the cost of changing lanes. This would cause the ego car to lane change and clip the back side of the car in front of it. This makes sure the ego car slows down first-- or is at an acceptable distance, before attempting a lane change. 

---

- Spline Function: Lines [319 - 414] & [642 - 668]:

[332 - 361] Use either the car as the reference point or the previous paths end point as the reference to start or continue creating a spline. This uses the previous points if they're avaliable to create a nice smooth transition. 

[365 - 376] Incrementally project out points in the distance chosen by `spline_waypoint_distance` and push them onto our vector of spline points. This gives us a smooth line that we can take 30 points from to pass to the simulator. This is a summarized version of what was discussed above in Spline waypoint distance. 

[380 - 388] Go through each point in the spline vector and shift the coordinate reference from global to local perspective. This makes the points easier to calculate as they now are now at the origin. 

[391 - 394] Inititalize spline function and pass the prepared point vectors to it. 

[397 - 407] Initialize variables `next_x_vals` & `next_y_vals` to store the x and y values passed to the simulator. Loop over and use previous points from the last iteration so we don't recalculate points.

[411 - 414] Assign `waypoint_x = 30` for 30 meters and pass to spline function to get y value for it. Calculate magnitude from x and y value. 

[643] Loop up to 50 times and push points onto the `next_x_vals` & `next_y_vals` vectors. This loop accounts for previous points we have created so we don't recalculate them. 

[647 - 667] Calculate the distance each waypoint needs to be at to travel the desired speed. Change the reference point back to global perspective so the points can be passed to the simulator. `N = (waypoint_dist/(.02*current_speed/2.2369))` This first multiplies the current desired speed by .02 since the simulator runs 50 times per second. Then, it divides the product of that by 2.2369 to convert it from mph to m/s. Finally, it divides waypoint distance by the quotient of that. N is then used to split up our line into evenly spaced points `x_point = x_add_on+(waypoint_x)/N` so the car travels at our desired speed. 

---

- Defining Lane Checking: Lines [417 - 447]:

This section of code checks which lane the ego car is in and maps out which lanes the algorithm can observe to see if there is a lower cost option available for the ego car. 1s are set for lanes that have an out of bounds area adjacent to them. The center lane or lane 1 has `forward_distance` set to 50 currently so that the cost function can choose a better fitting lane when it has 2 choices. `lanes_to_check` is a vector that represents which lanes should be checked when the ego car is in a specified lane. Lane 0 = Should check lane 1. Lane 1 = should check lane 0 and 2, Lane 2 = should check lane 1. This doesn't scale very well, but the solution is good enough for the problem. 

---

- Checking If Cars Are In Other Lanes: Lines [450 - 527]:

Loops through the sensor fusion data and extracts the d value to be used to check which lane the car is in. If the d value is in the lane that is being checked `if (d < (2+4*lanes_to_check[i]+2) && d > (2+4*lanes_to_check[i]-2))` then it will take that cars s for location and vx,vy for determining how fast the car is going. Since we are using previous path points, we need to update the cars s to get a better estimate `s_to_check += ((double)size_prev*.02*other_car_speed)`. The cars s is then checked to see if the car in the lane is within the `forward_distance` or `rear_distance` of the ego car and sets flags if so. These flags are then evaluated in [490 - 524]. If there is a car behind, it checks which lane it's in and sets a cost of 1 in the cost vector for switching to that lane. If there is a car in the lane ahead, it will check the lane, and then calculate the cost based on how far along the detection distance that car is `((forward_distance - (nearest_s[i] - car_s)) / forward_distance)`. This will ensure that lane changes are both safe and beneficial to navigational efficiency. 

---

- Checking If Cars Are In The Same Lane: Lines [532 - 555]:

This is mostly the same as checking other lanes. As mentioned in my code comments, I could have added this to the code to check other lanes, but I think it's more readable on its own. The difference here is that I check the distance ahead in the ego car lane using `same_lane_distance` to compare. I also use the car speed to determine how fast the ego car should go if within the range `current_desired_speed = ((other_car_speed * 2.2369) - 0.5)`. The value is multiplied by 2.2369 to convert from m/s to mph. The - 0.5 is just as a buffer due to latency.  I also store the S value of the in-range car to use for later on in the program. `s_to_compare = other_car_s`.  

---

- Determining How Fast To Go: Lines [558 - 585]:

[558 - 570] Checks if the current ego speed is greater than the desired set by the car in lane detection. This way the car only slows down if something is impeding it. 

[563 - 566] Bound the current speed to be the desired speed. 

[569] Calculates the cost of going slow. This is calculated by comparing the max speed it could go versus how fast it's currently going. `costs[0] = ((absolute_max_speed - current_speed) / absolute_max_speed)` 

[572 - 585] Checks if current speed is less than the desired speed. Will either be 50 mph or the speed of another car in front of it. 

[575 - 578] Checks if the ego car is too close to another car to perform safe acceleration. This logic rarely happens but is included for robustness. 

[581 - 584] Bound the current speed to be the max speed so it doesn't go higher.

---

- Find Lowest Cost Option: Lines [590 - 596]:

Finds the index of the cost vector with the lowest value. Lowest value = action with cheapest cost. Worth mentioning: Since the default lane change cost is currently the same for both left and right lane changes, the logic of this function will choose the left lane if both are at their default value.

---

- Debug Output: Lines [602 - 622]:

Ouputs the costs for staying in your lane, changing lanes left, and changing lanes right. This will also output the action chosen to execute, but may be hard to see due to how fast this outputs to the console. 

---

- Check Lane Change: Lines [625 - 640]:

Checks if the lowest costs option is a lane change and increments or decrements the lane variable appropriately. Before it can change lanes, it has to check if it's too close to another car. This is just a summarized version of what was mentioned in the tunable paramaters section.
