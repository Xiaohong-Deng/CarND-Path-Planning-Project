# Path Planning Project

## Latency
The simulator runs at 50 fps. And by default we set 50 waypoints and the vehicle moves from one point to another in a single frame. It's noted that the c++ program can not receive and send data in a single 20 ms cycle. So the latency is at least 20 ms. This means
1. vehicle relies on a certain path for several waypoints.
2. when the simulator receives new path, it will locate the nearest waypoint to the current vehicle location and use the points starting from the nearest point, discarding the points have been driven over.

## highway_map.csv
The file contains a set of waypoints forming a line looping around the highway. The line lies in the center of the highway. The last point is the same as the first point.

## d vector
The last two elements in the waypoint vector are the x and y coordinates for the unit vector that is perpendicular to the highway direction. It is not explicitly pointed out which unit vector it is among the two possible vectors that are perpendicular to the highway. It is noted that if you want to move a point on the highway to the left, you can add the vector represented by the point's x and y coordinates to the unit vector. It implies the unit vector is pointing to the left of the highway.

## Frenet Coordinate
d is the lateral distance relative to the center of highway. The left part of highway are 3 lanes, each of which is 4 meters wide. The left part are 3 identical lanes heading towards the opposite direction.

## Tangential Path
In the Q&A video, the instructors provide a way to set the path tangential to the vehicle heading. For the `prev_size < 2`, the first two points in the path should be `prev_car_x` and `car_x`
```cpp
double prev_car_x = car_x - cos(car_yaw)
```
where formally `cos(car_yaw) = (car_x - prev_car_x) / (sqrt(car_x * car_x + car_y * car_y) - sqrt(prev_car_x * prev_car_x + prev_car_y * prev_car_y))` is the right equation. But we can take the previous point such that the denominator is equal to 1. This justify the previous equation.

Another thing we should notice is between 2 waypoints the vehicle moves in straight line.

I guess the reason we want to get the last two points in the previous path is because we want to make sure the new points added can form a path that the vehicle can follow. So we need to add new points to the end of the last two points from the previous path.

## Inputs from Simulator

- `end_path_s` and `end_path_d`: the s and d values for the point at previous_path
- `sensor_fusion`: 2d vector where each member of the outer vector is a vector of the form [ id, x, y, vx, vy, s, d ]. `sensor_fusion[0][6]` is the d value of the 1st car in the vector. Note 0 does not mean the car id is 0.
