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

## Spline
It is a piecewise curve you fit with a bunch of anchor points. The pieces are formed by adjacent pairs of the anchor points.

The farther away the anchor points are to each other, the smoother the curve is. This is important to lane change. If car speed is high a steep curve will cause an abrupt change in vehicle's heading thus exceeds the maximum angular acceleration.

## s coordinate
s values for the ego car and the surrounding cars are from 0 to infinity. That is, if a car have looped for once its s value is greater than 3945. This is useful when you want to calculate the distance between two cars. e.g. s = fmod(s, max_s). This applies to `end_path_s`, too.

s values for surrounding cars starts off large, e.g. 6000+ meter or so. But they become normal quickly and grow incrementally since.

## 3-module structure
### Predictor
**TL;DR Predictor can be skipped in this assignment.**
Predictor is like a module that provides the structure of the surroundings of the ego vehicle by giving the states of the surrounding vehicles.

In general the positions of the vehicles should suffice. In each iteration we should consider the current position of a car, and also the future position of a car at a waypoint. In the current setting we use the last waypoint from `previous_path_x` and `previous_path_y` to make sure there is a safe gao between the ego car and the front car.

When the ego car starts changing lane we need another set of future positions which are the positions of the closest car in front of and behind the ego car. During the lane changing we must make sure that the other cars keep a safe distance between the ego car.

Other variable we might need are
- the velocity of a surrounding vehicle `ref_car_vel`
- two buffer distances to avoid collision: `buffer_dist_front` and `buffer_dist_rear`
- length of a car which is typically 5 meters

To summarize
- If the ego car is keeping lane, we need the current and future positions of the surrounding cars in the lane in which the ego car is.
- If the ego car wants to change lane, we need the current positions of the surrounding cars in the intended lane. And we need the future positions of the surrounding car in the intended lane when lane changing is happening. We can simplify this step by change lane when there is enough gap between the front car and the car behind. It's hard to compute the velocity of the ego car along the s direction when it's changing lane. Might as well assume it is going in the s direction to simplify things.
- If multiple lanes are feasible for changing, choose the lane with the largest gap between the ego car and the front car.

The behavior of the car behind you usually is slowing down to keep distance if it is too close to you. But
- the buffer distance for the other car is almost zero. It will go backwards until it finds that it is adjacent to you.
- it won't consider you being in front of it if you are not entirely in the same lane as it. So collision if you are changing lane.

Based on that the buffer distance between the car behind you should be `behind_car_vel` * `time_needed_for_change_lane`. Time needed might be a fixed number. This is another future position/state we need to compute when the ego vehicle starts to change lane.

The buffer distance between you and the car in front of you can be a fixed number since you have the logic to smoothly slow down if you are too close to the other car.

The true distance should be the computed distance - 5 where 5 is the front part of the car behind plus the rear part of the car ahead.

Another behavior of the car in front of you that matters is if it will change lane. Instead of using a probabilistic process model, a simpler model can be a binary model to determine if a car is changing lane by its current position.

In practice predictor should be called before and after **Behavior Planner** is called. BP needs the future (current) structure of its surroundings to decide if it needs lane change. If lane change is needed it need to know the future (current) structure of the intended lane to decide which lane to change or is it feasible to change now (if not then wait).

**Note that we can substitute future states of the cars for a fixed safe distance so as to avoid writing a predictor completely.** The only part that involves future is the last waypoint in the current trajectory, which is not a must.
### Behavior Planner
- If the gap between the car in front of the ego car and the ego car is > 30 meters, keep lane
- If the gap between the car in front of the ego car and the ego car is < 30 meters, get the states of the adjacent lanes.
  
  - If change lane is feasible for at least one adjacent lane, change lane
  - If change lane is infeasible, gradually slow down the ego car to the speed of the car in front of it. Optionally, we can compute the right acceleration for the ego car such that it won't collide to the front car before they have the same velocity. Another option is find the gap between the front car and the car in the other lane you might collide to, speed up your car to make a feasible lane change. That requires computing the acceleration you need while minimizing the jerk.
  - When it comes to choosing lane to change, one criteria is choose the lane in which the front car is further away after lane change is finished.
  
### Trajectory Generator
Hybrid A Star needs to know the ego vehicle's surroundings so it's more like Behavior Planner + Trajectory Generator. 

We need a way to tell if a lane change is finished
