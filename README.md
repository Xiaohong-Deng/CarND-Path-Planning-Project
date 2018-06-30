# Path Planning Project

I recorded [a clip of the path planner running in the simulator](https://www.youtube.com/watch?v=auRrXORI0zY)

## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### The code compiles correctly
It compiles correctly on my machine.

### Valid Trajectory
#### 1. The car is able to drive at least 4.32 miles without incident.
Check.
#### 2. The car drives according to the speed limit.
Check. I set the max speed to 49.5 m/h
#### 3. Max Acceleration and Jerk are not Exceeded.
Check.
#### 4. Car does not have collisions.
Check.
#### 5. The car stays in its lane, except for the time between changing lanes.
Check.
#### 6. The car is able to change lanes.
Check.

### Reflection
#### There is a reflection on how to generate paths
For a full-fledged Path Planner, it can be divided to 3 modules: Predictor, Behavior Planner and Trajectory Generator. However, my implementation is a simplified one, e.g., it has no Predictor.

##### Predictor
Usually we want to predict the surrounding car's imminent behavior. In the simulator the surrounding cars just follow lane most of the time. They also change lane rarely. I reason it is an overkill to predict if the car is going to change lane.

##### Behavior Planner
For a full-fledged Behavior Planner often we have a FSM containing all possible states the ego car can be in and a cost function based on which we pick a next best state for the ego car. I didn't use either of them.

In my implementation, if the ego car doesn't have another car blocking its way, it just keeps lane at highest velocity while not breaking the speed limit.

If there is a car in front of it and changing to an adjacent lane is feasible, then do it. If changing lane is not feasible, slow down to keep distance with the blocking car until lane change is possible. When multiple lanes are feasible for change, choose the lane that has more space ahead of you.

I reason that the middle lane has two possible lanes to switch to. When the middle lane is wide open, switch to it even if the current lane is also wide open.

Here the logic is so clear and simple, I do not see a reason to design a cost function and a FSM

##### Trajectory Generator
Spline is a piecewise polynomial curve that guarantees the curve goes through all the waypoints. I used 5 anchor points to define a spline curve in each iteration.

The problem is the further away each anchor points are to each other, the smoother the curve is. If the behavior is change lane and the car velocity is low, the car may not be able to change lane immediately because the trajectory is short and the spline is so smooth that it has a large portion in the current lane. The consequence is the car will consider itself being in the intended lane while it'll take it a few more iterations to be there. Thus, the car it thinks that is in front of it is not the car actually in front of it. That may cause collision.

In that case I set the distance between anchor points smaller. It has its own flaw. The turn the car takes when changing lane is abrupt and may create intolerable jerk.
