#**Path Planning** 
---

**Path Planning Project**

The goals / steps of this project are the following:

* Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic
* The car is able to drive at least 4.32 miles without any incident, which is described as the following
* The car drives according to the speed limit
* Max Acceleration and Jerk are not Exceeded
* Car does not have collisions
* The car stays in its lane, except for the time between changing lanes
* The car is able to change lanes
* Summarize the results with a written report


[//]: # (Image References)

[video1]: ./results/MPC_80mph.mp4 "MPC controller"

---
###Describe the model in detail
The path planning model includes three main parts, a finite state machine, a set of cost functions and a smooth path generator. Each time when the car receives the refreshed sensor fusion data and the state information of its self, the finite state machine will generate possible trajectories according to its current state. And then through the set of cost functions, the best trajectory with the lowest cost will be selected for the smooth path generator of which the outputs will be feeded into the simulator.
####1. The finite state machine
The finite state machine includes five states, as the following.

 * "KL" state, the "keep lane" state, has "PLCL" and "PLCR" states as its possible next states.
 * "PLCL" state, the "prepare to lane change to left" state, has "KL" and "LCL" states as its possible next states.
 * "PLCR" state, the "prepare to lane change to right" state, has "KL" and "LCR" states as its possible next states.
 * "LCL" state, the "lane change to left" state, has "KL" state as its possible next states.
 * "LCR" state, the "lane change to right" state, has "KL" state as its possible next states.

Every state has its self as its possible next state(FSM is on line 91 through 111 in vehicle.cpp). 

For "KL", "PLCL" and "PLCR" states, each of them will generate five trajectories with different s-direction speed, which includes "SPEEDUP", "HALF-SPEEDUP", "MAINTAIN", "HALF-SLOWDOWN" and "SLOWDOWN"("KL" trajectories generated on line 163 through 190 in vehicle.cpp, and "PLCL"/"PLCR" trajectories generated on line 199 through 236 in vehicle.cpp). 

For "LCL" and "LCR" states, besides the above mentioned speed options, they also have three different d position options for trajectories. As a result, each of "LCL" and "LCR" states will generate fifteen possible trajectories("LCL"/"LCR" trajectories generated on line 245 through 294 in vehicle.cpp).

####2. The cost functions
There are five cost functions used to evaluate the total cost of each possible trajectory which is generated from the above mentioned possible next state according to the current state.

 * "goal distance cost" Every time before evaluation, there will be a virtual s direction goal. Each trajectory must pay cost according to the distance between the goal and its farthest s position it can reach(the calculation is on line 22 through 26 in cost.cpp).
 * "inefficiency cost" Each trajectory has to pay cost of the difference between the preferred driving speed and the average moving speed of the lane belonging to its destination. Meanwhile, if a car is detected ahead within certain range on the lane intended, the trajectory also has to pay additional cost of the difference between the preferred driving speed and the speed of the detected car(the calculation is on line 38 through 63 in cost.cpp).
 * "collision cost" Each trajector has to pay cost for the s distance between the s position of its destination and the predicted s position of the nearest car in front of or behind its destination if either of them is decteted. The closer the more cost(the calculation is on line 77 through 88 in cost.cpp).
 * "s over acceleration cost" Each trajectory must pay cost of the difference between the s direction speed of its destination and the s direction speed of the current state(the calculation is on line 97 through 99 in cost.cpp).
 * "d over acceleration cost" Each trajectory must also pay cost of the difference between the d direction speed of its destination and the d direction speed of the current state(the calculation is on line 108 through 110 in cost.cpp).
 
Before summing up the total costs for each trajectory, the cost from each of the cost function will be differently weighted(the summing up on line 149 through 152 in cost.cpp).
 
####3. The smooth path generator
The spline tool is used to generate the path required for the simulator without jerk. In order to let the generated path connected to the previous path smoothly, the last two points of the previous path are used as the first two of the four waypoints needed by the tool. The other two way points come from the selected best trajectory, namely, the positions extended from the destination of the trajectory(the anchor points set on line 348 through line 388 in main.cpp).

Each time the path planner will feed the simulator 50 path points. The succeeding path points from the previous path will first be used, and the remaining vaccancies will be filled by the currently generated path(path points are added on line 415 through 441 in main.cpp). 

###Describe how the incidents will be prevented
 
####1. Driving according to the speed limit
When generating trajectories on each possible next state, the s direction speed larger than the preferred speed is not allowed(action function on line number 154 in vehicle.cpp). And the preferred speed is set under the speed limit of 50MPH.

The goal distance cost will push the path planner to select the trajectory with the fatest speed under the speed limit as possible.

####2. Max Acceleration and Jerk are not Exceeded
The maximum acceleration is set and is used for the generation of trajectories(action function on line number 153 in vehicle.cpp). 

The "s over acceleration cost" and "d over acceleration cost" will let the path planner select the trajectory with the least speed variation as possible.

####3. No collisions
A logistic function is used to calculate the cost of collision for each trajectory(collision_cost function on line 80 and 87 in cost.cpp). And the path planner will choose the trajectory with the least possibility of collision as possible.

####4. Staying in its lane, except for the time between changing lanes
When generating trajectories on the states of "KL", "PLCL" and "PLCR", the d position of the center the currently occupied lane is always used for the destination. As a result, the spline generated path should be able to maintain at the position around the center of the lane.

####5. Able to change lanes
The design of inefficiency cost is to encourage the path planner to jump into the state of preparing lane change instead of following a slow car. If possible, the path planner will choose a suitable lane change trajectory finally. Meanwhile, the jrajectory of following a car must maintain a higher collision cost than thoses which have destination without cars occupied ahead.







