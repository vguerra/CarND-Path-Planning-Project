# Path Planning

* [The Goal](#the-goal)
* [Rubric](#rubric)
* [Code](#code)
* [Path Planning](#pid-controller)

---

## The Goal

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

---

## [Rubric](https://review.udacity.com/#!/projects/318/rubric)
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Code

The project is organized as follows:
* [`data`](https://github.com/vguerra/CarND-Path-Planning-Project/tree/master/data): Folder containing map data. 
* [`src`](https://github.com/vguerra/CarND-Path-Planning-Project/tree/master/src): Folder containing all source code of the project. Containing:
    - `main.cpp`: Main workflow of comunication between path planning generator and simulator.
    - `helpers.cpp`: A set of helper functions.
    - `cost-functions.cpp`: All supported cost functions and logic that computes best lane based on costs.
    - `paths.cpp`: Feasibility of trajectories.
---

### Path Planning

The Simulator sends messages to the backend containing information about current state of the car and information about other cars on the road. The backend in response, sends back a set of points that represent the path the car shuold follow.

Here we describe the workflow we follow to produce the path the car follows in the simulator.

First step is:

#### Preprocessing of sensor data.

At each step, the simulator informs us about the current state of the car and state of other vehicules through the captured sensor data. For each of the cars we get the following information: 

* car's unique ID
* car's x position in map coordinates
* car's y position in map coordinates
* car's x velocity in m/s
* car's y velocity in m/s
* car's s position in frenet coordinates
* car's d position in frenet coordinates. 

Using all this data, we extract information that is key to take decision during the generation of the car's path. Mainly, this information is valuable during computation of cost of all possible paths our car could take.

The [extraction process](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/paths.cpp#L52) computes the following information *per lane*:

* The *absolute distance* to the closest car.
* The *distance* to the closest car ahead of us.
* The *velocity* of the closest car ahead of us.
* The car id of the car closest to us.
* The car id of the closest car ahead of us.
* All car ids found ( again, per lane ).

All this information is then fed to the next step:

#### Choosing best lane possible:

All logic of this step is encapsulated in the [`compute_best_lane`](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/cost-functions.cpp#L51) function. The general idea is to come up with the lane that minimizes a cost function computed per lane.

* Given the current lane the car drives on, we select the possible target lanes. In order to reduce Jerk when producing the car's path in a later step, we only allow the car to change to an adjacent lane, meaning that if the car finds itself on lane 0, it can only move to lane 1.

* For each possible lane ( including the current lane ) a cost is computed. This cost represents how *expensive* it would be for the car to drive on that lane and it can be mathematically expressed as follows: 

<p align="center">
 <img src="https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/images/cost-function.png" width="650">
</p>

We explain each of the terms:

##### [Cost of change of lane](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/cost-functions.cpp#L25):
Refers to term `CL(l)`. This function penalizes the change of lane. If the target lane happens to be the current lane, then cost is 0. 

Mathematical representation:

<p align="center">
 <img src="https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/images/change-of-lane.png" width="300">
</p>

where *l* represents the current lane and *l<sub>t</sub>* the target lane.

If we plot it's behaviour:

<p align="center">
 <img src="https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/images/change-of-lane-graph.png" width="500">
</p>

##### [Cost of closest car](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/cost-functions.cpp#L31):
Refers to term `CC(l)`. Penalizes short distances to cars ahead of us. The closer we get to other cars, the higher the cost.

Mathematical representation:

<p align="center">
 <img src="https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/images/closest-car.png" width="400">
</p>

If we plot it's behaviour:

<p align="center">
 <img src="https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/images/closest-car-graph.png" width="500">
</p>


##### [Cost of slowest car](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/cost-functions.cpp#L37):
Refers to term `SC(l)`. Penalizes having slow cars ahead. The slower the car ahead of us drives, the higher the cost.

#### [Cost of colission](https://github.com/vguerra/CarND-Path-Planning-Project/blob/master/src/cost-functions.cpp#L43):
Refers to term `C(l)`. Binary cost function that penalizes a colission.

Now, the weights that multiply each function are as follow:

* Weight for cost of Change of Lane = 4.
* Weight for cost of Closest Car = 18.
* Weight for cost of Slowest Car = 8.
* Weight for Cost of Colission = 100;

As we can see, Weight for colission is high. If there is a possible colission when car transitions to the target lane, we avoid it. Then, weight for Closest Car cost is a bit more than double the Slowest Car's weights because we want to incentivate lane change if we get too close to a car. Finally, Change of Lane's weight cost is the lowest one of all since it only comes into play when the car is alone on the road to avoid changes of lane when differences between lane costs are small.

---

## Output Video
