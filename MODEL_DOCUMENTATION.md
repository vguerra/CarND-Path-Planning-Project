# Path Planning

* [The Goal](#the-goal)
* [Rubric](#rubric)
* [Code](#code)
* [Path Planning](#pid-controller)
* [Output Video](#output-video)

---

## The Goal

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

---

## [Rubric](https://review.udacity.com/#!/projects/318/rubric)
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Code

The project is organized as follows:
* `src` folder containing all source code of the project. Containing:
    - `main.cpp`: Main workflow of comunication between path planning generator and simulator.
    - `calibration.py`: Implementation of calibration of car's camera.
    - `colorthreshold.py`: Threshold of color and gradient for images.
    - `line.py`: Class that encapsulates all information about found Lanes.
    - `linefinder.py` : Implements the algorithm to find lanes.
    - `transform.py` : Perspective transformation implementation.
    - `config.py`: Configuration parameters for the entire pipeline.
    - `util.py`: Utilities for parsing command line arguments and debugging.


---

### Path Planning

Now we are going to describe the process of generating the desired path the car should follow in the simulator. The following diagram gives the bigger picture:


Now we describe each of the blocks in the diagram:

#### Preprocessing of sensor data.

At each step, the simulator informs us about the state of other vehicules through the captured sensor data. For each of the cars we get the following information: 

* car's unique ID
* car's x position in map coordinates
* car's y position in map coordinates
* car's x velocity in m/s
* car's y velocity in m/s
* car's s position in frenet coordinates
* car's d position in frenet coordinates. 

Using all this data, we extract information that is key to take decision during the generation of the car's path. Mainly, this information is valuable during computation of cost of all possible paths our car could take.

The extraction process computes the following:

* For each lane, what's the *absolute distance* to the closest car: Since absolute distance is computed, it can happen that the closest car is ahead or behind of us.
* For each lane, what's the *distance* to the closest car ahead of us.
* For each lane, what's the *velocity* of the closest car ahead of us.
* 

#### Preprocessing sensor data


---

## Output Video
