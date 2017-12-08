# Overview
This repository contains the implementation of MPC project in Udacity's Self-Driving Car Nanodegree. I implemented the controller by following the instructions provided in the course. My implementation meets the Success Criteria. 

## Project Introduction
The car is supposed to follow a desired path in the simulator. The controler is using the car state to compute the appropriate acceleration and steering angle.

## Success Criteria
My implementation meets the follwoing success criteria:
1. The vehicle successfully drives around the track.
2. No tire leaves the drivable portion of the track surface.
3. The car doesn't pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). 

## Running the Code
I used docker to setup the required environment. Once run_term2_docker.sh is executed, the main program can be built and ran by doing the following from the /src directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

## Implementation
I followed the instructions provided in the course to implement the controler. I have also used some hyperparameters to fine tune my implementation so my controller results in smoother motions:

1. I defined some weights to evalute the quality of the to be executed trajectory during the optimization phase.
2. I used try and error method and also some analysis to choose weights in the cost fuction. 

### The Model
The same model as described in the course was used in this project. Here is the model:
* x1 = (x0 + v0 * CppAD::cos(psi0) * dt)
* y1 = (y0 + v0 * CppAD::sin(psi0) * dt)
* psi1 = (psi0 + v0 * delta0 / Lf * dt)
* v1 = (v0 + a0 * dt)

### Timestep Length and Elapsed Duration (N & dt)
I used 40 points with the dt of 0.1sec.

### Polynomial Fitting and MPC Preprocessing
I preprocessed waypoints, the vehicle state to covert the state in the coordination of the ego car. Also, steering is converted to the range of -25 to +25 degree.

### Model Predictive Control with Latency
This was the most interesting part of the project. I used the same model as I used in MPC to predict the state of the car in 100ms and then used the predicted state as input to MPC.

### Cost Evaluation
I used the following weights to evaluate the quality of the trajectory:
* cte_w = 10;
* epsi_w = 2.0;
* v_w = 4.0;
* delta_w = 2500.0;
* acc_w = 5.0;
* delta_dot_w = 50.0;
* acc_dot_w = 40.0;
      
## Discussion
I followed the instructions to the course to implement MPC algorithm. Dealing with latency was quite interesting step and in result, the vehicle successfully drives around the track. 
