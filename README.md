# Overview
This repository contains the code for the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## Particle Filter Implementation
### Inititalization
I chose to use 50 particles in my implementation.  I found that I could go as low as 10 particles and still localize successfully. `particle_filter.cpp:27`

All particles are initialized across a normal distribution with standard deviations provided to the `init()` function. To generate the normal distribution `default_random_engine` and `normal_distribution` defined in the c++ `<random>` header. A vector of weights is also maintained.  All weights are initialized to 1.  `particle_filter.cpp:33-69`

### Prediction
Particle prediction is calculated in `ParticleFilter::prediction()`, lines 72-122.  I loop through all particles and predict the new position using the time elapsed, `delta_t`, `velocity`, and `yaw_rate`, being careful to avoid a division by zero when the yaw rate is zero.  Again, noise is applied using a normal distribution with the passed in standard deviations.

### updateWeights
Each particle represents our possible location.  The chance that it actually is our location is the probability stored in the particle's `weight`.  In `ParticleFilter::updateWeights()`, I loop through all particles and update each weight using a multi-variate Gaussian function I defined in `helper_functions.h`.

Initially, I set the particle's weight to 1.0 and allocate a couple of list to hold associations and sensor coordinates for debugging. lines 160-168.  If we were actually at the particle's location we would only be able to sense landmarks within the `sensor_range` of the particle.  Lines 173-182 fills a list of landmarks within this range.  Observations made by the Car or Robot are made within the coordinate system of the Car/Robot.  Since we will be associating them with landmarks in the map's coordinate system a transformation is required.  Observations are transformed by lines 199-203.  In Lines 210-231 I find the closest landmark with `sensor_range` to the observation.  I calculate and apply the multi-variate Gaussian weight in lines 243-251. 

### Resampling
Now that the probability that the particle is our position has been calculated a resampling of all particles is done.  A new particle list is obtained by performing a discrete distribution using all the weights obtained in the `updateWeights` phase. Lines 266-296.

