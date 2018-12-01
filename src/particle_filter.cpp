/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <limits>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  //
  // set the number of particles.
  //
  num_particles = 50;

  //
  // create the list of particles with coordinates and
  // heading across a normal distribution
  //
  std::default_random_engine generator;
  std::normal_distribution<double> x_normal(x,     std[0]);
  std::normal_distribution<double> y_normal(y,     std[1]);
  std::normal_distribution<double> t_normal(theta, std[2]);

  for (unsigned int i = 0; i < num_particles; ++i)
  {
    //
    // initialize a particle and add to the
    // particles list.
    // x, y and theta are initialized to a
    // normal distributino with standard deviation
    // given by std[]
    //
    Particle p;
    p.id     = i;
    p.x      = x_normal(generator);
    p.y      = y_normal(generator);
    p.theta  = t_normal(generator);
    p.weight = 1;

    //
    // add our particle to the particle list
    //
    particles.push_back(p);

    //
    // we are also keeping a list of weights.
    //
    weights.push_back(1);
  }

  //
  // initialization complete
  //
  is_initialized = true;
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
  //
  // we need the generator to add the guassian noise.
  //
  std::default_random_engine generator;


  //
  // loop through all particles, calculate the new position
  // and add noise.
  //
  for (unsigned int i = 0; i < num_particles; ++i)
  {
    double theta_new;
    double x_new;
    double y_new;

    //
    // Calculate the new position based on the yaw rate and
    // the elapsed time.
    //
    if (yaw_rate == 0)
    {
      theta_new = particles[i].theta;
      x_new = particles[i].x + velocity * delta_t * cos(theta_new);
      y_new = particles[i].y + velocity * delta_t * sin(theta_new);
    }
    else
    {
      theta_new = particles[i].theta + yaw_rate * delta_t;
      x_new = particles[i].x + (velocity / yaw_rate) * ( sin(theta_new) - sin(particles[i].theta));
      y_new = particles[i].y + (velocity / yaw_rate) * ( cos(particles[i].theta) - cos(theta_new));
    }

    //
    // add noise by generating a random number in a normal distribution
    // around the new values.
    //
    std::normal_distribution<double> x_normal(x_new, std_pos[0]);
    std::normal_distribution<double> y_normal(y_new, std_pos[1]);
    std::normal_distribution<double> t_normal(theta_new, std_pos[2]);

    //
    // add them back to the particle data.
    //
    particles[i].x = x_normal(generator);
    particles[i].y = y_normal(generator);
    particles[i].theta = t_normal(generator);
  }
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

  //
  //  Find the predicted measurement that is closest to each observed measurement and assign the
  //  observed measurement to this particular landmark.
  //
  for (auto& o : observations)
  {
    double minDist = std::numeric_limits<double>::max();

    for (auto& p : predicted)
    {
      double dist = (p.x - o.x) * (p.x - o.x)  +  (p.y - o.y) * (p.y - o.y);

      if ( dist < minDist )
      {
        minDist = dist;
        o.id = p.id;
      }
    }
  }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
  //
  // Each particle represents a prediction of our location.
  // Update the weights of each particle using a mult-variate Gaussian distribution.
  //
  for (unsigned p = 0; p < num_particles; ++p)
  {
    particles[p].weight = 1.0;

    //
    // keep lists for particle associations with landmarks
    // useful for debugging.
    //
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    //
    // Populate a list of all map landmarks in sensor range.
    //
    std::vector<LandmarkObs> map_landmarks_in_range;
    for (auto& m : map_landmarks.landmark_list)
    {
      if (dist(m.x_f, m.y_f, particles[p].x, particles[p].y) < sensor_range)
      {
        LandmarkObs landmark = {m.id_i, m.x_f, m.y_f};

        map_landmarks_in_range.push_back(landmark);
      }
    }

    //
    // For each particle find the closest landmark observation
    // by calculating the euclidian distance of each and taking
    // the lowest value.  The final weight of the particle is
    // the product across all of these observations
    //
    std::vector<LandmarkObs> transformed;
    for (unsigned i = 0; i < observations.size(); ++i)
    {
      LandmarkObs obs = observations[i];

      //
      // the observations are in the vehicle's coordinate space.  We
      // need map coordinates.  Here we do the tranformation.
      //
      double rad = particles[p].theta;
      double transformed_x = obs.x * cos(rad) - obs.y * sin(rad) + particles[p].x;
      double transformed_y = obs.x * sin(rad) + obs.y * cos(rad) + particles[p].y;
      LandmarkObs landmark = {static_cast<int>(i), transformed_x, transformed_y};
      transformed.push_back(landmark);

      //
      // calculate the distance of each map landmark with the
      // transformed coordinates.  Keep track of the lowest
      // distance.
      //
      int lowest_dist  = std::numeric_limits<int>::max();
      int lowest_index = std::numeric_limits<int>::max();
      for (unsigned m = 0; m < map_landmarks_in_range.size(); ++m)
      {
        float landmark_x = map_landmarks_in_range[m].x;
        float landmark_y = map_landmarks_in_range[m].y;
        double distance = dist(landmark_x, landmark_y, transformed_x, transformed_y);

        if (distance < lowest_dist)
        {
          lowest_dist  = distance;
          lowest_index = m;
        }
      }

      //
      // the landmark with the lowest euclidian distance
      // is at index 'lowest_index'
      //
      float closest_landmark_x = map_landmarks_in_range[lowest_index].x;
      float closest_landmark_y = map_landmarks_in_range[lowest_index].y;
      int landmark_id = map_landmarks_in_range[lowest_index].id;

      //
      // add associations for debugging.
      //
      associations.push_back(landmark_id);
      sense_x.push_back(closest_landmark_x);
      sense_y.push_back(closest_landmark_y);

      //
      // use that landmark position to calculate the multivariant gaussian.
      //
      double mvg = multivariant_gaussian(transformed_x,      transformed_y,
                                         closest_landmark_x, closest_landmark_y,
                                         std_landmark[0],    std_landmark[1]);

      //
      // the particle's final weight is the product
      // of all multivariant_gaussians
      //
      particles[p].weight *= mvg;
    }

    //
    // dataAssociation.
    //
    //dataAssociation(map_landmarks_in_range, transformed);
    SetAssociations(particles[p], associations, sense_x, sense_y);


    weights[p] = particles[p].weight;
  }
}


void ParticleFilter::resample()
{
  //
  // Resample particles with replacement with probability proportional to their weight.
  //
  std::default_random_engine generator;

  //
  // Initialize the discrete_distributino with the weights.
  //
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());

  //
  // Create a new particles vector that we will populate
  // with the discrete_distribution.
  //
  std::vector<Particle> new_particles;

  //
  // Resample.
  //
  for (unsigned int i = 0; i < num_particles; ++i)
  {
    int index = distribution(generator);
    new_particles.push_back(particles[index]);
  }

  //
  // set the particle list to the resampled list.
  //
  particles = new_particles;
}


void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}


string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}


string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}


string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
