/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  num_particles = 100;  // Set the number of particles
  particles.resize(num_particles);
  weights.resize(num_particles);
  for (unsigned particle_i = 0; particle_i < particles.size(); particle_i++) {
    particles[particle_i].id = particle_i;
    particles[particle_i].x = dist_x(gen);
    particles[particle_i].y = dist_y(gen);
    particles[particle_i].theta = dist_theta(gen);
    particles[particle_i].weight = 1;
    particles[particle_i].sense_x.clear();
    particles[particle_i].sense_y.clear();
    particles[particle_i].associations.clear();

    weights[particle_i] = 1;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  for (unsigned particle_i = 0; particle_i < particles.size(); particle_i++) {
    double temp_x, temp_y;
    double temp_theta = particles[particle_i].theta + yaw_rate * delta_t;
    if (fabs(yaw_rate) > 0.0001) {
      temp_x = particles[particle_i].x + 
               velocity / yaw_rate * (sin(temp_theta) - sin(particles[particle_i].theta));
      temp_y = particles[particle_i].y + 
               velocity / yaw_rate * (-cos(temp_theta) + cos(particles[particle_i].theta));
    } else {
      temp_x = particles[particle_i].x + velocity * cos(temp_theta) * delta_t;
      temp_y = particles[particle_i].y + velocity * sin(temp_theta) * delta_t;
    }

    std::normal_distribution<double> dist_x(temp_x, std_pos[0]);
    std::normal_distribution<double> dist_y(temp_y, std_pos[1]);
    std::normal_distribution<double> dist_theta(temp_theta, std_pos[2]);

    particles[particle_i].x = dist_x(gen);
    particles[particle_i].y = dist_y(gen);
    particles[particle_i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for (unsigned particle_i = 0; particle_i < particles.size(); particle_i++) {
    double car_x = particles[particle_i].x;
    double car_y = particles[particle_i].y;
    double car_theta = particles[particle_i].theta;

    // Debug vectors
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    associations.clear();
    sense_x.clear();
    sense_y.clear();

    particles[particle_i].weight = 1;
    for (unsigned observation_i = 0; observation_i < observations.size(); observation_i++) {
      // Car coordinates
      double observation_x_car = observations[observation_i].x;
      double observation_y_car = observations[observation_i].y;

      // Transform to global coordinates
      double observation_x_global = car_x + observation_x_car * cos(car_theta) - observation_y_car * sin(car_theta);
      double observation_y_global = car_y + observation_x_car * sin(car_theta) + observation_y_car * cos(car_theta);

      // Nearest neighbour
      double min_dist = 0;
      int index = 0;
      int landmark_id = 0;
      for (unsigned landmark_index = 0; landmark_index < map_landmarks.landmark_list.size(); landmark_index++) {
        double temp_dist = dist(
          observation_x_global, observation_y_global,
          map_landmarks.landmark_list[landmark_index].x_f, map_landmarks.landmark_list[landmark_index].y_f);
        if (landmark_index == 0 || temp_dist < min_dist) {
          min_dist = temp_dist;
          index = landmark_index;
          landmark_id = map_landmarks.landmark_list[landmark_index].id_i;
        }
      }
      // Debug vectors
      associations.push_back(landmark_id);
      sense_x.push_back(observation_x_global);
      sense_y.push_back(observation_y_global);

      // Computing weight
      double base = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

      double temp_x = observation_x_global - map_landmarks.landmark_list[index].x_f;
      temp_x = temp_x * temp_x / (2 * std_landmark[0] * std_landmark[0]);

      double temp_y = observation_y_global - map_landmarks.landmark_list[index].y_f;
      temp_y = temp_y * temp_y / (2 * std_landmark[1] * std_landmark[1]);

      particles[particle_i].weight *= base * exp(-(temp_x + temp_y));
    }
    // Debug vectors
    SetAssociations(particles[particle_i], associations, sense_x, sense_y);

    weights[particle_i] = particles[particle_i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<> dist(weights.begin(), weights.end());

  std::vector<Particle> backup_particles = particles;
  for (unsigned particle_index = 0; particle_index < particles.size(); particle_index++) {
    particles[particle_index] = backup_particles[dist(gen)];
  }
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}