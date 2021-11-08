//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include <fstream>
#include "slam.h"
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "ros/ros.h"
#include "config_reader/config_reader.h"
#include "vector_map/vector_map.h"

namespace slam {

constexpr const int32_t NUM_PIXELS = 200;
constexpr const double_t LIDAR_STD = 0.2;
constexpr const double_t MIN_TRANS = 0.05;
constexpr const double_t MIN_ROT = 1 * M_PI / 180.0;

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});
CONFIG_FLOAT(init_x, "init_x");
CONFIG_FLOAT(init_y, "init_y");
CONFIG_FLOAT(init_theta, "init_theta");


void PrintPose(Pose pose, std::string name){
  ROS_INFO("pose(%s) = (x = %f, y = %f, theta = %f", name.c_str(), pose.x, pose.y, pose.theta);
}

SLAM::SLAM() : prev_update_loc_(0,0), prev_update_angle_(0) {}


void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) {
  // Return the latest pose estimate of the robot.
  if (poses_.size() > 0) {
    // Get the last and second to last pose
    const Pose& last_pose = poses_.back();
    *loc = last_pose.t_map;
    *angle = last_pose.theta_map;
  }
}

void SLAM::GetObservedPointCloud(const std::vector<double>& ranges,
                                 double_t range_min,
                                 double_t range_max,
                                 double_t angle_min,
                                 double_t angle_max,
                                 scan_ptr& obs_scan_ptr,
                                 Pose& pose) {

  const int num_ranges = static_cast<int>(ranges.size());
  scan_ptr obs_scan(new std::vector<Eigen::Vector2f>());
  // Just reserve since since we might toss out some ranges due to out of range.
  obs_scan->reserve(num_ranges);

  const double_t angle_inc = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++) {
    // Only include range value less than sensor range.
    double_t current_angle;
    if (ranges[i] < 10.0) {
      current_angle = i * angle_inc + angle_min;
      obs_scan->push_back(
        Eigen::Vector2f(ranges[i] * cos(current_angle), ranges[i] * sin(current_angle)));
    }
  }
  obs_scan_ptr = obs_scan;
}


double_t SampleLikelihood(double_t val, double_t mean, double_t std) {
  return (1 / (std * sqrt(2 * M_PI))) * exp(-0.5 * pow((val - mean) / std, 2.0));
}

// create the rasterized cost table based on prev_scan points
void SLAM::CreateCostTable(const std::vector<Eigen::Vector2f>& prev_scan,
                           std::shared_ptr<Eigen::MatrixXd>& cost_table_ptr) {

  /*
    (-10, 10) ...  (10, 10)
        .              .
        .              .
        .              .
    (-10, -10) ... (10, -10)     
  */
  double_t resolution = NUM_PIXELS / 20.0;  // 100 px / (10 * 2)m

  cost_table_ptr = std::make_shared<Eigen::MatrixXd>(NUM_PIXELS, NUM_PIXELS);
  Eigen::MatrixXd& cost_table = *cost_table_ptr;
  cost_table.setZero();
  
  for (const auto& point : prev_scan) {

    // ROS_INFO("adjusted point = (%f, %f)", x, y);
    if ((point.x() <= 10.0 && point.x() >= -10.0)
        && (point.y() <= 10.0 && point.y() >= -10.0)) {

      // Loop over all pixels in the table and create the rasterized conditional
      // conditional probability version. For each point in the scan calculate
      // the probability of observing the point at that different pixel location.
      for (int row = 0; row < cost_table.rows(); row++) {
        // y coordinate of the point in m
        double_t y_loc = -((1 / resolution) * (row + 1)) + 10.0;
        double_t prob_y =  SampleLikelihood(y_loc, point.y(), LIDAR_STD);
        for (int col = 0; col < cost_table.cols(); col++) {
          // x coordinate of the point in m
          double_t x_loc = ((1 / resolution) * col) - 10.0;
          // Sample from pdf for value of finding this point at this (x_loc, y_loc).
          double_t prob_x = SampleLikelihood(x_loc, point.x(), LIDAR_STD);
          cost_table(row, col) += std::max(0.0, prob_x) * std::max(prob_y, 0.0);
        }
      }
    }
  }
  // std::ofstream file("test.txt");
  // if (file.is_open())
  // {
  //   file << cost_table << '\n';
  // }
  // file.close();
}


// Observation Likelihood: return log likelihood of how likely it is 
// that a given x, y, theta and curr_scan are correlated to cost_table
/*
 *
 * @param cost_table: cost table created from the previous pose
 * @param curr_scan: the scan of the current pose
 */
PoseObservation SLAM::FindObservationLogLikelihood(
  double_t x, double_t y, double_t theta, const Pose prev_pose,
  Eigen::MatrixXd cost_table, const std::vector<Eigen::Vector2f>& curr_scan, int c, bool plot) {

  double_t observation_liklihood = 0.0;
  double_t resolution = (double_t)NUM_PIXELS / 20.0;  // 100 px / (10 * 2)m

  // Translation and rotation to move point in new scan back to frame of previous scan.
  Eigen::Vector2f translation(x - prev_pose.x, y - prev_pose.y);
  double_t angle = (-(theta - prev_pose.theta));
  for (const auto& point : curr_scan) {

    // Now we need to transform this point from the frame of the candidate pose to the
    // frame of the previous pose.
    Eigen::Vector2f trans_point(
      (cos(angle) * point.x()) +  -(sin(angle) * point.y()) + translation.x(),
      (sin(angle) * point.x()) +  (cos(angle) * point.y()) + translation.y());

    // Since (0, 0) is in the upper left hand corner of the rasterized cost table, we
    // need to ensure the Cartesian to pixel transform is proper. For x, we just shift
    // the point over by half the x width.
    int x_loc = static_cast<int>(resolution * (trans_point.x() + 10));
    // point.y() \in [-10, 10]. Flip over y axis
    int y_loc = static_cast<int>(resolution * (-trans_point.y() + 10));

    // Ensure pixels are within the image
    x_loc = std::max(0, x_loc);
    x_loc = std::min(x_loc, NUM_PIXELS - 1);

    y_loc = std::max(0, y_loc);
    y_loc = std::min(y_loc, NUM_PIXELS - 1);

    observation_liklihood += cost_table(y_loc, x_loc);
    if (plot) {
      cost_table(y_loc, x_loc) += 100;
    }
  }
  if (plot) {
    ROS_INFO("OBS: %.30f", observation_liklihood);
    std::string f("./logs/test-new-" + std::to_string(c_) + "X" + std::to_string(x) + "Y" + std::to_string(y) + "T" + std::to_string(theta) + "obs" + std::to_string(observation_liklihood) + ".txt");

    std::ofstream file(f);
    if (file.is_open())
    {
      file << cost_table << '\n';
    }
    file.close();
    ROS_INFO("%ld %ld", cost_table.cols(), cost_table.rows());
  }
  return PoseObservation(observation_liklihood, cost_table);
}

void SLAM::PrintPoses(){
  int i = 0;
  for (auto& pose : poses_){
    PrintPose(pose, std::to_string(i));
    i++;
  }
}

void SLAM::ObserveLaser(const std::vector<float>& ranges,
                        double_t range_min,
                        double_t range_max,
                        double_t angle_min,
                        double_t angle_max) {

   std::vector<double_t> ranges_double(ranges.begin(), ranges.end());
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  if (poses_.size() == 0) {
    return;
  // First ObserveLaser called with a single pose in poses_
  } else if (poses_.size() == 1) {
    // Add the scan to this pose
    GetObservedPointCloud(
      ranges_double, range_min, range_max, angle_min,
      angle_max, poses_.back().scan, poses_.back());

  // Compute Correlative Scan Matching with previous two poses
  } else {
    if (!poses_.back().scan) {

      auto start_time = ros::Time::now().toSec();

      GetObservedPointCloud(
        ranges_double, range_min, range_max, angle_min,
        angle_max, poses_.back().scan, poses_.back());
      ROS_INFO("SIZE 1: %ld", poses_.size());
      Pose& curr_pose = poses_[poses_.size() - 1];
      Pose prev_pose = poses_[poses_.size() - 2];

      std::shared_ptr<Eigen::MatrixXd> cost_table_ptr;
      CreateCostTable(*prev_pose.scan, cost_table_ptr);

      double_t x_width = 1.0;
      double_t y_width = 1.0;
      double_t theta_width = 45 * M_PI / 180.0;
      double_t resolution = 25.0;
      double_t x_inc = 2 * x_width / resolution;
      double_t y_inc = 2 * y_width / resolution;
      double_t theta_inc = 2 * theta_width / resolution; 

      double_t max_loglikelihood = 0.0;
      double_t best_x = curr_pose.x;
      double_t best_y = curr_pose.y;
      double_t best_theta = curr_pose.theta;

      double_t dx_odom = curr_pose.x - prev_pose.x;
      double_t dy_odom = curr_pose.y - prev_pose.y;
      double_t dtheta_odom = curr_pose.theta - prev_pose.theta;

      double_t std = sqrt(pow(dx_odom, 2) + pow(dy_odom, 2));
      std += abs(dtheta_odom);
      std *= 0.5;

      std::vector<double> x_probs, x_vals;
      x_probs.reserve(static_cast<int>(resolution));
      for (double_t x_i = -x_width + curr_pose.x; x_i <= x_width + curr_pose.x; x_i += x_inc) {
          x_probs.push_back(SLAM::MotionModelProb(x_i, curr_pose.x, std));
          x_vals.push_back(x_i);
      }
      Eigen::MatrixXd x_probs_mat(1, x_probs.size());
      x_probs_mat.rowwise() = Eigen::Map<Eigen::RowVectorXd>(x_probs.data(), x_probs.size());

      std::vector<double> y_probs, y_vals;
      y_probs.reserve(static_cast<int>(resolution));
      for (double_t y_i = y_width + curr_pose.y; y_i >= -y_width + curr_pose.y; y_i -= y_inc) {
          y_probs.push_back(SLAM::MotionModelProb(y_i, curr_pose.y, std));
          y_vals.push_back(y_i);
      }
      Eigen::MatrixXd y_probs_mat(1, y_probs.size());
      y_probs_mat.rowwise() = Eigen::Map<Eigen::RowVectorXd>(y_probs.data(), y_probs.size());

      std::vector<double> theta_probs, theta_vals;
      theta_probs.reserve(static_cast<int>(resolution));
      for (double_t theta_i = -theta_width + curr_pose.theta; theta_i <= theta_width + curr_pose.theta; theta_i += theta_inc) {
          theta_probs.push_back(SLAM::MotionModelProb(theta_i, curr_pose.theta, std));
          theta_vals.push_back(theta_i);
      }

      Eigen::MatrixXd xy_probs = y_probs_mat.transpose() * x_probs_mat;
      ROS_INFO("x_start: %f, x_end: %f", x_vals.front(), x_vals.back());
      ROS_INFO("y_start: %f, y_end: %f", y_vals.front(), y_vals.back());
      int c = 0;
      for (int x = 0; x < xy_probs.cols(); x++) {
        double_t x_val = x_vals[x];

        for (int y = 0; y < xy_probs.rows(); y++) {
          double_t y_val = y_vals[y];
          //double_t xy_prob = xy_probs(y, x);

          for (int theta = 0; theta < static_cast<int>(theta_probs.size()); theta++) {
            double_t theta_val = theta_vals[theta];
            //double_t log_likelihood = xy_prob * theta_probs[theta];
            double_t log_likelihood = 1.0;
            auto output = FindObservationLogLikelihood(
              x_val, y_val, theta_val, prev_pose, *cost_table_ptr, *curr_pose.scan, c);

            log_likelihood *= output.obsliklihood;
            if (log_likelihood > max_loglikelihood) {
              max_loglikelihood = log_likelihood;
              best_x = x_vals[x];
              best_y = y_vals[y];
              best_theta = theta_vals[theta];
            }
            c += 1;
          }

        }
      }
      ROS_INFO("curr_x: %f, new_x: %f", curr_pose.x, best_x);
      ROS_INFO("curr_y: %f, new_y: %f", curr_pose.y, best_y);
      ROS_INFO("curr_theta: %f, new_theta: %f", curr_pose.theta, best_theta);

      curr_pose.x = best_x;
      curr_pose.y = best_y;
      curr_pose.theta = best_theta;

      // With optimized pose, update map coordinates.
      Eigen::Vector2f trans = Eigen::Vector2f(curr_pose.x, curr_pose.y) - Eigen::Vector2f(prev_pose.x, prev_pose.y);
      Eigen::Rotation2Df rot(-prev_pose.theta);
      Eigen::Rotation2Df map_rot(prev_pose.theta_map);
      curr_pose.t_map = prev_pose.t_map + (map_rot * (rot * trans));
      curr_pose.theta_map = prev_pose.theta_map + (curr_pose.theta - prev_pose.theta);

      PrintPoses();

      FindObservationLogLikelihood(
        best_x, best_y, best_theta, prev_pose, *cost_table_ptr, *curr_pose.scan, c, true);

      auto end_time = ros::Time::now().toSec();
      ROS_INFO("time: %f", end_time - start_time);
    }    
  }
}

void SLAM::ObserveOdometry(const Eigen::Vector2f& odom_loc, const double_t odom_angle) {

  // if new pose has moved or rotated more than threshold
  if (((prev_update_loc_ - odom_loc).norm() >= MIN_TRANS) || 
                    (abs(prev_update_angle_ - odom_angle) >= MIN_ROT)) {

    // If this is the first pose or the last pose has a scan which has been added, add
    // this new odometry info as a pose.
    if (poses_.size() == 0 || poses_.back().scan) {
      // create a new pose with odometry information
      Pose pose(odom_loc.x(), odom_loc.y(), odom_angle);
      // If this is the first pose, set the map frame to 0
      if (poses_.size() == 0) {
        pose.t_map = Eigen::Vector2f(0.0, 0.0);
        pose.theta_map = 0.0;
      } else {
        // Not the first pose, find the map coordinates using the previous pose.
        const Pose prev_pose = poses_.back();
        Eigen::Vector2f trans = odom_loc - Eigen::Vector2f(prev_pose.x, prev_pose.y);
        Eigen::Rotation2Df rot(-prev_pose.theta);
        Eigen::Rotation2Df map_rot(prev_pose.theta_map);
        pose.t_map = prev_pose.t_map + map_rot * (rot * trans);
        pose.theta_map = prev_pose.theta_map + (odom_angle - prev_pose.theta);
      }
      poses_.push_back(pose);
  
    // The scan for the last pose has not been filled in yet, update that pose with the
    // latest odometry information.
    } else {

      // // Get the previous and last poses.
      std::vector<Pose>::const_iterator prev_pose = poses_.end();
      prev_pose = std::prev(prev_pose, 2);

      Pose& last_pose = poses_.back();

      last_pose.x = odom_loc.x();
      last_pose.y = odom_loc.y();
      last_pose.theta = odom_angle;

      // // Update the map coordinates for the last pose. 
      Eigen::Vector2f trans = odom_loc - Eigen::Vector2f(prev_pose->x, prev_pose->y);
      Eigen::Rotation2Df rot(-prev_pose->theta);
      Eigen::Rotation2Df map_rot(prev_pose->theta_map);
      last_pose.t_map = prev_pose->t_map + map_rot * (rot * trans);
      last_pose.theta_map = prev_pose->theta_map + (odom_angle - prev_pose->theta);
    }
    // Hold on to this odometry information for the next update
    prev_update_loc_ = odom_loc;
    prev_update_angle_ = odom_angle;
    ROS_INFO("SIZE 2: %ld", poses_.size());
  }
}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
  std::vector<Eigen::Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses and their
  // respective scans.
  if (poses_.size() > 0) {
    for (const auto& curr_pose : poses_) {
      std::vector<Eigen::Vector2f> trans_scan;
      
      for (const auto& point : *curr_pose.scan) { 
        // The scan points are in the reference frame of the pose. Transform the points
        // into the current odometry frame.
        Eigen::Vector2f p1(
          (cos(curr_pose.theta_map) * point.x()) +  -(sin(curr_pose.theta_map) * point.y()) + curr_pose.t_map.x(),
          (sin(curr_pose.theta_map) * point.x()) +  (cos(curr_pose.theta_map) * point.y()) + curr_pose.t_map.y());
        trans_scan.push_back(p1);
      }
      map.reserve(trans_scan.size());
      map.insert(map.end(), trans_scan.begin(), trans_scan.end());
    }
  }
  return map;
}
}  // namespace slam
