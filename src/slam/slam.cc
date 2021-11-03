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

#include "slam.h"

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

using namespace math_util;

namespace slam {

constexpr const int NUM_PIXELS = 200;
const int debug_odom = false;

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});
CONFIG_FLOAT(init_x, "init_x");
CONFIG_FLOAT(init_y, "init_y");
CONFIG_FLOAT(init_theta, "init_theta");

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    curr_odom_loc_(0,0),
    curr_odom_angle_(0),
    curr_time_(0),
    prev_time_(0),
    del_time_(0),
    del_odom_loc_(0,0),
    prev_odom_vel2f_(0,0),
    odom_vel2f_(0,0),
    odom_accel2f_(0,0),
    odom_vel_(0),
    odom_accel_(0),
    del_odom_angle_(0),
    odom_omega_(0),
    prev_update_loc_(0,0),
    prev_update_angle_(0) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  if (poses_.size() != 0) {
    Eigen::Vector2f del_trans = Eigen::Vector2f(poses_.back().x, poses_.back().y);
    *loc = del_trans;
    *angle = poses_.back().theta;
  }
}

void SLAM::UpdateOdometry(const Eigen::Vector2f& odom_loc,
                                    const float odom_angle){
  if (!odom_initialized_) {
    curr_odom_angle_ = odom_angle;
    curr_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

  curr_odom_loc_ = odom_loc;
  curr_odom_angle_ = odom_angle;
  curr_time_ = ros::Time::now().toSec();
  del_time_ = curr_time_ - prev_time_;
  del_odom_loc_ = curr_odom_loc_ - prev_odom_loc_;
  odom_vel2f_ = GetOdomVel2f();
  odom_accel2f_ = GetOdomAccel2f();
  odom_vel_ = Vel2fToVel();
  odom_accel_ = Accel2fToAccel();
  del_odom_angle_ = curr_odom_angle_ - prev_odom_angle_;
  odom_omega_ = del_odom_angle_ / del_time_;

  if (debug_odom) {
    ROS_INFO("----------------------------");
    ROS_INFO("prev_time_ = %f", prev_time_);
    ROS_INFO("curr_time_ = %f", curr_time_);
    ROS_INFO("del_time_ = %f", del_time_);

    ROS_INFO("prev_odom_loc_ = (%f, %f)", prev_odom_loc_.x(), prev_odom_loc_.y());
    ROS_INFO("curr_odom_loc_ = (%f, %f)", curr_odom_loc_.x(), curr_odom_loc_.y());
    ROS_INFO("del_odom_loc_ = (%f, %f)", del_odom_loc_.x(), del_odom_loc_.y());

    ROS_INFO("prev_odom_vel2f_ = (%f, %f)", prev_odom_vel2f_.x(), prev_odom_vel2f_.y());
    ROS_INFO("odom_vel2f_ = (%f, %f)", odom_vel2f_.x(), odom_vel2f_.y());
    ROS_INFO("odom_vel_ = %f", odom_vel_);

    ROS_INFO("odom_accel2f_ = (%f, %f)", odom_accel2f_.x(), odom_accel2f_.y());
    ROS_INFO("odom_accel_ = %f", odom_accel_);

    ROS_INFO("prev_odom_angle_ = %f", prev_odom_angle_);
    ROS_INFO("curr_odom_angle_ = %f", curr_odom_angle_);
    ROS_INFO("del_odom_angle_ = %f", del_odom_angle_);
    ROS_INFO("odom_omega_ = %f", odom_omega_);

    ROS_INFO("prev_update_loc_ = (%f, %f)", prev_update_loc_.x(), prev_update_loc_.y());
    ROS_INFO("prev_update_angle_ = %f", prev_update_angle_);
  }

  prev_time_ = curr_time_;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  prev_odom_vel2f_ = odom_vel2f_;         
}

void SLAM::GetObservedPointCloud(const std::vector<float>& ranges,
                                 float range_min,
                                 float range_max,
                                 float angle_min,
                                 float angle_max,
                                 std::vector<Eigen::Vector2f>* obs_scan_ptr) {
  
  const int num_ranges = static_cast<int>(ranges.size());
  std::vector<Eigen::Vector2f>& obs_scan = *obs_scan_ptr;
  obs_scan.reserve(num_ranges);

  const float angle_inc = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++) {
    // Only include range value less than sensor range.
    float current_angle;
    if (ranges[i] < 10.0) {
      current_angle = i * angle_inc + angle_min;
      obs_scan.push_back(
        Eigen::Vector2f(ranges[i] * cos(current_angle), ranges[i] * sin(current_angle)));
    }
  }
}

float_t SampleNormalDensity(float_t val, float_t mean, float_t std_dev) {
  return 0.5 * (1 + erf((val - mean) / (sqrt(2) * std_dev)));
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
  float resolution = NUM_PIXELS / 20.0;  // 100 px / (10 * 2)m

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
        float y_loc_next = -((1 / resolution) * row) + 10.0;
        float y_loc = -((1 / resolution) * (row + 1)) + 10.0;
        float prob_y = SampleNormalDensity(y_loc_next, point.y(), 0.3);
        prob_y -= SampleNormalDensity(y_loc, point.y(), 0.3);
        for (int col = 0; col < cost_table.cols(); col++) {
          // x coordinate of the point in m
          float x_loc = ((1 / resolution) * col) - 10.0;
          float x_loc_next = ((1 / resolution) * (col + 1)) - 10.0;
          // Sample from pdf for value of finding this point at this (x_loc, y_loc).
          // Use sensor STD of 0.15m.
          float prob_x = SampleNormalDensity(x_loc_next, point.x(), 0.3);
          prob_x -= SampleNormalDensity(x_loc, point.x(), 0.3);
          cost_table(row, col) += std::max(float(0.0), prob_x) * std::max(prob_y, float(0.0));
        }
      }
    }
  }
}

// Observation Likelihood: return log likelihood of how likely it is 
// that a given x, y, theta and curr_scan are correlated to cost_table
float_t SLAM::FindObservationLogLikelihood(
  float x, float y, float theta, const Pose& prev_pose,
  const Eigen::MatrixXd& cost_table, const std::vector<Eigen::Vector2f>& curr_scan) {

  float observation_liklihood = 0.0;
  float resolution = NUM_PIXELS / 20.0;  // 100 px / (10 * 2)m

  // Translation and rotation to move point in new scan back to frame of previous scan.
  Eigen::Vector2f translation(x - prev_pose.x, y - prev_pose.y);
  Eigen::Rotation2Df rot(-(theta - prev_pose.theta));
  for (const auto& point : curr_scan) {
    Eigen::Vector2f trans_point = rot * (point - translation);
    // Since (0, 0) is in the upper left hand corner of the rasterized cost table, we
    // need to ensure the Cartesian to pixel transform is proper. For x, we just shift
    // the point over by half the x width.
    int x_loc = static_cast<int>(resolution * (trans_point.x() + 10));
    x_loc = std::min(0, x_loc);
    x_loc = std::max(x_loc, NUM_PIXELS - 1);

    // point.y() \in [-10, 10]. Flip over y axis
    int y_loc = static_cast<int>(resolution * (-trans_point.y() + 10));
    y_loc = std::min(0, y_loc);
    y_loc = std::max(y_loc, NUM_PIXELS - 1);
    
    observation_liklihood += cost_table(y_loc, x_loc);
  }

  return observation_liklihood;
}

// Motion Model: return log likelihood of how likely it is that a given x, y, theta
// exits given curr_pose.
float_t SLAM::FindMotionModelLogLikelihood(float x1,
                                           float x2, 
                                           float y1,
                                           float y2,
                                           float theta1,
                                           float theta2,
                                           const Pose& curr_pose,
                                           const Pose& prev_pose) {

  float_t dx1 = x1 - prev_pose.x;
  float_t dx2 = x2 - prev_pose.x;
  float_t dy1 = y1 - prev_pose.y;
  float_t dy2 = y2 - prev_pose.y;
  float_t dtheta1 = theta1 - prev_pose.theta;
  float_t dtheta2 = theta2 - prev_pose.theta;

  float_t dx_odom = curr_pose.x - prev_pose.x;
  float_t dy_odom = curr_pose.y - prev_pose.y;
  float_t dtheta_odom = curr_pose.theta - prev_pose.theta;

  float_t std = sqrt(pow(dx_odom, 2) + pow(dy_odom, 2));
  std += 1.0 * abs(dtheta_odom);

  float_t dx_prob = SampleNormalDensity(dx2, dx_odom, std);
  dx_prob -= SampleNormalDensity(dx1, dx_odom, std);

  float_t dy_prob = SampleNormalDensity(dy2, dx_odom, std);
  dy_prob -= SampleNormalDensity(dy1, dx_odom, std);

  float_t dtheta_prob = SampleNormalDensity(dtheta2, dx_odom, std);
  dtheta_prob -= SampleNormalDensity(dtheta1, dx_odom, std);

  return dx_prob * dy_prob * dtheta_prob;
}
float_t MotionModel(float val1,
                    float val2,
                    float mean,
                    float std) {

  float_t dx1 = val1 - mean;
  float_t dx2 = val2 - mean;

  return SampleNormalDensity(dx2, mean, std) - SampleNormalDensity(dx1, mean, std);
}

void SLAM::ObserveLaser(const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  std::vector<Eigen::Vector2f> observed_scan;
  GetObservedPointCloud(
    ranges, range_min, range_max, angle_min, angle_max, &observed_scan);

  // ObserveLaser called before there is a single pose in poses_
  if (poses_.size() == 0) {
    return;
  // First ObserveLaser called with a single pose in poses_
  } else if (poses_.size() == 1) {
    poses_.back().scan = observed_scan;
  // Compute Correlative Scan Matching with previous two poses
  } else {

    if (poses_.back().scan.size() == 0) {
      
      poses_.back().scan = observed_scan;
      Pose curr_pose = poses_.back();
      auto prev_pose_iter = poses_.end();
      prev_pose_iter = std::prev(prev_pose_iter, 2);
      Pose prev_pose = *prev_pose_iter;

      std::shared_ptr<Eigen::MatrixXd> cost_table_ptr;
      CreateCostTable(prev_pose.scan, cost_table_ptr);

      float x_width = 0.75;
      float y_width = 0.75;
      float theta_width = 30 * M_PI / 180.0;
      float resolution = 50.0;
      float x_inc = 2 * x_width / resolution;
      float y_inc = 2 * y_width / resolution;
      float theta_inc = 2 * theta_width / resolution; 

      float_t max_loglikelihood = 0.0;
      float best_x = curr_pose.x;
      float best_y = curr_pose.y;
      float best_theta = curr_pose.theta;

      float_t dx_odom = curr_pose.x - prev_pose.x;
      float_t dy_odom = curr_pose.y - prev_pose.y;
      float_t dtheta_odom = curr_pose.theta - prev_pose.theta;

      float_t std = sqrt(pow(dx_odom, 2) + pow(dy_odom, 2));
      std += 1.0 * abs(dtheta_odom);


      std::vector<float> x_probs, x_vals;
      x_probs.reserve(static_cast<int>(resolution));
      float dx_mean = curr_pose.x - prev_pose.x;
      for (float x_i = -x_width + curr_pose.x; x_i <= x_width + curr_pose.x; x_i += x_inc) {
          x_probs.push_back(MotionModel(x_i, x_i + x_inc, dx_mean, std));
          x_vals.push_back(x_i);
      }
      Eigen::MatrixXf x_probs_mat(1, x_probs.size());
      x_probs_mat.rowwise() = Eigen::Map<Eigen::RowVectorXf>(x_probs.data(), x_probs.size());

      std::vector<float> y_probs, y_vals;
      y_probs.reserve(static_cast<int>(resolution));
      float dy_mean = curr_pose.y - prev_pose.y;
      for (float y_i = -y_width + curr_pose.y; y_i <= y_width + curr_pose.y; y_i += y_inc) {
          y_probs.push_back(MotionModel(y_i, y_i + y_inc, dy_mean, std));
          y_vals.push_back(y_i);
      }
      Eigen::MatrixXf y_probs_mat(1, y_probs.size());
      y_probs_mat.rowwise() = Eigen::Map<Eigen::RowVectorXf>(y_probs.data(), y_probs.size());

      std::vector<float> theta_probs, theta_vals;
      theta_probs.reserve(static_cast<int>(resolution));
      float dtheta_mean = curr_pose.theta - prev_pose.theta;
      for (float theta_i = -theta_width + curr_pose.theta; theta_i <= theta_width + curr_pose.theta; theta_i += theta_inc) {
          theta_probs.push_back(MotionModel(theta_i, theta_i + theta_inc, dtheta_mean, std));
          theta_vals.push_back(theta_i);
      }

      Eigen::MatrixXf xy_probs = y_probs_mat.transpose() * x_probs_mat;
  
      for (int x = 0; x < xy_probs.cols(); x++) {
        for (int y = 0; y < xy_probs.rows(); y++) {
          float xy_prob = xy_probs(y, x);
          for (int theta = 0; theta < static_cast<int>(theta_probs.size()); theta++) {
            float_t log_likelihood = xy_prob * theta_probs[theta];
            if (log_likelihood > max_loglikelihood) {
              max_loglikelihood = log_likelihood;
              best_x = x_vals[x];
              best_y = y_vals[x];
              best_theta = theta_vals[theta];
            }
          }

        }
      }

      curr_pose.x = best_x;
      curr_pose.y = best_y;
      curr_pose.theta = best_theta;
    }    
  }  
}

void SLAM::ObserveOdometry(const Eigen::Vector2f& odom_loc, const float odom_angle) {
  
  UpdateOdometry(odom_loc, odom_angle);

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  float min_trans = 0.1;
  float min_rot = 1 * M_PI / 180.0;

  // if new pose has moved or rotated more than threshold
  if (((prev_update_loc_ - odom_loc).norm() >= min_trans) || 
                    (abs(prev_update_angle_ - odom_angle) >= min_rot)) {
    // ROS_INFO("Pose has moved above threshhold.");
    // if last pose has been previously added

    if (poses_.size() == 0 || poses_.back().scan.size() > 0) {
      // create a new pose with odometry
      Pose current_pose;
      current_pose.x = odom_loc.x();
      current_pose.y = odom_loc.y();
      current_pose.theta = odom_angle;
      current_pose.scan = std::vector<Eigen::Vector2f>();
      poses_.push_back(current_pose);
      // ROS_INFO("Created new last pose loc, theta.");
    } else {
      // update last pose with updated odometry
      poses_.back().x = odom_loc.x();
      poses_.back().y = odom_loc.y();
      poses_.back().theta = odom_angle;
      // ROS_INFO("Updated last pose loc, theta.");
    }
    prev_update_loc_ = odom_loc;
    prev_update_angle_ = odom_angle;
  } else {
    // ROS_INFO("Pose hasn't moved above threshhold.");
  }

}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
  std::vector<Eigen::Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for (const auto& curr_pose : poses_) {
    Eigen::Rotation2Df curr_pose_rot(curr_pose.theta);
    std::vector<Eigen::Vector2f> trans_scan;

    for (const auto& point : curr_pose.scan) { 
      // Rotate the point into the orientation of the pose then translate it by the
      // x and y location of the pose
      trans_scan.push_back(
        curr_pose_rot * point + Eigen::Vector2f(curr_pose.x, curr_pose.y));
    }
    map.reserve(trans_scan.size());
    map.insert(map.end(), trans_scan.begin(), trans_scan.end());
  }
  return map;
}
}  // namespace slam
