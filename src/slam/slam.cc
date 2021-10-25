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

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

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
    Eigen::Vector2f del_trans = Eigen::Vector2f(poses_.back().x - poses_.front().x, poses_.back().y - poses_.front().y);
    float_t del_rot = abs(poses_.back().theta - poses_.front().theta);
    
    *loc = Eigen::Vector2f(CONFIG_init_x, CONFIG_init_y) + del_trans;
    *angle = CONFIG_init_theta + del_rot;
    
    // ROS_INFO("GetPose loc = (%f, %f)", poses_.back().x, poses_.back().y);
    // ROS_INFO("GetPose angle = %f", poses_.back().theta);
  }
}

void SLAM::UpdateOdometry(const Vector2f& odom_loc,
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
  
  const int num_ranges = ranges.size();
  std::vector<Eigen::Vector2f>& obs_scan = *obs_scan_ptr;
  obs_scan.resize(num_ranges);

  // old code to use a subset of the ranges
  // const float index_inc = (ranges.size() / (num_ranges - 1));
  // const float angle_inc = (angle_max - angle_min) / (num_ranges - 1);

  const float angle_inc = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++){
    float current_angle = i * angle_inc + angle_min;
    obs_scan[i] = Vector2f(ranges[i] * cos(current_angle), ranges[i] * sin(current_angle));
  }

}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  std::vector<Eigen::Vector2f> observed_scan;
  GetObservedPointCloud(ranges, range_min, range_max, angle_min, angle_max, &observed_scan);
  
  // ObserveLaser called before there is a single pose in poses_
  if (poses_.size() == 0) {
    return;
  // First ObserveLaser called with a single pose in poses_
  } else if (poses_.size() == 1) {
    poses_.back().scan = observed_scan;
  // Compute Correlative Scan Matching with previous two poses
  } else {
    poses_.back().scan = observed_scan;

    // look at the last pose.scan in poses_ and if empty:
    //    given: prev_pose.x,y,theta,scan,
    //           curr_pose.x,y,theta
    //           scan(ranges,min,max,etc)
    //    create cost_table = prev_pose.scan + normal_distribution_for_each_point
    //    create corr_scan_probabilities (x,y,theta) centered at curr_pose.x,y,theta
    //        where each (x,y,theta) is probability of scan matching with prev_pose.scan
    //            for each (x,y,theta):
    //                use cost_table to determine observation_likelihood probability
    //                      multiplied by motion_model_probability

    // return highest probability in corr_scan_probabilities as optimized pose:
    //     curr_pose.x,y,theta = optimized pose
    // cur_pose.scan = observed_scan
          




    // compute rasterized cost table for each pose - fill in table with previous scan,
    //   use current scan to determine the observation likelihood



    // At End:  for 2 relative poses, compute likelihood of pose given motion 
    //    model and observation liklihood


    // poses_.push_back(optimized_posed);
  }  
  
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  
  UpdateOdometry(odom_loc, odom_angle);

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  float min_trans = 0.5;
  float min_rot = 30 * M_PI / 180.0;

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
      current_pose.scan = std::vector<Vector2f>();
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

// Raf: so we need to save the selected pose at each observe laser and store its
// relative laser scan
vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  //loop through poses:
  //   add pose.scan points to map (scan is transformed to map frame)

  return map;
}

}  // namespace slam
