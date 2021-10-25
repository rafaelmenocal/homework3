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
    odom_omega_(0) {}

// Raf: similar purpose as Get_Loc in the particle_filter assignment
void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
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

  prev_time_ = curr_time_;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  prev_odom_vel2f_ = odom_vel2f_;         
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.


}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  
  UpdateOdometry(odom_loc, odom_angle); // Added to keep track of other variables

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  // Raf: I think we need a data structure to store all of the poses (maybe only
  //    if the robot has moved a minimum amount)


}

// Raf: so we need to save the selected pose at each observe laser and store its
// relative laser scan
vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.


  return map;
}

}  // namespace slam
