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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
  float_t x;
  float_t y;
  float_t theta;
  std::vector<Eigen::Vector2f> scan;
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Called to update variables based on odometry
  void UpdateOdometry(const Eigen::Vector2f& odom_loc, const float odom_angle);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc, const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  void GetObservedPointCloud(const std::vector<float>& ranges,
                             float range_min,
                             float range_max,
                             float angle_min,
                             float angle_max,
                             std::vector<Eigen::Vector2f>* obs_scan_ptr);

  void CreateCostTable(const std::vector<Eigen::Vector2f>& prev_scan,
                       std::shared_ptr<Eigen::MatrixXd>& cost_table_ptr);

  float_t FindObservationLogLikelihood(float x, 
                                       float y, 
                                       float theta, 
                                       const Pose& prev_pose,
                                       const Eigen::MatrixXd& cost_table, 
                                       const std::vector<Eigen::Vector2f>& curr_scan);
 

  float_t FindMotionModelLogLikelihood(float x1,
                                       float x2, 
                                       float y1,
                                       float y2,
                                       float theta1,
                                       float theta2,
                                       const Pose& curr_pose,
                                       const Pose& prev_pose);
  void PrintPoses();
  

 private:

  // vector of previous poses
  std::vector<Pose> poses_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  Eigen::Vector2f curr_odom_loc_;
  float curr_odom_angle_;
  double curr_time_;
  double prev_time_;
  double del_time_;
  
  Eigen::Vector2f del_odom_loc_;
  
  Eigen::Vector2f prev_odom_vel2f_;
  Eigen::Vector2f odom_vel2f_;
  Eigen::Vector2f odom_accel2f_;
  float odom_vel_;
  float odom_accel_;
  float del_odom_angle_;
  float odom_omega_;

  Eigen::Vector2f prev_update_loc_;
  float prev_update_angle_;

  // A few small helper functions.
  inline float_t Vel2fToVel() const {
    return sqrt(pow(odom_vel2f_.x(), 2) + pow(odom_vel2f_.y(), 2));
  };

  inline float_t Accel2fToAccel() const {
    return sqrt(pow(odom_accel2f_.x(), 2) + pow(odom_accel2f_.y(), 2));
  }

  inline Eigen::Vector2f GetOdomVel2f() const {
    return (1.0 / del_time_) * 
      Eigen::Vector2f(curr_odom_loc_.x() - prev_odom_loc_.x(), curr_odom_loc_.y() - prev_odom_loc_.y());
  }

  inline Eigen::Vector2f GetOdomAccel2f() const {
    return (prev_odom_vel2f_ - odom_vel2f_) * del_time_;
  }
};
}  // namespace slam

#endif   // SRC_SLAM_H_
