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

#ifndef __SRC_SLAM_SLAM_H__
#define __SRC_SLAM_SLAM_H__

#include <algorithm>
#include <vector>
#include <memory>

#include "ros/ros.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace slam {

typedef std::shared_ptr<std::vector<Eigen::Vector2f>> scan_ptr;

struct Pose {
  double_t x;
  double_t y;
  double_t theta;
  scan_ptr scan;

  Pose(double_t x, double_t y, double_t theta) : x(x), y(y), theta(theta) {}

};
struct PoseObservation {
  double_t obsliklihood;
  Eigen::MatrixXd cost_table;

  PoseObservation(double_t o, Eigen::MatrixXd m) : obsliklihood(o), cost_table(m) {};
};
class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    double_t range_min,
                    double_t range_max,
                    double_t angle_min,
                    double_t angle_max);

  /* Return last pose recorded
   *
   * @param loc: vector to full with (x, y) of pose
   * @param angle: float to fill with angle
   */
  inline void GetPose(Eigen::Vector2f* loc, float* angle) const {
    // Return the latest pose estimate of the robot.
    if (poses_.size() != 0) {
      *loc = Eigen::Vector2f(poses_.back().x, poses_.back().y);
      *angle = poses_.back().theta;
    }
  };

  /* Turn the vector of ranges into a proper LIDAR scan
   *
   * @param ranges: distances along each ray in the scan
   * @param range_min: the minimum possible range
   * @param range_max: the maximum possible range
   * @param angle_min: starting angle of the scan
   * @param angle_max: ending angle of the can
   * @param obs_scan_ptr: pose's scan to populate
   */
  void GetObservedPointCloud(const std::vector<double>& ranges,
                             double_t range_min,
                             double_t range_max,
                             double_t angle_min,
                             double_t angle_max,
                             scan_ptr& obs_scan_ptr,
                             Pose& pose);

  /*
   * Get the probability from a normal CDF
   *
   * @param val: the value to sample
   * @param mean: mean value of the normal distribution
   * @param std_dev: standard deviation of the distribution
   */
  inline double_t GetNormalProb(double_t val, double_t mean, double_t std_dev) {
    return 0.5 * (1 + erf((val - mean) / (sqrt(2) * std_dev)));
  }

  /*
   * Find P(a < x < b)
   *
   * @param val1: the left bound (a)
   * @param val2: the right bound (a)
   * @param mean: mean value of the normal distribution
   * @param std_dev: standard deviation of the distribution
   */
  inline double_t MotionModelProb(double_t val1,
                                  double_t val2,
                                  double_t mean,
                                  double_t std) {
    // ROS_INFO("val2: %f, val1: %f, mean: %f", val2, val1, mean);
    // ROS_INFO("p2: %.40f",SLAM::GetNormalProb(val2 - mean, mean, std));
    // ROS_INFO("p1: %.40f",SLAM::GetNormalProb(val1 - mean, mean, std));

    // return (SLAM::GetNormalProb(val2 - mean, mean, std)
    //          - SLAM::GetNormalProb(val1 - mean, mean, std));

    return (1 / (std * sqrt(2 * M_PI))) * exp(-0.5 * pow((val1 - mean) / std, 2.0));
  }


  void CreateCostTable(const std::vector<Eigen::Vector2f>& prev_scan,
                       std::shared_ptr<Eigen::MatrixXd>& cost_table_ptr);

  PoseObservation FindObservationLogLikelihood(double_t x, 
                                        double_t y, 
                                        double_t theta,
                                        const Pose prev_pose,
                                        Eigen::MatrixXd cost_table, 
                                        const std::vector<Eigen::Vector2f>& curr_scan, 
                                        int c,
                                        bool plot = false);
 
  void PrintPoses();
  double_t FindMotionModelLogLikelihood(double_t x1,
                                        double_t x2, 
                                        double_t y1,
                                        double_t y2,
                                        double_t theta1,
                                        double_t theta2,
                                        const Pose curr_pose,
                                        const Pose prev_pose);

  /*
  * Take in the latest odometry reading. Only record the new odometry if the
  * robot has moved a certain amount.
  * 
  * @param: odometry location of the robot
  * @param: odometry angle of the robot
  */
  void ObserveOdometry(const Eigen::Vector2f& odom_loc, const double_t odom_angle);

  /*
   * Reconstruct the map using the optimized poses.
   */
  std::vector<Eigen::Vector2f> GetMap();

 private:

  // vector of previous poses
  std::vector<Pose> poses_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_update_loc_;
  double_t prev_update_angle_;

  int c_ = 0;

};
}  // namespace slam

#endif   // __SRC_SLAM_SLAM_H__
