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
    float_t del_rot = poses_.back().theta - poses_.front().theta;

    auto bl = Eigen::Rotation2Df(-poses_.front().theta) * del_trans;
    *loc = Eigen::Vector2f(CONFIG_init_x, CONFIG_init_y) + Eigen::Rotation2Df(CONFIG_init_theta) * bl;

    //*loc = Eigen::Vector2f(CONFIG_init_x, CONFIG_init_y) + del_trans;
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
  //obs_scan.resize(num_ranges);

  // old code to use a subset of the ranges
  // const float index_inc = (ranges.size() / (num_ranges - 1));
  // const float angle_inc = (angle_max - angle_min) / (num_ranges - 1);

  const float angle_inc = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++){
    if (ranges[i] < 10.0) {
      float current_angle = i * angle_inc + angle_min;
      obs_scan.push_back(Vector2f(ranges[i] * cos(current_angle), ranges[i] * sin(current_angle)));
    }
  }
}

void PrintCostTable(Eigen::MatrixXd cost_table){
  int row_size = cost_table.rows();
  int col_size = cost_table.cols();

  for (int i = 0; i < row_size; i++){
    for (int j = 0; j < col_size; j++){
      ROS_INFO("Cost_Table(%d,%d) = %f", i, j, cost_table(i,j));
    }
  }
}

/* float_t getGaussianProbability(float_t val, float_t mean, float_t std_dev) {
  //P(val) = 1/(std_dev * sqrt(2*PI)) * exp((-1 * pow((val - mean), 2) / (2 * pow(std_dev, 2)))
  float_t e_power = (val - mean) * (val - mean) * -1 / (2 * std_dev * std_dev);
  float_t coefficient = 1 / (std_dev * sqrt(2 * m_pi));
  return coefficient * exp(e_power);
} */

// create the rasterized cost table based on prev_scan points
void SLAM::CreateCostTable(const std::vector<Eigen::Vector2f>& prev_scan,
                           std::shared_ptr<Eigen::MatrixXd>& cost_table_ptr) {
  float resolution = 100.0 / 20.0;  // 100 px / (10 * 2)m
  // int cost_table_size = (int)(20 * resolution);

  cost_table_ptr = std::make_shared<Eigen::MatrixXd>(100, 100);
  Eigen::MatrixXd& cost_table = *cost_table_ptr;
  cost_table.setZero();
  
  /* Determine range of points, standard deviation of gaussian */
  // range = -1;
  // std_dev = -1;
  for (const auto& point : prev_scan){
    // ROS_INFO("point = (%f, %f)", point.x(), point.y());    
    // ROS_INFO("adjusted point = (%f, %f)", x, y);
    if ((point.x() <= 10.0 && point.x() >= -10.0) && (point.y() <= 10.0 && point.y() >= -10.0)){
      float pixel_x = resolution * (point.x() + 10.0);
      float pixel_y = resolution * (point.y() + 10.0);
      //int pixel_x = (int) pixelf_x;
      //int pixel_y = (int) pixelf_y;

      /* for x in [pixel_x - range, pixel_x + range], 0 < x < matrix size */
      //int x = 0 < (pixel_x - range) ? (pixel_x - range) : 0;
      //while (x < cost_table_size && x <= (pixel_x + range)) {
        /* for y in [pixel_y - range, pixel_y + range], 0 < y < matrix size */
          //int y = 0 < (pixel_y - range) ? (pixel_y - range) : 0;
          //while (y < cost_table_size && y <= (pixel_y + range)) {
            /* add value according to gaussian distribution for x, y */
            /* getGaussianProbability will be a function that gets the probability that a given value
                would be randomly selected from a gaussian with a given mean and std_dev */
            //cost_table(x, y) += 
            //   getGaussianProbability(x, pixel_x, std_dev) * getGaussianProbability(y, pixel_y, std_dev);
            //++y;
          //}
        //++x;
      //}
      //          (0, 100)

      // ROS_INFO("3: pixel_x: %d, pixel_y: %d", (int)pixel_x, (int)pixel_y);
      cost_table((int)pixel_x, (int)pixel_y) = 1.0;
      // cost_table((int)pixel_y, (int)pixel_x) = 1.0;
    }
  }
  // ROS_INFO("DONE FILLING");
  // ROS_INFO("COST TABLE %f", cost_table(0,0));
}

// Observation Likelihood: return log likelihood of how likely it is 
// that a given x, y, theta and curr_scan are correlated to cost_table
float_t SLAM::FindObservationLogLikelihood(float x, 
                                           float y, 
                                           float theta, 
                                           const Pose& prev_pose,
                                           const Eigen::MatrixXd& cost_table, 
                                           const std::vector<Eigen::Vector2f>& curr_scan) {

  float dx = x - prev_pose.x;
  float dy = y - prev_pose.y;
  float dtheta = theta - prev_pose.theta;

  Eigen::Vector2f translation(dx, dy);
  std::vector<Eigen::Vector2f> scan_transformed;
  Eigen::Rotation2Df rot(-dtheta);
  for (const auto& point : curr_scan) {
    scan_transformed.push_back(rot * (point - translation));
  }
  // ROS_INFO("COST(0, 0) %f", cost_table(0, 0));
  // ROS_INFO("Cost table sum: %f", cost_table.sum());
  // ROS_INFO("HERE");
  
  std::shared_ptr<Eigen::MatrixXd> cost_table_ptr;
  CreateCostTable(scan_transformed, cost_table_ptr);
  
  // ROS_INFO("New cost table sum: %f", cost_table_ptr->sum());

  return log(cost_table.cwiseProduct(*cost_table_ptr).sum() / (cost_table.sum() + 1.0e-7));

}

// Motion Model: return log likelihood of how likely it is
// that a given x, y, theta exits given curr_pose
float_t SLAM::FindMotionModelLogLikelihood(float x, 
                                         float y, 
                                         float theta, 
                                         const Pose& curr_pose,
                                         const Pose& prev_pose) {

  float_t dx = x - prev_pose.x;   
  float_t dy = y - prev_pose.y;    
  float_t dtheta = theta - prev_pose.theta;

  float_t dx_odom = curr_pose.x - prev_pose.x;
  float_t dy_odom = curr_pose.y - prev_pose.y;
  float_t dtheta_odom = curr_pose.theta - prev_pose.theta;

  float_t std = sqrt(pow(dx_odom, 2) + pow(dy_odom, 2));
  std += abs(dtheta_odom);

  float_t dx_loglikelihood = log(exp(- (pow((dx - dx_odom), 2) / 2 * pow(std, 2))));
  float_t dy_loglikelihood = log(exp(- (pow((dy - dy_odom), 2) / 2 * pow(std, 2))));
  float_t dtheta_loglikelihood = log(exp(- (pow((dtheta - dtheta_odom), 2) / 2 * pow(std, 2))));

  return dx_loglikelihood + dy_loglikelihood + dtheta_loglikelihood;
}

void PrintScan(std::vector<Eigen::Vector2f> scan) {
  int i = 0;
  for (auto& point : scan){
    ROS_INFO("Point %d: (%f, %f)", i, point.x(), point.y());
    i++;
  }
}

void PrintPose(Pose pose, string name){
  ROS_INFO("pose(%s) = (x = %f, y = %f, theta = %f", name.c_str(), pose.x, pose.y, pose.theta);
}

void SLAM::PrintPoses(){
  int i = 0;
  for (auto& pose : poses_){
    PrintPose(pose, std::to_string(i));
    i++;
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
  
  // ROS_INFO("Printing Observed Scan");
  // PrintScan(observed_scan);

  // ObserveLaser called before there is a single pose in poses_
  if (poses_.size() == 0) {
    return;
  // First ObserveLaser called with a single pose in poses_
  } else if (poses_.size() == 1) {
    poses_.back().scan = observed_scan;
  // Compute Correlative Scan Matching with previous two poses
  } else {
    // poses_.back().scan = observed_scan; // remove this when uncommenting

    if (poses_.back().scan.size() == 0) {

      poses_.back().scan = observed_scan;
      Pose curr_pose = poses_.back();
      auto prev_pose_iter = poses_.end();
      prev_pose_iter = std::prev(prev_pose_iter, 2);
      Pose prev_pose = *prev_pose_iter;

      std::shared_ptr<Eigen::MatrixXd> cost_table_ptr;
      CreateCostTable(prev_pose.scan, cost_table_ptr);
      // ROS_INFO("FIRST TRY COST(0, 0): %f", (*cost_table_ptr)(0, 0));
      
      // ROS_INFO("Printing Cost_Table");
      // PrintCostTable(cost_table);

      float x_width = 0.25;
      float y_width = 0.25;
      float theta_width = 15 * M_PI / 180.0;
      float resolution = 25.0;
      float x_inc = 2 * x_width / resolution; // 2 * .5 / 50 = 0.02 increment
      float y_inc = 2 * y_width / resolution;
      float theta_inc = 2 * theta_width / resolution; 

      float_t max_loglikelihood = -10000000.0;
      // float_t max_log_mm = -1000000.0;
      // float_t max_log_ol = -1000000.0;
      float best_x = curr_pose.x;
      float best_y = curr_pose.y;
      float best_theta = curr_pose.theta;

      // int num_elements = 0;
      // int num_x = 0;
      // int num_y = 0;
      // int num_theta = 0;
      // ROS_INFO("Printing x_i, y_i, theta_i");
      //ROS_INFO("curr_pos is (%f, %f, %f)", best_x, best_y, best_theta);
      // ROS_INFO("range is (%f - %f, %f - %f, %f - %f)", -x_width + curr_pose.x, x_width + curr_pose.x, -y_width + curr_pose.y, y_width + curr_pose.y, -theta_width + curr_pose.theta, theta_width + curr_pose.theta);
      // ROS_INFO("iterations is (%f, %f, %f)", x_width * 2 / x_inc + 1, y_width * 2 / y_inc + 1, theta_width * 2 / theta_inc + 1);

      // PrintPose(curr_pose, "Current Pose");
      // PrintPose(prev_pose, "Previous Pose");
      // PrintPoses();
      
      // double log_ol = FindObservationLogLikelihood(prev_pose.x, prev_pose.y, prev_pose.theta, prev_pose, cost_table, prev_pose.scan);
      // ROS_INFO("log_ol = %f", log_ol);
      // ROS_INFO("sum cost_table = %f", cost_table.sum());
      
      for (float x_i = -x_width + curr_pose.x; x_i <= x_width + curr_pose.x; x_i += x_inc) {
        
        for (float y_i = -y_width + curr_pose.y; y_i <= y_width + curr_pose.y; y_i += y_inc) {
          
          for (float theta_i = -theta_width + curr_pose.theta; theta_i <= theta_width + curr_pose.theta; theta_i += theta_inc) {
            
            float_t log_ol = FindObservationLogLikelihood(x_i, y_i, theta_i, prev_pose, *cost_table_ptr, curr_pose.scan);
            float_t log_mm = FindMotionModelLogLikelihood(x_i, y_i, theta_i, curr_pose, prev_pose);
            float_t log_likelihood = log_ol + log_mm;
            
            if (log_likelihood > max_loglikelihood){ 
              // max_log_mm = log_mm;
              // max_log_ol = log_ol;
              max_loglikelihood = log_likelihood;
              
              best_x = x_i;
              best_y = y_i;
              best_theta = theta_i;
            }
          }
        }
      }

      // ROS_INFO("log_mm(%f, %f, %f) = %.10f", best_x, best_y, best_theta, max_log_mm);
      // ROS_INFO("log_ol(%f, %f, %f) = %.10f", best_x, best_y, best_theta, max_log_ol);
      // ROS_INFO("best_log_likelihood(%f, %f, %f) = %.10f", best_x, best_y, best_theta, max_loglikelihood);
      // ROS_INFO("Num Elements = %d", num_elements);
      // ROS_INFO("num_x = %d", num_x);
      // ROS_INFO("num_y = %d", num_y / num_x);
      // ROS_INFO("num_theta = %d", num_theta / num_y);
      
      // ROS_INFO("best_x = %f", best_x);
      // ROS_INFO("best_y = %f", best_y);
      // ROS_INFO("best_theta = %f", best_theta);
      curr_pose.x = best_x;
      curr_pose.y = best_y;
      curr_pose.theta = best_theta;
      
    }    

  }  
  
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  
  UpdateOdometry(odom_loc, odom_angle);

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  float min_trans = 0.5;
  float min_rot = 30 * M_PI / 180.0;

  // ROS_INFO("poses_.size() = %ld", poses_.size());

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

// vector<Vector2f> SLAM::GetMap() {
//   std::vector<Vector2f> map;
//   // Reconstruct the map as a single aligned point cloud from all saved poses
//   // and their respective scans.

//   Pose initial_pose = poses_.front();
//   // map = initial_pose.scan;

//   // PrintPose(initial_pose, "Initial Pose");
//   for (auto& curr_pose : poses_) {
    
//     // need to translate the pose.scan relative to poses_.front().scan
//     // PrintPose(curr_pose, "Current Pose");
//     Eigen::Rotation2Df base_rot (-initial_pose.theta);
//     Eigen::Vector2f del_trans = Eigen::Vector2f(curr_pose.x - initial_pose.x - CONFIG_init_x,
//                                                 curr_pose.y - initial_pose.y - CONFIG_init_y);
//     Eigen::Rotation2Df del_rot(-(curr_pose.theta - initial_pose.theta - CONFIG_init_theta));
    
//     std::vector<Eigen::Vector2f> trans_scan;
//     for (auto& point : curr_pose.scan){ 
//       // ROS_INFO("point(%f, %f), del_trans (%f, %f)", point.x(), point.y(), del_trans.x(), del_trans.y());
//       trans_scan.push_back(del_rot * (point + base_rot * del_trans)); //del_rot * (point - del_trans));
//     }
//     map.insert(map.end(), trans_scan.begin(), trans_scan.end());
//   }
//   PrintPoses();
//   return map;
// }

vector<Vector2f> SLAM::GetMap() {
  std::vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  Pose initial_pose = poses_.front();

  for (auto& curr_pose : poses_) {
    ROS_INFO("init_x = %f, init_y = %f, init_theta = %f", CONFIG_init_x, CONFIG_init_y, CONFIG_init_theta);
    ROS_INFO("initial_pose = (%f, %f)", initial_pose.x, initial_pose.y);
    ROS_INFO("curr_pose = (%f, %f)", curr_pose.x, curr_pose.y);

    Eigen::Rotation2Df rot_to_initial_pose(curr_pose.theta - initial_pose.theta);
    Eigen::Vector2f trans_to_initial_pose(curr_pose.x - initial_pose.x, curr_pose.y - initial_pose.y);
    
    ROS_INFO("curr_pose.theta - initial_pose.theta = %f", curr_pose.theta - initial_pose.theta);
    ROS_INFO("trans_to_initial_pose = (%f, %f)", trans_to_initial_pose.x(), trans_to_initial_pose.y());

    Eigen::Rotation2Df rot_to_init_frame(initial_pose.theta - CONFIG_init_theta);
    Eigen::Vector2f trans_to_init_frame(initial_pose.x - CONFIG_init_x, initial_pose.y - CONFIG_init_y);

    ROS_INFO("initial_pose.theta - CONFIG_init_theta = %f", initial_pose.theta - CONFIG_init_theta);
    ROS_INFO("trans_to_init = (%f, %f)", trans_to_init_frame.x(), trans_to_init_frame.y());

    std::vector<Eigen::Vector2f> trans_scan;
    for (auto& point : curr_pose.scan){ 
      ROS_INFO("point = (%f, %f)", point.x(), point.y());
      Eigen::Vector2f point_in_initial_pose(rot_to_initial_pose * (point + trans_to_initial_pose));
      ROS_INFO("point_in_initial_pose = (%f, %f)", point_in_initial_pose.x(), point_in_initial_pose.y());
      Eigen::Vector2f point_in_init_frame(rot_to_init_frame * (point_in_initial_pose + trans_to_init_frame));
      ROS_INFO("point_in_initial_pose = (%f, %f)", point_in_initial_pose.x(), point_in_initial_pose.y());
      trans_scan.push_back(point_in_init_frame);
      break;
    }
    map.insert(map.end(), trans_scan.begin(), trans_scan.end());
    break;
  }
  PrintPoses();
  return map;
}

}  // namespace slam
