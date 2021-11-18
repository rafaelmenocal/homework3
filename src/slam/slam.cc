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
#include <fstream>
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

const int debug_odom = false;
const int row_size = 500;
const int col_size = 500;
Eigen::MatrixXf cost_table = Eigen::MatrixXf::Zero(row_size, col_size); 
const int kernel_size = 5;
Eigen::Matrix<float,kernel_size,kernel_size> gaussian_kernel;

void SLAM::InitializeGuassianKernel(){
  gaussian_kernel << 1,  4,  7,  4, 1,
                     4, 16, 26, 16, 4,
                     7, 26, 41, 26, 7,
                     4, 16, 26, 16, 4,
                     1,  4,  7,  4, 1;
  gaussian_kernel = gaussian_kernel / 273.0; //41.0;
}

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
    prev_update_angle_(0)
    { InitializeGuassianKernel(); }


void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  // if (poses_.size() != 0) {
  //   Eigen::Vector2f del_trans = Eigen::Vector2f(poses_.back().x - poses_.front().x, poses_.back().y - poses_.front().y);
  //   float_t del_rot = poses_.back().theta - poses_.front().theta;

  //   *loc = Eigen::Rotation2Df(-poses_.front().theta) * del_trans;
  //   *angle = del_rot;
    
  //   // ROS_INFO("GetPose loc = (%f, %f)", poses_.back().x, poses_.back().y);
  //   // ROS_INFO("GetPose angle = %f", poses_.back().theta);
  // }

  if (poses_.size() != 0) {
    *loc = Eigen::Vector2f(poses_.back().x, poses_.back().y);
    *angle = poses_.back().theta;
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


// Save RGB Cost Table with given rgb %float (ie. 1 1 1 is white)
void SLAM::SaveRGBCostTable(std::string file_name, float r, float g, float b){
  std::ofstream file(file_name + ".mat", std::ofstream::out | std::ofstream::trunc);
  if (file.is_open()) {
    
    file << "# name: " + file_name + "\n";
    file << "# type: matrix\n";
    file << "# ndims: 3\n";
    file << " " << row_size << " " << col_size << " " << "3\n";

    // r color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        file <<  " " << r * cost_table(i,j) << "\n";
      }
    }

    // g color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        file <<  " " << g * cost_table(i,j) << "\n";
      }
    }

    // b color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        file <<  " " << b * cost_table(i,j) << "\n";
      }
    }

  }
  file.close();
  ROS_INFO("RGB Cost Table Saved to %s.mat.", file_name.c_str());
}


void SLAM::SaveCostTable(std::string file_name){
  std::ofstream file(file_name + ".mat", std::ofstream::out | std::ofstream::trunc);
  if (file.is_open()) {
    
    file << "# name: " + file_name + "\n";
    file << "# type: matrix\n";
    file << "# rows: " << row_size << "\n";
    file << "# columns: " << col_size << "\n";
    file << cost_table << "\n";
  }
  file.close();
  ROS_INFO("Cost Table Saved to %s.mat.", file_name.c_str());
}

void PrintPose(Pose pose, string name){
  ROS_INFO("pose(%s) = (%.2f, %.2f, %.2f)", name.c_str(), pose.x, pose.y, pose.theta);
}

void SLAM::PrintPoses(){
  ROS_INFO("=======PRINT POSES=======");
  int i = 0;
  for (auto& pose : poses_){
    PrintPose(pose, std::to_string(i));
    i++;
  }
  ROS_INFO("========================");
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

  const float angle_inc = (angle_max - angle_min) / num_ranges;
  for (int i = 0; i < num_ranges; i++){
    float current_angle = i * angle_inc + angle_min;
    if (ranges[i] < range_max && ranges[i] > range_min){
      obs_scan[i] = Vector2f(ranges[i] * cos(current_angle), ranges[i] * sin(current_angle));
    }
  }
}

Eigen::Vector2i PointToIndex(Eigen::Vector2f point){
  return Eigen::Vector2i(row_size - int(((point.y() + 10)/20.0) * row_size),int(((point.x() + 10)/20.0) * col_size));
}


// create the rasterized cost table based on prev_scan points
void SLAM::UpdateCostTable(std::vector<Eigen::Vector2f> prev_scan) {
  cost_table = Eigen::MatrixXf::Zero(row_size, col_size); 
  // ROS_INFO("cost_table size = (%ld, %ld)", cost_table.rows(), cost_table.cols());
  
  // for each point in the scan, add a 1 to the cost_table
  for (Eigen::Vector2f point : prev_scan){
    // ROS_INFO("point(%f, %f)",point.x(),point.y());
    Eigen::Vector2i index = PointToIndex(point);
    cost_table(index.x(),index.y()) = 1;
  }
  // SaveRGBCostTable("cost_table_before_blur", 1.0, 1.0, 1.0);

  Eigen::MatrixXf temp_cost_table = Eigen::MatrixXf::Zero(row_size,col_size);
  // add gaussian blur
  for (int i = 0; i < row_size; i++){
    for (int j = 0; j < col_size; j++){
      if (cost_table(i,j) == 1){
        for (int k_i = 0; k_i < kernel_size; k_i++){
          for (int k_j = 0; k_j < kernel_size; k_j++){
            int trans_i = i - 2 + k_i;
            int trans_j = i - 2 + k_j;
            if ((trans_i >= 0 && trans_i < row_size) && 
                      (trans_j >= 0 && trans_j < col_size)) {
              temp_cost_table(i-2+k_i,j-2+k_j) += gaussian_kernel(k_i,k_j);
            }
          }
        }
      }
    }
  }

  cost_table = temp_cost_table;
  // SaveRGBCostTable("cost_table_after_blur", 1.0, 1.0, 1.0);
  return;
}

void SLAM::PrintGaussianKernel(){
  for (int i = 0; i < kernel_size; i++){
    for (int j = 0; j < kernel_size; j++){
      ROS_INFO("gaus(%d,%d) = %f", i, j, gaussian_kernel(i,j));
    }
  }
}

float_t GetLikelihood(float_t val, float_t mean, float_t std_dev) {
  return (1/(std_dev * (sqrt(2 * M_PI)))) * exp(-0.5 * pow((val - mean), 2) / pow(std_dev, 2));
}

float_t GetLogLikelihood(float_t val, float_t mean, float_t std_dev) {
  return log(GetLikelihood(val, mean, std_dev));
}

// Observation Likelihood: return log likelihood of how likely it is 
// that a given x, y, theta and curr_scan are correlated to cost_table
float SLAM::FindObservationLogLikelihood(float x, 
                                         float y, 
                                         float theta,
                                         Pose& prev_pose,
                                         std::vector<Eigen::Vector2f>& curr_scan) {
  
  Eigen::MatrixXf curr_cost_table = Eigen::MatrixXf::Zero(row_size, col_size); 

  // Eigen::Rotation2Df rot1(-prev_pose.theta);
  // Eigen::Rotation2Df rot2(prev_pose.theta);
  // Eigen::Vector2f delT_base_link = rot1 * (Eigen::Vector2f(x,y) - Eigen::Vector2f(prev_pose.x, prev_pose.y));
  // Eigen::Vector2f T2map = Eigen::Vector2f(prev_pose.x, prev_pose.y) + rot2 * delT_base_link;
  // float delTheta_base_link = theta - prev_pose.theta;
  // float Theta2map = prev_pose.theta + delTheta_bas_link;

  float dx = x - prev_pose.x;
  float dy = y - prev_pose.y;
  float dtheta = theta - prev_pose.theta;

  float observation_likelihood = 0.0;

  // Translation and rotation to move point in new scan back to frame of previous scan.
  Eigen::Vector2f translation(dx, dy);
  Eigen::Rotation2Df rot(dtheta);

  for (const auto& point : curr_scan) {
    Eigen::Vector2f trans_point = (rot * point) - translation;
    Eigen::Vector2i trans_index = PointToIndex(trans_point);
    
    curr_cost_table(trans_index.x(), trans_index.y()) = 1;
    observation_likelihood += cost_table(trans_index.x(),trans_index.y());
  }

  return observation_likelihood;
}

void SLAM::MergeAndSaveBestCostTable(float x, 
                                     float y, 
                                     float theta,
                                     Pose& prev_pose,
                                     std::vector<Eigen::Vector2f>& curr_scan) {
  
  Eigen::MatrixXf curr_cost_table = Eigen::MatrixXf::Zero(row_size, col_size); 

  // Eigen::Rotation2Df rot1(-prev_pose.theta);
  // Eigen::Rotation2Df rot2(prev_pose.theta);
  // Eigen::Vector2f delT_base_link = rot1 * (Eigen::Vector2f(x,y) - Eigen::Vector2f(prev_pose.x, prev_pose.y));
  // Eigen::Vector2f T2map = Eigen::Vector2f(prev_pose.x, prev_pose.y) + rot2 * delT_base_link;
  // float delTheta_base_link = theta - prev_pose.theta;
  // float Theta2map = prev_pose.theta + delTheta_bas_link;


  float dx = x - prev_pose.x;
  float dy = y - prev_pose.y;
  float dtheta = theta - prev_pose.theta;

  // Translation and rotation to move point in new scan back to frame of previous scan.
  Eigen::Vector2f translation(dx, dy);
  Eigen::Rotation2Df rot(dtheta);

  for (const auto& point : curr_scan) {
    Eigen::Vector2f trans_point = (rot * point) - translation;
    Eigen::Vector2i trans_index = PointToIndex(trans_point);
    curr_cost_table(trans_index.x(), trans_index.y()) = 1;
  }

  std::string file_name = "merged_cost_table";
  std::ofstream file(file_name + ".mat", std::ofstream::out | std::ofstream::trunc);
  if (file.is_open()) {
    
    file << "# name: " + file_name + "\n";
    file << "# type: matrix\n";
    file << "# ndims: 3\n";
    file << " " << row_size << " " << col_size << " " << "3\n";

    // r color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        if (curr_cost_table(i, j) != 1){
          file <<  " " << cost_table(i,j) << "\n";
        } else {
          file <<  " " << curr_cost_table(i,j) << "\n";
        }
      }
    }

    // g color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        if (curr_cost_table(i, j) != 1){
          file <<  " " << cost_table(i,j) << "\n";
        } else {
          file <<  " " << 0 << "\n";
        }      
      }
    }

    // b color channel
    for (int j = 0; j < col_size; j++){
      for (int i = 0; i < row_size; i++){
        if (curr_cost_table(i, j) != 1){
          file <<  " " << cost_table(i,j) << "\n";
        } else {
          file <<  " " << 0 << "\n";
        }      
      }
    }

  }
  file.close();
  ROS_INFO("Merged RGB Cost Table Saved to %s.mat.", file_name.c_str());

  return;
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
  
  float_t dx_loglikelihood = GetLogLikelihood(dx, dx_odom, std);
  float_t dy_loglikelihood = GetLogLikelihood(dy, dy_odom, std);
  float_t dtheta_loglikelihood = GetLogLikelihood(dtheta, dtheta_odom, std);

  return dx_loglikelihood + dy_loglikelihood + dtheta_loglikelihood;
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
    if (poses_.back().scan.size() == 0) {
      poses_.back().scan = observed_scan;
      
      Pose curr_pose = poses_.back();
      Pose prev_pose = poses_.end()[-2];

      UpdateCostTable(prev_pose.scan);  // cost_table
      
      float x_width = 0.50;
      float y_width = 0.50;
      float theta_width = 30 * M_PI / 180.0;
      float resolution = 100.0;
      float x_inc = 2 * x_width / resolution; 
      float y_inc = 2 * y_width / resolution;
      float theta_inc = 2 * theta_width / resolution;

      float max_loglikelihood = -10000000.0;
      float best_x = curr_pose.x;
      float best_y = curr_pose.y;
      float best_theta = curr_pose.theta;

      for (float x_i = -x_width + curr_pose.x; x_i <= x_width + curr_pose.x; x_i += x_inc) {
        for (float y_i = -y_width + curr_pose.y; y_i <= y_width + curr_pose.y; y_i +=y_inc) {
          for (float theta_i = -theta_inc + curr_pose.theta; theta_i <= theta_inc + curr_pose.theta; theta_i += theta_inc) {
            
            float log_ol = FindObservationLogLikelihood(x_i, y_i, theta_i, prev_pose, curr_pose.scan);
            // float_t log_mm = FindMotionModelLogLikelihood(x_i, y_i, theta_i, curr_pose, prev_pose);
            // float log_likelihood = log_ol + log_mm;
            float log_likelihood = log_ol;
            
            if (log_likelihood > max_loglikelihood){ 
              max_loglikelihood = log_likelihood;
              best_x = x_i;
              best_y = y_i;
              best_theta = theta_i;
            }
          }
        }
      }

      PrintPose(curr_pose,"curr_pose_before_optimization");
      ROS_INFO("max_log_likelihood = %f", max_loglikelihood);
      ROS_INFO("best_x = %f", best_x);
      ROS_INFO("best_y = %f", best_y);
      ROS_INFO("best_theta = %f", best_theta);

      curr_pose.x = best_x;
      curr_pose.y = best_y;
      curr_pose.theta = best_theta;

      MergeAndSaveBestCostTable(best_x, best_y, best_theta, prev_pose, curr_pose.scan);
      
      PrintPose(curr_pose,"curr_pose_after_optimization");

    }    

  }  
  
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  
  UpdateOdometry(odom_loc, odom_angle);

  float min_trans = 0.5;
  float min_rot = 30 * M_PI / 180.0;

  // PrintPoses();

  // if new pose has moved or rotated more than threshold
  if (((prev_update_loc_ - odom_loc).norm() >= min_trans) || 
                    (abs(prev_update_angle_ - odom_angle) >= min_rot)) {
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

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  std::vector<Eigen::Vector2f> trans_scan;

  map.clear();

  Pose initial_pose = poses_.front();
  // for (auto& curr_pose : poses_) {
  //   Eigen::Vector2f del_trans = Eigen::Vector2f(curr_pose.x - initial_pose.x,
  //                                               curr_pose.y - initial_pose.y);
  //   Eigen::Rotation2Df del_rot(curr_pose.theta - initial_pose.theta);

    trans_scan.clear();
    // for (auto& point : curr_pose.scan){
    for (auto& point : initial_pose.scan){
      // trans_scan.push_back(del_rot * (point - del_trans));
      trans_scan.push_back(point);
    }
    map.insert(map.end(), trans_scan.begin(), trans_scan.end());
  // }

  return map;
}

}  // namespace slam
