/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GUARD_REACTIVE_GRASPING_DETECTION_H
#define GUARD_REACTIVE_GRASPING_DETECTION_H

// Standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>
// MoveIt libraries
#include <moveit/move_group_interface/move_group.h>
// Auto-generated from msg/ directory libraries
#include "reactive_grasping/GloveIMU.h"
#include "reactive_grasping/GloveIMUArray.h"
#include "reactive_grasping/DataHistory.h"
#include "reactive_grasping/MotionAction.h"
// TODO try to use the glove messages (instead of the copies in reactive_grasping)

// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_GLOVE_TOPIC_NAME "glove_topic"
#define DEFAULT_ACCEL_MAP_TOPIC_NAME "accel_map_topic"
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_NUM_IMUS 5  // number of IMUs on the sensorized Glove
#define DEFAULT_GRAVITY_VALUE 4096  // 1g gravity value
#define DEFAULT_CONTACT_THRESHOLD 0.5*DEFAULT_GRAVITY_VALUE  // threshold to be considered as a contact
#define DEFAULT_FALSE_POSITIVE_THRESHOLD 500  // gyro threshold to avoid false positive detection
#define DEFAULT_WINDOW_SIZE 30  // number of samples (size) of the detection and correlation window
#define DEFAULT_TAILS_SCALE_FACTOR 0.5  // scale factor of the peaks of the other IMUs
#define DEFAULT_DELAY_THRESHOLD 60  // aborts a goal if not reached in at least 'delay_threshold_' seconds
#define DEFAULT_ACTION_SERVER "pose_action_server"
#define DEFAULT_VERBOSE_MODE true
#define DEFAULT_VERY_VERBOSE_MODE false
#define DEFAULT_ONLY_DETECTION false
#define DEFAULT_CALIBRATION false
#define DEFAULT_LOG_FILE_BASE_PATH "logs/"
#define DEFAULT_LOG_FILE_NAME_RAW "accelerations_raw"
#define DEFAULT_LOG_FILE_NAME_FILT "accelerations_filtered"
#define DEFAULT_LOG_FILE_NAME_MAP "accelerations_map"
#define DEFAULT_LOG_FILE_NAME_GYRO "gyro_velocities_raw"


class ReactiveGraspingDetection {
 public:
  ReactiveGraspingDetection();
  ~ReactiveGraspingDetection();

 private:
  ros::NodeHandle *private_node_handle_;
  ros::NodeHandle node_handle_;
  ros::Subscriber glove_subscriber_;
  ros::Publisher accel_map_publisher_;
  actionlib::SimpleActionClient<reactive_grasping::MotionAction> *motion_action_client_;

  // system state variables
  bool contact_detected_;
  bool false_positive_;
  bool hand_closed_;
  ros::Time goal_activate_time_;
  std::vector<reactive_grasping::DataHistory> accel_raw_;
  std::vector<reactive_grasping::DataHistory> accel_filt_;
  std::map<std::string, std::map<std::string, std::vector<double>>> accel_map_;
  std::vector<reactive_grasping::DataHistory> gyro_raw_;

  // settings variables
  std::string glove_topic_name_;
  std::string accel_map_topic_name_;
  std::string action_server_;
  int topic_queue_length_;
  int gravity_value_;
  int num_imus_;
  int window_size_;
  double contact_threshold_;
  double false_positive_threshold_;
  double tails_scale_factor_;
  double delay_threshold_;
  bool only_detection_;
  bool calibration_;
  bool demo_;
  int skip_samples_;
  std::vector<double> filter_coeff_a_;
  std::vector<double> filter_coeff_b_;

  // statistics variables
  ros::Time start_time_;
  int num_data_processed_;
  int num_contacts_detected_;

  // log files
  std::string log_file_name_raw_;
  std::string log_file_name_filt_;
  std::string log_file_name_map_;
  std::string log_file_name_gyro_;
  std::string log_file_base_path_;
  std::string date_time_;
  std::ofstream log_file_accel_raw_;
  std::ofstream log_file_accel_filt_;
  std::ofstream log_file_accel_map_;
  std::ofstream log_file_gyro_raw_;

  // verbosity control variables
  bool verbose_mode_;
  bool very_verbose_mode_;


  void actionActiveCallback();

  void actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                          const reactive_grasping::MotionResultConstPtr &result);

  void actionFeedbackCallback(const reactive_grasping::MotionFeedbackConstPtr &feedback);

  void appendToLogFile(std::ofstream *log_file, const std::vector<reactive_grasping::DataHistory> &data,
                      const ros::Duration &acquisition_time);

  int checkOscillation(int imu_id);

  int detectContact();

  void eraseOldestSample();

  void eraseOldestSample(std::vector<reactive_grasping::DataHistory> &data);

  std::string extractGraspPrimitive(const std::vector<double> &data);

  geometry_msgs::Pose fillTargetPose(std::string approaching_direction);

  std::vector<double> filter(std::vector<double> b, std::vector<double> a, std::vector<double> x, std::vector<double> y);

  void filterAccelerations();

  void generateAndSendGoal(std::string status, std::string approaching_direction);

  void gloveMessageCallback(const reactive_grasping::GloveIMUArray::ConstPtr &msg);

  std::vector<double> normalizeAccelerations(std::vector<double> data, int imu_id);

  void parseAccelerationMap();

  void printGloveIMUArray(const std::vector <reactive_grasping::GloveIMU> &data);

  void processData(std::vector<reactive_grasping::GloveIMU> data, ros::Duration acquisition_time);

  void pushBackNewSample(const std::vector<reactive_grasping::GloveIMU> &new_sample);

  std::string toString(const std::vector<double> &data);

  std::vector<double> toVector(const std::vector<reactive_grasping::DataHistory> &data);

  void updateLogFiles(const ros::Duration &acquisition_time);

  std::vector<double> xcorr(std::vector<double> x, std::vector<double> y, int max_lag);
};

#endif
