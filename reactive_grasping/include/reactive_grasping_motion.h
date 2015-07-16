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

#ifndef GUARD_REACTIVE_GRASPING_MOTION_H
#define GUARD_REACTIVE_GRASPING_MOTION_H

// Standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <actionlib/server/simple_action_server.h>
// MoveIt libraries
#include <moveit/move_group_interface/move_group.h>
// Auto-generated from msg/ directory libraries
#include "reactive_grasping/MotionAction.h"

// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_HAND_SYNERGY_JOINT "right_hand_synergy_joint"
#define DEFAULT_HAND_SYNERGY_TOPIC "/right_hand/joint_trajectory_controller/command"
#define DEFAULT_JOINT_STATE_TOPIC "/joint_states"
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_ACTION_SERVER "pose_action_server"
#define DEFAULT_MOVE_GROUP "right_hand_arm"
#define DEFAULT_MOVE_GROUP_HOME "right_hand_arm_home"
#define DEFAULT_FRAME_BASE "vito_anchor"
#define DEFAULT_FRAME_EE_KUKA "right_hand_palm_link"
#define DEFAULT_FRAME_EE_GLOVE "wrist_measures_fixed_frame"
#define DEFAULT_TIME_FOR_GRASP 0.5
#define DEFAULT_ARM_VELOCITY_THRESHOLD 0.05
#define DEFAULT_ARM_DISTANCE_THRESHOLD 0.001
#define DEFAULT_HAND_VELOCITY_THRESHOLD 0.05
#define DEFAULT_HAND_DISTANCE_THRESHOLD 0.005
#define DEFAULT_DETECT_CONTACT_DELAY 1  // wait for a small amount of time when target pose is reached (oscillations)
#define DEFAULT_VERBOSE_MODE true
#define DEFAULT_VERY_VERBOSE_MODE false

class ReactiveGraspingMotion {
 public:
  ReactiveGraspingMotion();
  ~ReactiveGraspingMotion();

 private:
  ros::NodeHandle *private_node_handle_;
  ros::NodeHandle node_handle_;
  ros::Publisher hand_synergy_publisher_;
  ros::Subscriber joints_subscriber_;
  tf::TransformListener tf_listener_;
  actionlib::SimpleActionServer<reactive_grasping::MotionAction> *server_;
  moveit::planning_interface::MoveGroup *group_;
  moveit::planning_interface::MoveGroup::Plan plan_to_target_;

  // system state variables
  double hand_grasped_;
  bool homing_;
  bool arm_moving_;
  bool hand_moving_;
  sensor_msgs::JointState last_joint_state_;
  geometry_msgs::PoseStamped home_pose_;
  geometry_msgs::PoseStamped current_target_pose_;

  // settings variables
  std::string hand_synergy_joint_;
  std::string hand_synergy_topic_;
  std::string joint_state_topic_;
  std::string action_server_;
  std::string move_group_;
  std::string move_group_home_;
  std::string frame_base_;
  std::string frame_ee_kuka_;
  std::string frame_ee_glove_;
  int topic_queue_length_;
  double time_for_grasp_;
  double arm_velocity_threshold_;
  double arm_distance_threshold_;
  double hand_velocity_threshold_;
  double hand_distance_threshold_;
  double detect_contact_delay_;

  // verbosity control variables
  bool verbose_mode_;
  bool very_verbose_mode_;


  geometry_msgs::PoseStamped changeFrame(std::string to_frame, std::string from_frame,
                                         const geometry_msgs::Pose &from_pose_msg);

  bool checkForEndMove(const sensor_msgs::JointState &current_joint_state);

  bool checkForEndGrasp(const sensor_msgs::JointState &current_joint_state);

  void generateAndPublishResult(std::string status, const sensor_msgs::JointState &joint_state);

  void generateAndPublishFeedback(std::string status, const sensor_msgs::JointState &joint_state);

  void grasp(double command_value);

  void goalCallback();

  void homing();

  void moveToTargetPose(geometry_msgs::PoseStamped grasp_pose);

  bool planAndExecute();

  void preemptCallback();

  sensor_msgs::JointState selectJoints(const std::vector<std::string> &joint_names,
                                       const sensor_msgs::JointState &all_joint_state);

  std::string toString(const std::vector<std::string> &data);

  void waitCallback(const sensor_msgs::JointState::ConstPtr &current_joint_state);

  bool waitForEndGrasp(const sensor_msgs::JointState &joint_state);

  bool waitForEndMove(const sensor_msgs::JointState &joint_state);
};

#endif
