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

#include "reactive_grasping_motion.h"

ReactiveGraspingMotion::ReactiveGraspingMotion() {
  // Handles server private parameters (private name are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  // retrieves settings from ROS params if specified
  private_node_handle_->param("hand_synergy_joint", hand_synergy_joint_, std::string(DEFAULT_HAND_SYNERGY_JOINT));
  private_node_handle_->param("hand_synergy_topic", hand_synergy_topic_, std::string(DEFAULT_HAND_SYNERGY_TOPIC));
  private_node_handle_->param("joint_state_topic", joint_state_topic_, std::string(DEFAULT_JOINT_STATE_TOPIC));
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("action_server", action_server_, std::string(DEFAULT_ACTION_SERVER));
  private_node_handle_->param("move_group", move_group_, std::string(DEFAULT_MOVE_GROUP));
  private_node_handle_->param("move_group_home", move_group_home_, std::string(DEFAULT_MOVE_GROUP_HOME));
  private_node_handle_->param("frame_base", frame_base_, std::string(DEFAULT_FRAME_BASE));
  private_node_handle_->param("frame_ee_kuka", frame_ee_kuka_, std::string(DEFAULT_FRAME_EE_KUKA));
  private_node_handle_->param("frame_ee_glove", frame_ee_glove_, std::string(DEFAULT_FRAME_EE_GLOVE));
  private_node_handle_->param("time_for_grasp", time_for_grasp_, (double)DEFAULT_TIME_FOR_GRASP);
  private_node_handle_->param("verbose_mode", verbose_mode_, DEFAULT_VERBOSE_MODE);
  private_node_handle_->param("very_verbose_mode", very_verbose_mode_, DEFAULT_VERY_VERBOSE_MODE);
  private_node_handle_->param("arm_velocity_threshold", arm_velocity_threshold_, DEFAULT_ARM_VELOCITY_THRESHOLD);
  private_node_handle_->param("arm_distance_threshold", arm_distance_threshold_, DEFAULT_ARM_DISTANCE_THRESHOLD);
  private_node_handle_->param("hand_velocity_threshold", hand_velocity_threshold_, DEFAULT_HAND_VELOCITY_THRESHOLD);
  private_node_handle_->param("hand_distance_threshold", hand_distance_threshold_, DEFAULT_HAND_DISTANCE_THRESHOLD);
  private_node_handle_->param("detect_contact_delay", detect_contact_delay_, (double)DEFAULT_DETECT_CONTACT_DELAY);

  hand_grasped_ = 0.0;
  homing_ = false;
  arm_moving_ = false;
  hand_moving_ = false;
  group_ = new moveit::planning_interface::MoveGroupInterface(move_group_);

  server_ = new actionlib::SimpleActionServer<reactive_grasping::MotionAction>(node_handle_, action_server_, false);
  server_->registerGoalCallback(boost::bind(&ReactiveGraspingMotion::goalCallback, this));
  server_->registerPreemptCallback(boost::bind(&ReactiveGraspingMotion::preemptCallback, this));
  server_->start();

  joints_subscriber_ = node_handle_.subscribe(joint_state_topic_, topic_queue_length_,
                                              &ReactiveGraspingMotion::waitCallback, this);
  hand_synergy_publisher_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(hand_synergy_topic_,
                                                                                     topic_queue_length_);
}

ReactiveGraspingMotion::~ReactiveGraspingMotion() {
  delete group_;
  delete server_;
  delete private_node_handle_;
}

geometry_msgs::PoseStamped ReactiveGraspingMotion::changeFrame(std::string to_frame, std::string from_frame,
                                                               const geometry_msgs::Pose &from_pose_msg) {
  geometry_msgs::PoseStamped to_pose_msg;
  tf::StampedTransform change_frame_stamped_transform;

  try {  // retrieves the homogeneous transform between the two given frames
    tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(0), change_frame_stamped_transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM("[Motion::goalCallback] Exception catched: " << ex.what());
  }
  tf::Transform change_frame_transform = change_frame_stamped_transform;

  // computes the new pose (p2 = T12 * p1 * inv(T12))
  tf::Pose from_pose;
  poseMsgToTF(from_pose_msg, from_pose);
  tf::Pose to_pose = change_frame_transform * from_pose * change_frame_transform.inverse();
  poseTFToMsg(to_pose, to_pose_msg.pose);
  to_pose_msg.header.frame_id = to_frame;

  return to_pose_msg;
}

bool ReactiveGraspingMotion::checkForEndGrasp(const sensor_msgs::JointState &current_joint_state) {
  std::vector<std::string> selected_joint_names = {hand_synergy_joint_};
  sensor_msgs::JointState reduced_joint_state = selectJoints(selected_joint_names, current_joint_state);

  if (last_joint_state_.name.empty()) {
    last_joint_state_ = reduced_joint_state;
    if (verbose_mode_) {
      ROS_DEBUG_STREAM("[Motion::checkForEndGrasp] First time called: the 'last_joint_state_' has been initilized.");
    }
    return false;
  }

  // the target trajectory for the grasp is usually set as the one which lead the hand fully grasped with no object
  // inside; however, with something to be grasped, the hand can't reach that final configuration; for this reason
  // the grasp is consider to be completed when two distinct following joint states are enough similar (threshold)
  if (std::norm(reduced_joint_state.position.at(0) - last_joint_state_.position.at(0)) < hand_distance_threshold_) {
    ROS_INFO_STREAM("[Motion::checkForEndGrasp] Grasp complete.");

    // the 'last_joint_state_' must be cleared for next calls
    sensor_msgs::JointState clear_joint_state;
    last_joint_state_ = clear_joint_state;
    return true;
  }

  if (verbose_mode_) {
    ROS_DEBUG_STREAM("[Motion::checkForEndGrasp]  The distance to the goal is greater than threshold ("
                     << std::norm(reduced_joint_state.position.at(0) - last_joint_state_.position.at(0)) << ")");
  }
  last_joint_state_ = reduced_joint_state;
  return false;
}

bool ReactiveGraspingMotion::checkForEndMove(const sensor_msgs::JointState &current_joint_state) {
  trajectory_msgs::JointTrajectory joint_trajectory = plan_to_target_.trajectory_.joint_trajectory;
  std::vector<std::string> selected_joint_names = joint_trajectory.joint_names;
  sensor_msgs::JointState joint_state = selectJoints(selected_joint_names, current_joint_state);

  if (joint_trajectory.points.size() == 0) {
    ROS_WARN_STREAM("[Motion::checkForEndMove] There is no trajectory to be executed.");
    return false;
  }

  // checks if joint names are correctly sorted and matches each other
  if (joint_trajectory.joint_names != joint_state.name) {
    if (joint_trajectory.joint_names.size() != joint_state.name.size()) {
      ROS_FATAL_STREAM("[Motion::checkForEndMove] Joint sizes are not equal");
    }
    else {
      ROS_FATAL_STREAM("[Motion::checkForEndMove] Joint names do not match");
    }
    if (verbose_mode_) {
      ROS_FATAL_STREAM("[Motion::checkForEndMove] Joint trajectory names : " + toString(joint_trajectory.joint_names));
      ROS_FATAL_STREAM("[Motion::checkForEndMove] Joint state names : " + toString(joint_state.name));
    }
    return false;
  }

  // checks current joint velocities
  double velocity_sum = 0;
  for (auto const &vel : joint_state.velocity) {
    velocity_sum += std::abs(vel);
  }
  if (velocity_sum > arm_velocity_threshold_) {
    if (verbose_mode_) {
      ROS_DEBUG_STREAM("[Motion::checkForEndMove] Velocity greater than threshold (" << velocity_sum << ")");
    }
    return false;
  }

  // checks current distance from the target pose (joint velocities are smaller than their threshold)
  int index = 0;
  double distance_sum = 0;
  for (auto const &pos : joint_state.position) {
    distance_sum += std::norm(pos - joint_trajectory.points.back().positions.at(index++));
  }
  if (distance_sum > arm_distance_threshold_) {
    if (verbose_mode_) {
      ROS_DEBUG_STREAM("[Motion::checkForEndMove] Distance to the goal greater than threshold (" << distance_sum << ")");
    }
    return false;
  }

  ROS_INFO_STREAM("[Motion::checkForEndMove] Target pose reached (vel: " << velocity_sum
                  << ", dist: " << distance_sum << ")");
  return true;
}

void ReactiveGraspingMotion::generateAndPublishFeedback(std::string status, const sensor_msgs::JointState &joint_state) {
  std::vector<std::string> selected_joint_names = {hand_synergy_joint_};
  sensor_msgs::JointState hand_synergy_joint_state = selectJoints(selected_joint_names, joint_state);

  reactive_grasping::MotionFeedback feeback;
  feeback.info = status;
  geometry_msgs::PoseStamped current_pose = group_->getCurrentPose();
  feeback.distance_to_the_goal = std::sqrt(std::norm(current_target_pose_.pose.position.x - current_pose.pose.position.x) +
                                           std::norm(current_target_pose_.pose.position.y - current_pose.pose.position.y) +
                                           std::norm(current_target_pose_.pose.position.z - current_pose.pose.position.z));
  feeback.grasp_value = std::round(100*std::abs(hand_synergy_joint_state.position.at(0)))/100;
  feeback.joint_state = joint_state;
  server_->publishFeedback(feeback);
}

void ReactiveGraspingMotion::generateAndPublishResult(std::string status, const sensor_msgs::JointState &joint_state) {
  std::vector<std::string> selected_joint_names = {hand_synergy_joint_};
  sensor_msgs::JointState hand_synergy_joint_state = selectJoints(selected_joint_names, joint_state);

  reactive_grasping::MotionResult result;
  result.info = status;
  result.final_grasp_value = std::round(100*std::abs(hand_synergy_joint_state.position.at(0)))/100;
  result.final_pose = group_->getCurrentPose();

  ros::Duration(detect_contact_delay_).sleep();  // due to kuka and hand oscillations (SoftHand dependent)
  server_->setSucceeded(result);
}

void ReactiveGraspingMotion::goalCallback() {
  reactive_grasping::MotionGoalConstPtr goal = server_->acceptNewGoal();

  if (hand_grasped_ || goal->info == "first_homing") {
    if (goal->info == "first_homing" && verbose_mode_) {
      ROS_DEBUG_STREAM("[Motion::goalCallback] First time homing... (this may take a while)");
    }
    homing();
    if (goal->info != "first_homing") {
      current_target_pose_ = home_pose_;
    }
    return;
  }

  if (!hand_grasped_) {  // new grasp primitive has to be performed
    ROS_DEBUG_STREAM("[Motion::goalCallback] New goal recieved.");
    ROS_DEBUG_STREAM("[Motion::goalCallback] Evaluating the final target pose of the end-effector.");

    // updates 'current_target_pose_' with the goal pose expressed in the world frame 'frame_base_' (xyz-RPY data is
    // measured in an approach-sliding-normal-xyz frame: 'frame_ee_glove_')
    geometry_msgs::PoseStamped target_pose = changeFrame(frame_ee_kuka_, frame_ee_glove_, goal->target);
    tf_listener_.transformPose(frame_base_, target_pose, current_target_pose_);

    moveToTargetPose(current_target_pose_);
    return;
  }
}

void ReactiveGraspingMotion::grasp(double command_value) {
  if (command_value < 0) {
    ROS_WARN_STREAM("[Motion::grasp] The given command value (" << command_value
                    << ") is smaller than the minimum (0.0) which has been set");
    command_value = 0.0;
  }
  else if (command_value > 1) {
    ROS_WARN_STREAM("[Motion::grasp] The given command value (" << command_value
                    << ") is greater than the maximum (1.0) which has been set");
    command_value = 1.0;
  }

  hand_grasped_ = command_value;
  hand_moving_ = true;

  trajectory_msgs::JointTrajectory grasp_trajectory;
  trajectory_msgs::JointTrajectoryPoint trajectory_points;

  trajectory_points.positions.push_back(hand_grasped_);
  trajectory_points.time_from_start = ros::Duration(time_for_grasp_);

  grasp_trajectory.joint_names.push_back(hand_synergy_joint_);
  grasp_trajectory.points.push_back(trajectory_points);

  hand_synergy_publisher_.publish(grasp_trajectory);
}

void ReactiveGraspingMotion::homing() {
  homing_ = true;
  grasp(0.0); // ungrasp
  hand_moving_ = false;  // does not need to check for end position (ungrasp is faster than robot motion)

  ROS_INFO_STREAM("[Motion::homing] Ungrasping and planning the motion of the end-effector to the initial pose...");
  group_->setNamedTarget(move_group_home_);

  while (!planAndExecute()) {  // TODO: timeout?
    ROS_ERROR_STREAM("[Motion::homing] Waiting for homing...");
  }
}

void ReactiveGraspingMotion::moveToTargetPose(geometry_msgs::PoseStamped target_pose) {
  ROS_INFO_STREAM("[Motion::moveToTargetPose] Planning to the grasp pose...");

  group_->setPoseTarget(target_pose);
  if (!planAndExecute()) {
    ROS_ERROR_STREAM("[Motion::moveToTargetPose] Something went wrong. Aborting goal...");
    server_->setAborted();
  }
}

bool ReactiveGraspingMotion::planAndExecute() {
  // finds a trajectory to reach the target pose
  if (!group_->plan(plan_to_target_)) {
    ROS_ERROR_STREAM("[Motion::planAndExecute] Faiulre in the planning to the target pose.");
    return false;
  }

  // executes the planned trajectory in an asynchronous non-blocking mode
  ROS_INFO_STREAM("[Motion::planAndExecute] Executing the trajectory to the target pose...");
  arm_moving_ = true;
  return group_->asyncExecute(plan_to_target_);
}

void ReactiveGraspingMotion::preemptCallback() {
  ROS_WARN_STREAM("[Motion::preemptCallback] Preempted...");

  group_->stop();
  homing_ = false;
  hand_moving_ = false;
  arm_moving_ = false;

  homing();
  server_->setPreempted();
}

sensor_msgs::JointState ReactiveGraspingMotion::selectJoints(const std::vector<std::string> &joint_names,
                                                             const sensor_msgs::JointState &joint_state) {
  sensor_msgs::JointState reduced_joint_state;
  reduced_joint_state.header = joint_state.header;

  for (auto const &joint : joint_names) {
    int id_joint = std::find(joint_state.name.begin(), joint_state.name.end(), joint) - joint_state.name.begin();
    if (id_joint == joint_state.name.size()) {
      ROS_FATAL_STREAM("[Motion::selectJoints] Joint " + joint + " not found.");
      if (verbose_mode_) {
        ROS_FATAL_STREAM("[Motion::selectJoints] Joint state names : " + toString(joint_state.name));
      }
    }

    reduced_joint_state.name.push_back(joint);
    reduced_joint_state.position.push_back(joint_state.position.at(id_joint));
    reduced_joint_state.velocity.push_back(joint_state.velocity.at(id_joint));
    reduced_joint_state.effort.push_back(joint_state.effort.at(id_joint));
  }

  return reduced_joint_state;
}

std::string ReactiveGraspingMotion::toString(const std::vector<std::string> &data) {
  std::stringstream ss;
  std::copy(data.begin(), data.end(), std::ostream_iterator<std::string>(ss, " "));
  return ss.str();
}

void ReactiveGraspingMotion::waitCallback(const sensor_msgs::JointState::ConstPtr &current_joint_state) {
  if (!server_->isActive()) {
    if (very_verbose_mode_) {
      ROS_WARN_STREAM("[Motion::waitCallback] There is no goal active at the moment.");
    }
    return;
  }

  if (arm_moving_) {
    if (waitForEndMove(*current_joint_state)) {
      ROS_INFO("[Motion::waitCallback] Homing succeeded!");
    }
    return;
  }

  if (hand_moving_) {
    if (waitForEndGrasp(*current_joint_state)) {
      ROS_INFO("[Motion::waitCallback] Grasp succeeded!");
    }
    //sleep(0.9);
    return;
  }
}

bool ReactiveGraspingMotion::waitForEndGrasp(const sensor_msgs::JointState &joint_state) {
  if (checkForEndGrasp(joint_state)) {
    hand_moving_ = false;
    sleep(1.0);
    generateAndPublishResult("grasp accomplished", joint_state);
    
    return true;
  }
  else {
    generateAndPublishFeedback("hand still moving", joint_state);
  }
  return false;
}

bool ReactiveGraspingMotion::waitForEndMove(const sensor_msgs::JointState &joint_state) {
  if (checkForEndMove(joint_state)) {
    arm_moving_ = false;
    group_->stop();

    if (!homing_) {
      grasp(1.0); // full grasp
    }
    else {
      homing_ = false;
      generateAndPublishResult("homing accomplished", joint_state);
      home_pose_ = group_->getCurrentPose();
      return true;
    }
  }
  else {
    generateAndPublishFeedback("arm still moving", joint_state);
  }
  return false;
}
