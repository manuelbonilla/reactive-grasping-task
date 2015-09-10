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

#include <XmlRpcValue.h>

#include "reactive_grasping_detection.h"

ReactiveGraspingDetection::ReactiveGraspingDetection() {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  // default notch filter parameters
  std::vector<double> a = {1, -0.98};
  std::vector<double> b = {1, -1};
  // retrieves settings from ROS params if specified
  private_node_handle_->param("verbose_mode", verbose_mode_, DEFAULT_VERBOSE_MODE);
  private_node_handle_->param("very_verbose_mode", very_verbose_mode_, DEFAULT_VERY_VERBOSE_MODE);
  private_node_handle_->param("only_detection", only_detection_, DEFAULT_ONLY_DETECTION);
  private_node_handle_->param("calibration", calibration_, DEFAULT_CALIBRATION);
  private_node_handle_->param("glove_topic_name", glove_topic_name_, std::string(DEFAULT_GLOVE_TOPIC_NAME));
  private_node_handle_->param("accel_map_topic_name", accel_map_topic_name_, std::string(DEFAULT_ACCEL_MAP_TOPIC_NAME));
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("filter_coeff_a", filter_coeff_a_, a);
  private_node_handle_->param("filter_coeff_b", filter_coeff_b_, b);
  private_node_handle_->param("num_imus", num_imus_, DEFAULT_NUM_IMUS);
  private_node_handle_->param("gravity_value", gravity_value_, DEFAULT_GRAVITY_VALUE);
  private_node_handle_->param("contact_threshold", contact_threshold_, (double)DEFAULT_CONTACT_THRESHOLD);
  private_node_handle_->param("window_size", window_size_, DEFAULT_WINDOW_SIZE);
  private_node_handle_->param("tails_scale_factor", tails_scale_factor_, (double)DEFAULT_TAILS_SCALE_FACTOR);
  private_node_handle_->param("delay_threshold", delay_threshold_, (double)DEFAULT_DELAY_THRESHOLD);
  private_node_handle_->param("action_server", action_server_, std::string(DEFAULT_ACTION_SERVER));
  private_node_handle_->param("log_file_base_path", log_file_base_path_, std::string(DEFAULT_LOG_FILE_BASE_PATH));
  private_node_handle_->param("log_file_name_raw", log_file_name_raw_, std::string(DEFAULT_LOG_FILE_NAME_RAW));
  private_node_handle_->param("log_file_name_filt", log_file_name_filt_, std::string(DEFAULT_LOG_FILE_NAME_FILT));
  private_node_handle_->param("log_file_name_map", log_file_name_map_, std::string(DEFAULT_LOG_FILE_NAME_MAP));

  // retrieves the comparison acceleration evolutions and their relative target pose of the end-effector
  parseAccelerationMap();

  // at the startup the raw and filtered acceleration structures are filled with 0 values
  reactive_grasping::AccelHistory starting_accel;
  starting_accel.x.resize(window_size_);
  starting_accel.y.resize(window_size_);
  starting_accel.z.resize(window_size_);
  starting_accel.abs_contribution.resize(window_size_);
  for (int i=0; i<num_imus_; i++) {
    accelerations_raw_.push_back(starting_accel);
    accelerations_filt_.push_back(starting_accel);
  }

  // stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
  std::time_t raw_time;
  char buffer[16];
  std::time(&raw_time);
  std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&raw_time));
  date_time_ = buffer;
  // creates folder if it doesn't exist
  std::string command = "mkdir -p " + log_file_base_path_;
  system(command.c_str());
  // opens log files
  log_file_accelerations_raw_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat");
  log_file_accelerations_filt_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat");
  log_file_accelerations_map_.open(log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat");

  // statistics variables initialization
  start_time_ = ros::Time::now();
  num_data_processed_ = 0;
  num_contacts_detected_ = 0;

  motion_action_client_ = new actionlib::SimpleActionClient<reactive_grasping::MotionAction>(node_handle_,
                                                                                             action_server_,
                                                                                             true);
  ROS_INFO_STREAM("[Detection] Waiting for Motion Action Server to be started...");
  motion_action_client_->waitForServer();
  ROS_INFO_STREAM("[Detection] Motion Action Server started.");

  ros::Duration(2).sleep();  // waits for Gazebo, rviz and all the stuff to load correctly

  // simulates a fake contact detection to let the robot reach the home pose
  contact_detected_ = true;
  generateAndSendGoal("first_homing","none");

  // subscribes to the sensorized Glove topic and initializes the action client
  accel_map_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>(accel_map_topic_name_, topic_queue_length_);
  glove_subscriber_ = node_handle_.subscribe(glove_topic_name_, topic_queue_length_,
                                             &ReactiveGraspingDetection::gloveMessageCallback, this);
  ROS_INFO_STREAM("[Detection] Node is retrieving Glove messages... (<ctrl+c> to terminate)");
}

ReactiveGraspingDetection::~ReactiveGraspingDetection() {
  ros::Time elapsed_time = ros::Time::now();

  // prints statistics (can't use rosconsole macros due to ros::shutdown() callback)
  std::cout << "\n\n";
  std::cout << "[ INFO] [" << elapsed_time << "]: ReactiveGrasping Node statistics..." << '\n';
  std::cout << "       + Total elapsed time: " << elapsed_time - start_time_ << '\n';
  std::cout << "       + Total data processed: " << num_data_processed_ << '\n';
  std::cout << "       + Total data rate: " << (double)num_data_processed_ / (elapsed_time-start_time_).toSec() << '\n';
  std::cout << "       + Total contacts detected: " << num_contacts_detected_ << '\n';

  // closes log files if previously opened
  if (log_file_accelerations_raw_.is_open()) {
    std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_raw_ + ".dat\n";
    log_file_accelerations_raw_.close();
  }
  if (log_file_accelerations_filt_.is_open()) {
    std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_filt_ + ".dat\n";
    log_file_accelerations_filt_.close();
  }
  if (log_file_accelerations_map_.is_open()) {
    std::cout << "       + Log file generated: " + log_file_base_path_ + date_time_ + "_" + log_file_name_map_ + ".dat\n";
    log_file_accelerations_map_.close();
  }
  std::cout << std::endl;

  delete motion_action_client_;
  delete private_node_handle_;
}

void ReactiveGraspingDetection::actionActiveCallback() {
  if (verbose_mode_) {
    ROS_INFO_STREAM("[Detection::actionActiveCallback] Goal just went active.");
  }
  goal_activate_time_ = ros::Time::now();
}

void ReactiveGraspingDetection::actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                   const reactive_grasping::MotionResultConstPtr &result) {
  ROS_INFO_STREAM("[Detection::actionDoneCallback] Action finished in state " << state.toString().c_str()
                  << "after " << ros::Time::now() - goal_activate_time_ << "seconds.");
  ROS_INFO_STREAM("[Detection::actionDoneCallback] Action result: " << result->info);
  if (verbose_mode_) {
    ROS_INFO_STREAM("[Detection::actionDoneCallback] Action final grasp: " << result->final_grasp_value);
    ROS_INFO_STREAM("[Detection::actionDoneCallback] Action final pose (x: " << result->final_pose.pose.position.x
                    << ", y: " << result->final_pose.pose.position.y
                    << ", z: " << result->final_pose.pose.position.z << ")");
  }
  motion_action_client_->stopTrackingGoal();
  contact_detected_ = false;
}

void ReactiveGraspingDetection::actionFeedbackCallback(const reactive_grasping::MotionFeedbackConstPtr &feedback) {
  if (verbose_mode_) {
    ROS_DEBUG_STREAM("[Detection::actionFeedbackCallback] Feedback info: " << feedback->info);
    ROS_DEBUG_STREAM("[Detection::actionFeedbackCallback] Feedback dist: " << std::fixed
                     << feedback->distance_to_the_goal);
    ROS_DEBUG_STREAM("[Detection::actionFeedbackCallback] Feedback grasp: " << feedback->grasp_value);

    if (very_verbose_mode_) {
      // TODO: output joint_state (sensor_msgs/JointState feedback->joint_state)
    }

    if (ros::Time::now() - goal_activate_time_ > ros::Duration(delay_threshold_)) {
      ROS_ERROR_STREAM("[Detection::actionFeedbackCallback] Goal has been active for more than " << delay_threshold_
                      << "seconds without getting reached. It is going to be aborted...");
      motion_action_client_->cancelGoal();
      motion_action_client_->stopTrackingGoal();
      contact_detected_ = false;
    }
  }
}

void ReactiveGraspingDetection::appendToLogFile(std::ofstream *log_file,
                                                const std::vector<reactive_grasping::AccelHistory> &data,
                                                const ros::Duration &acquisition_time) {
  // stores all linear accelerations in the given log file (first term is acquisition time)
  *log_file << std::fixed << acquisition_time;
  for (auto const &imu : data) {
    *log_file << ";" << imu.x.back()
              << ";" << imu.y.back()
              << ";" << imu.z.back();
  }
  *log_file << std::endl;
}

int ReactiveGraspingDetection::checkOscillation(int imu_id) {
  std::vector<double> values_x, values_y, values_z;
  for (int j=0; j<3; j++) {
    values_x.push_back(accelerations_filt_.at(imu_id).x.at(window_size_/3 + j));
    values_y.push_back(accelerations_filt_.at(imu_id).y.at(window_size_/3 + j));
    values_z.push_back(accelerations_filt_.at(imu_id).z.at(window_size_/3 + j));
  }

  // TODO: otherwise finds the most relevant contribute and only for it evaluates the oscillation
  if ((values_x[0] > contact_threshold_ /3 && (values_x[0]*values_x[1] < 0 || values_x[0]*values_x[2] < 0))
      || (values_y[0] > contact_threshold_ /3 && (values_y[0]*values_y[1] < 0 || values_y[0]*values_y[2] < 0))
      || (values_z[0] > contact_threshold_ /3 && (values_z[0]*values_z[1] < 0 || values_z[0]*values_z[2] < 0))) {
    return 0;
  }

  return -1;
}

int ReactiveGraspingDetection::detectContact() {
  // finds which IMU has the biggest 'abs_contribution' from all the axes (at 'window_size_/3' sample)
  std::vector<double> abs_values;
  for (auto const &imu : accelerations_filt_) {
    abs_values.push_back(imu.abs_contribution.at(window_size_/3));
  }
  std::vector<double>::const_iterator it_max = std::max_element(abs_values.begin(), abs_values.end());

  // evaluates if the max(abs(xyz)) is greater than a specific threshold
  if (*it_max > contact_threshold_) {
    int imu_id = it_max - abs_values.begin();
    // checks for outliers (following 2 samples must be relevant and at least one opposite in sign to the first)
    if (accelerations_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 1) > contact_threshold_
        && accelerations_filt_.at(imu_id).abs_contribution.at(window_size_/3 + 2) > contact_threshold_) {
      if (checkOscillation(imu_id) == 0) {
        num_contacts_detected_++;
        return imu_id;
      }
    }
  }

  return -1;
}

void ReactiveGraspingDetection::eraseOldestSample(std::vector<reactive_grasping::AccelHistory> &data) {
  for (auto &imu : data) {
    imu.x.erase(imu.x.begin());
    imu.y.erase(imu.y.begin());
    imu.z.erase(imu.z.begin());
    imu.abs_contribution.erase(imu.abs_contribution.begin());
  }
}

void ReactiveGraspingDetection::eraseOldestSample() {
  eraseOldestSample(accelerations_raw_);
  eraseOldestSample(accelerations_filt_);
}

std::string ReactiveGraspingDetection::extractGraspPrimitive(const std::vector<double> &data) {
  std::map<std::string, double> max_xcorr_values_map;
  std::vector<double> xcorr_values;

  for (auto const &pair : acceleration_map_) {
    // evaluates the cross-correlation with the lag limited to 'window_size_/2'
    xcorr_values = xcorr(data, pair.second.at("samples"), window_size_/2);
    // stores the absolute peak of cross-correlation in a map containing also the relative approaching direction
    std::for_each(xcorr_values.begin(), xcorr_values.end(), [](double &d){ d = std::abs(d); });
    auto it_max = std::max_element(xcorr_values.begin(), xcorr_values.end());
    max_xcorr_values_map.insert(std::make_pair(pair.first, *it_max));
  }
  auto it_max = std::max_element(max_xcorr_values_map.begin(), max_xcorr_values_map.end(),
                                 [](const std::map<std::string, double>::value_type x,
                                    const std::map<std::string, double>::value_type y){ return x.second < y.second; });
  return it_max->first;
}

geometry_msgs::Pose ReactiveGraspingDetection::fillTargetPose(std::string approaching_direction) {
  geometry_msgs::Pose target_pose;

  target_pose.position.x = acceleration_map_.at(approaching_direction).at("position").at(0);
  target_pose.position.y = acceleration_map_.at(approaching_direction).at("position").at(1);
  target_pose.position.z = acceleration_map_.at(approaching_direction).at("position").at(2);

  double roll = acceleration_map_.at(approaching_direction).at("orientation").at(0);
  double pitch = acceleration_map_.at(approaching_direction).at("orientation").at(1);
  double yaw = acceleration_map_.at(approaching_direction).at("orientation").at(2);
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(roll),
                                                                    angles::from_degrees(pitch),
                                                                    angles::from_degrees(yaw));
  return target_pose;
}

std::vector<double> ReactiveGraspingDetection::filter(std::vector<double> b, std::vector<double> a,
                                                      std::vector<double> x, std::vector<double> y) {
  if (x.size() < b.size() - 1) {
    ROS_WARN_STREAM("[Detection::filter] Measurements are shorter than parameters.");
    return y;
  }

  // builds first filtered values (to compute aj_yi for all aj in the following)
  if (y.size() < a.size() - 1) {
    if (y.empty()) {
      // y(1) = b1 * x(1) / a1
      y.push_back((b.front() * x.front()) / a.front());
    }

    std::vector<double>::const_iterator it_a = a.begin();
    std::vector<double>::const_iterator it_b = b.begin();
    while (y.size() < a.size() - 1) {
      // bj_xi = b1_xk + b2_xk-1 + ... + bnb_xk-nb-1
      double bj_xi = 0;
      for (std::vector<double>::const_iterator it_x = x.begin(); it_x != x.begin() + y.size(); it_x++) {
        bj_xi += *it_x * *(it_b + y.size() - (it_x - x.begin()));
      }

      // aj_yi = a1_yk-1 + a2_yk-2 + ... + ana_yk-na-1
      double aj_yi = 0;
      for (std::vector<double>::const_iterator it_y = y.begin(); it_y != y.end(); it_y++) {
        aj_yi += *it_y * *(it_a + y.size() - (it_y - y.begin()));
      }

      y.push_back((bj_xi - aj_yi) / a.front());
    }
  }

  int num_elements_x_exceed_y = x.size() - y.size();
  if (num_elements_x_exceed_y < 0) {  // few measuremens
    ROS_WARN_STREAM("[Detection::filter] Previous filtered data exceeds measurements.");
    return y;
  }
  if (num_elements_x_exceed_y == 0 && y.size() != 1) {  // already updated vector
    ROS_WARN_STREAM("[Detection::filter] There are no new measurements.");
    return y;
  }

  // there are new measurements to be evaluated
  for (std::vector<double>::const_iterator it_x = x.end() - num_elements_x_exceed_y; it_x != x.end(); it_x++) {
    // bj_xi = b1_xk + b2_xk-1 + ... + bnb_xk-nb-1
    double bj_xi = 0;
    for (std::vector<double>::const_iterator it_b = b.begin(); it_b != b.end(); it_b++) {
      bj_xi += *it_b * *(it_x - (it_b - b.begin()));
    }

    // aj_yi = a1_yk-1 + a2_yk-2 + ... + ana_yk-na-1
    std::vector<double>::const_iterator it_y = y.begin() + (it_x - x.begin());
    double aj_yi = 0;
    if (a.size() > 1) {
      for (std::vector<double>::const_iterator it_a = a.begin() + 1; it_a != a.end(); it_a++) {
        aj_yi += *it_a * *(it_y - (it_a - a.begin()));
      }
    }

    y.push_back((bj_xi - aj_yi) / a.front());
  }

  return y;
}

void ReactiveGraspingDetection::filterAccelerations() {
  std::vector<reactive_grasping::AccelHistory>::const_iterator it_raw = accelerations_raw_.begin();
  for (auto &imu : accelerations_filt_) {
    imu.x = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).x, imu.x);
    imu.y = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).y, imu.y);
    imu.z = filter(filter_coeff_b_, filter_coeff_a_, (*it_raw).z, imu.z);
    imu.abs_contribution.push_back(std::abs(imu.x.back()) + std::abs(imu.y.back()) + std::abs(imu.z.back()));

    it_raw++;
  }
}

void ReactiveGraspingDetection::generateAndSendGoal(std::string status, std::string approaching_direction) {
  reactive_grasping::MotionGoal goal;
  goal.info = status;
  if (status != "first_homing") {
    goal.target = fillTargetPose(approaching_direction);
  }
  motion_action_client_->sendGoal(goal, boost::bind(&ReactiveGraspingDetection::actionDoneCallback, this, _1, _2),
                                  boost::bind(&ReactiveGraspingDetection::actionActiveCallback, this),
                                  boost::bind(&ReactiveGraspingDetection::actionFeedbackCallback, this, _1));
}

void ReactiveGraspingDetection::gloveMessageCallback(const reactive_grasping::GloveIMUArray::ConstPtr &msg) {
  if (very_verbose_mode_) {
    printGloveIMUArray(msg->data);  // debug info
  }

  num_data_processed_++;

  processData(msg->data, msg->acquisition_time);
}

std::vector<double> ReactiveGraspingDetection::normalizeAccelerations(std::vector<double> data, int imu_id) {
  double max_value = *(std::max_element(data.begin(), data.end()));
  double min_value = *(std::min_element(data.begin(), data.end()));
  double max_abs_value = std::max(std::abs(max_value), std::abs(min_value));

  int sample_id=0;
  for (auto &sample : data) {
    sample /= max_abs_value;
    // scales only the samples not relative to the IMU 'imu_id'
    if (sample_id < imu_id*3*window_size_ || sample_id >= (imu_id+1)*3*window_size_) {
      sample *= tails_scale_factor_;
    }
    sample_id++;
  }

  return data;
}

void ReactiveGraspingDetection::parseAccelerationMap() {
  ROS_INFO_STREAM("[Detection::parseAccelerationMap] Parsing object parameters from YAML configuration file...");

  std::string base_name = "approaching_directions";
  std::vector<std::string> approaching_directions;
  XmlRpc::XmlRpcValue list;
  if (!private_node_handle_->getParam("/" + base_name, list)) {
    ROS_ERROR_STREAM("[Detection::parseAccelerationMap] Can't find '" + base_name + "' in YAML configuration file.");
    return;
  }
  for (auto it = list.begin(); it != list.end(); it++) {
    approaching_directions.push_back(it->first);
  }

  for (auto const &direction : approaching_directions) {
    std::string param_name;
    std::string field_name;
    std::vector<double> samples;
    std::vector<double> position;
    std::vector<double> orientation;
    std::map<std::string, std::vector<double>> map;

    field_name = "samples";
    param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
    if (!private_node_handle_->getParam(param_name, samples)) {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, samples));

    field_name = "position";
    param_name = "/"  + base_name + "/" + direction + "/"  + field_name;
    if (!private_node_handle_->getParam(param_name, position)) {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, position));

    field_name = "orientation";
    param_name = "/"  + base_name + "/"  + direction + "/"  + field_name;
    if (!private_node_handle_->getParam(param_name, orientation)) {
      ROS_WARN_STREAM("[Detection::parseAccelerationMap] Can't find '" + param_name + "' in YAML configuration file.");
      continue;
    }
    map.insert(std::make_pair(field_name, orientation));

    acceleration_map_.insert(std::make_pair(direction, map));
  }
}

void ReactiveGraspingDetection::printGloveIMUArray(const std::vector<reactive_grasping::GloveIMU> &data) {
  for (auto const &imu : data) {
    ROS_DEBUG_STREAM("[Detection::printGloveIMUArray] accelerometer[" << imu.id << "] = ["
                     << imu.linear_acceleration.x << "  "
                     << imu.linear_acceleration.y << "  "
                     << imu.linear_acceleration.z << "]");
  }
  for (auto const &imu : data) {
    ROS_DEBUG_STREAM("[Detection::printGloveIMUArray] gyroscope[" << imu.id << "] = ["
                     << imu.angular_velocity.x << "  "
                     << imu.angular_velocity.y << "  "
                     << imu.angular_velocity.z << "]");
  }

  if (!data.empty()) {
    ROS_DEBUG_STREAM("");  // adds one extra line to space the output
  }
}

void ReactiveGraspingDetection::processData(std::vector<reactive_grasping::GloveIMU> data,
                                            ros::Duration acquisition_time) {
  eraseOldestSample();
  pushBackNewSample(data);
  filterAccelerations();
  updateLogFiles(acquisition_time);

  if (only_detection_ || calibration_) {
    if (skip_samples_ == 0) {
      contact_detected_ = false;
    }
    else {
      skip_samples_--;
    }
  }

  // TODO: otherwise checks the trend of contact vector for each IMU: / \ \ _ \ >> probably direction is \ ?
  if (!contact_detected_) {
    int imu_id = detectContact();
    if (imu_id >= 0) {
      contact_detected_ = true;
      ROS_INFO_STREAM("[Detection::processData] Contact Detected on IMU " << imu_id);
      // concatenates the samples in a vector of length equals 3 x 'num_imus_' x 'window_size_'
      std::vector<double> accelerations_concatenated = toVector(accelerations_filt_);
      // normalizes the vector and scales its 'tails' down (emphasizes only peaks of imu with id = 'imu_id')
      std::vector<double> accelerations_normalized = normalizeAccelerations(accelerations_concatenated, imu_id);
      // evaluates the best grasp primitive throught cross-correlation with the data set
      std::string approaching_direction = extractGraspPrimitive(accelerations_normalized);
      ROS_INFO_STREAM("[Detection::processData] Cross-Correlation Detection on '" << approaching_direction << "'");

      if (!motion_action_client_->isServerConnected()) {
        ROS_ERROR_STREAM("[Detection::processData] Motion Action Server not connected...");
        return;
      }

      if (!only_detection_ && !calibration_) {
        // flags the detection
        generateAndSendGoal("sending target pose", approaching_direction);
      }

      std_msgs::Float64MultiArray accelerations_map;
      accelerations_map.data = accelerations_normalized;
      accel_map_publisher_.publish(accelerations_map);
      
      if (calibration_) {
        std::string answer = "";
        ROS_INFO_STREAM("[Detection::processData] Calibration: insert the current approaching direction ('n' to skip)");
        std::cin >> answer;
        if (answer == "n" || answer == "no" || answer == "N" || answer == "NO") {
          ROS_WARN_STREAM("[Detection::processData] Calibration: acceleration map rejected (not marked in the log).");
        }
        else {
          // highlight the acceleration maps to be stored in 'log_file_accelerations_map_'
          log_file_accelerations_map_ << "*** To be copied in the current 'comparison_dataset' ***\n"
                                      << "*** Real contact on: " << answer << " ***\n";
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      log_file_accelerations_map_ << std::fixed << "Acquisition time: " << acquisition_time
                                                << "\nContact detected on IMU: " << imu_id
                                                << "\nGrasp primitive extracted: " << approaching_direction
                                                << "\n" << toString(accelerations_normalized) << "\n" << std::endl;

      // avoid following false contact detection
      if (only_detection_ || calibration_) {
        skip_samples_ = window_size_;
      }
    }
  }
}

void ReactiveGraspingDetection::pushBackNewSample(const std::vector<reactive_grasping::GloveIMU> &new_sample) {
  std::vector<reactive_grasping::GloveIMU>::const_iterator it_new = new_sample.begin();
  for (auto &imu : accelerations_raw_) {
    imu.x.push_back((*it_new).linear_acceleration.x);
    imu.y.push_back((*it_new).linear_acceleration.y);
    imu.z.push_back((*it_new).linear_acceleration.z);
    imu.abs_contribution.push_back(std::abs(imu.x.back()) + std::abs(imu.y.back()) + std::abs(imu.z.back()));

    it_new++;
  }
}

std::string ReactiveGraspingDetection::toString(const std::vector<double> &data) {
  std::stringstream ss;
  std::copy(data.begin(), data.end(), std::ostream_iterator<double>(ss, " "));
  return ss.str();
}

std::vector<double> ReactiveGraspingDetection::toVector(const std::vector<reactive_grasping::AccelHistory> &data) {
  std::vector<double> data_vector;
  for (auto const &imu : data) {
    std::copy_n(imu.x.begin(), imu.x.size(), std::back_inserter(data_vector));
    std::copy_n(imu.y.begin(), imu.y.size(), std::back_inserter(data_vector));
    std::copy_n(imu.z.begin(), imu.z.size(), std::back_inserter(data_vector));
  }
  return data_vector;
}

void ReactiveGraspingDetection::updateLogFiles(const ros::Duration &acquisition_time) {
  appendToLogFile(&log_file_accelerations_raw_, accelerations_raw_, acquisition_time);
  appendToLogFile(&log_file_accelerations_filt_, accelerations_filt_, acquisition_time);
}

std::vector<double> ReactiveGraspingDetection::xcorr(std::vector<double> x, std::vector<double> y, int max_lag) {
  std::vector<double> xcorr_values;
  int num_samples = std::max(x.size(), y.size());

  if (x.size() > y.size()) {
    y.resize(x.size());
  }
  else if (x.size() < y.size()) {
    x.resize(y.size());
  }

  if (max_lag > num_samples) {
    max_lag = num_samples;
  }

  double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / num_samples;
  double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / num_samples;

  // calculates the denominator
  double sx = 0;
  double sy = 0;
  for (int i=0; i<num_samples; i++) {
    sx += (x.at(i) - mean_x) * (x.at(i) - mean_x);
    sy += (y.at(i) - mean_y) * (y.at(i) - mean_y);
  }
  double denominator = std::sqrt(sx*sy);

//  TODO: verify if lambdas do their job
//  std::for_each(x.begin(), x.end(), [&](double d){ return sx += (d-mean_x)*(d-mean_x); });
//  std::for_each(y.begin(), y.end(), [&](double d){ return sy += (d-mean_y)*(d-mean_y); });

  // calculates the Cross-Correlation series
  for (int lag=-max_lag; lag<max_lag; lag++) {
    double sxy = 0;
    for (int i=0; i<num_samples; i++) {
       int j = i + lag;
       if (j < 0 || j >= num_samples) {
          sxy += (x.at(i) - mean_x) * (-mean_y);
       }
       else {
          sxy += (x.at(i) - mean_x) * (y.at(j) - mean_y);
       }
    }
    // stores the new value computed for the current lag in the vector
    xcorr_values.push_back(sxy / denominator);
  }

  return xcorr_values;
}
