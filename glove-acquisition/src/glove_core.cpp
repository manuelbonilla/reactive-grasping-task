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

#include "glove_acquisition/glove_core.h"

GloveCore::GloveCore(std::string mode) {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  // publisher/subscriber info (from ROS params if specified)
  private_node_handle_->param("topic_name", topic_name_, std::string(DEFAULT_TOPIC_NAME));
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("verbose_mode", verbose_mode_, false);

  if (mode == "talker") {
    // glove configuration dependent ROS params
    private_node_handle_->param("serial_port", serial_port_, std::string(DEFAULT_SERIAL_PORT));
    private_node_handle_->param("start_comm_char", start_comm_char_, std::string(DEFAULT_START_COMM_CHAR));
    private_node_handle_->param("num_imus", num_imus_, DEFAULT_NUM_IMUS);
    private_node_handle_->param("num_digits", num_digits_, DEFAULT_NUM_DIGITS);

    // both 'A' and '*' commands have the first line of data deriving from accelerometers
    is_next_accel_data_ = start_comm_char_[0] != 'G';
    // if one type of data will be not retrieved, it remains constantly filled with 0 elements
    data_accelerometers_.resize(3*num_imus_);
    data_gyroscopes_.resize(3*num_imus_);

    // initializes the serial communication with the glove
    initializeCommunication();

    publisher_ = node_handle_.advertise<glove_acquisition::GloveIMUArray>(topic_name_, topic_queue_length_);
    ROS_INFO_STREAM("[Glove] Talker node is publishing messages... (<ctrl+c> to terminate)");
  }
  else if (mode == "listener") {
    private_node_handle_->param("log_file_name_accelerometers", log_file_name_accelerometers_,
                                std::string(DEFAULT_LOG_FILE_TERMINAL_NAME_ACCEL));
    private_node_handle_->param("log_file_name_gyroscopes", log_file_name_gyroscopes_,
                                std::string(DEFAULT_LOG_FILE_TERMINAL_NAME_GYRO));

    // if 'log_multi_files_' is enabled, the user has to handle the logging process inside the main code using methods
    // provided by this class and the 'Subject()' one. Otherwise any linstener node will build just two log files for
    // the entire acquisition (accelerometers + gyroscopes)
    private_node_handle_->param("log_multi_files", log_multi_files_, DEFAULT_LOG_MULTI_FILES);
    private_node_handle_->param("log_file_base_path", log_file_base_path_, std::string(DEFAULT_LOG_FILE_BASE_PATH));

    if (!log_multi_files_) {
      // stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
      std::time_t raw_time;
      char buffer[16];
      std::time(&raw_time);
      std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&raw_time));
      date_time_ = buffer;

      // creates folder if it doesn't exist
      std::string command = "mkdir -p " + log_file_base_path_;
      system(command.c_str());

      // opens the log files
      log_file_accelerometers_.open(log_file_base_path_ + date_time_ + log_file_name_accelerometers_ + ".dat");
      log_file_gyroscopes_.open(log_file_base_path_ + date_time_ + log_file_name_gyroscopes_ + ".dat");
    }

    // the 'log_files_map_' is initialized in both cases
    log_files_map_.insert(std::make_pair(log_file_name_accelerometers_, &log_file_accelerometers_));
    log_files_map_.insert(std::make_pair(log_file_name_gyroscopes_, &log_file_gyroscopes_));

    subscriber_ = node_handle_.subscribe(topic_name_, topic_queue_length_, &GloveCore::messageCallback, this);
    ROS_INFO_STREAM("[Glove] Listener node is retrieving messages... (<ctrl+c> to terminate)");
  }
  else {
    // string 'mode' is neither 'talker' nor 'listener' (unexpectedly)
    ROS_FATAL_STREAM("[Glove] Undefined string passed to the constructor");
    throw excp_;
  }

  // statistics variables initialization
  num_data_retrieved_ = 0;
  num_data_error_ = 0;
  num_log_files_ = 0;
  start_time_ = ros::Time::now();
}

GloveCore::~GloveCore() {
  ros::Time elapsed_time = ros::Time::now();

  std::cout << "\n\n";

  // closes log files
  if (!log_multi_files_) {
    std::string complete_path;
    std::ifstream last_file;

    complete_path = log_file_base_path_ + date_time_ + log_file_name_accelerometers_ + ".dat";
    if (log_file_accelerometers_.is_open()) {
      num_log_files_++;
      std::cout << "[ INFO] [" << elapsed_time << "]: [Glove] Log file generated: " << complete_path << '\n';
      log_file_accelerometers_.close();
    }
    // checks for emptiness (in case removes the file)
    last_file.open(complete_path);
    if (last_file.peek() == std::ifstream::traits_type::eof() && std::remove(complete_path.c_str()) == 0) {
      std::cout << "[ INFO] [" << elapsed_time
                << "]: [Glove] Log file was empty, thus has been removed: " << complete_path << '\n';
    }

    complete_path = log_file_base_path_ + date_time_ + log_file_name_gyroscopes_ + ".dat";
    if (log_file_gyroscopes_.is_open()) {
      num_log_files_++;
      std::cout << "[ INFO] [" << elapsed_time << "]: [Glove] Log file generated: " << complete_path << '\n';
      log_file_gyroscopes_.close();
    }
    // checks for emptiness (in case removes the file)
    last_file.open(complete_path);
    if (last_file.peek() == std::ifstream::traits_type::eof() && std::remove(complete_path.c_str()) == 0) {
      std::cout << "[ INFO] [" << elapsed_time
                << "]: [Glove] Log file was empty, thus has been removed: " << complete_path << '\n';
    }
  }

  // prints statistics (can't use rosconsole macros due to ros::shutdown() callback)
  std::cout << "[ INFO] [" << elapsed_time << "]: [Glove] Statistics..." << '\n';
  std::cout << "       + Total elapsed time: " << elapsed_time - start_time_ << '\n';
  std::cout << "       + Total data acquired: " << num_data_retrieved_ << '\n';
  std::cout << "       + Total data rate: " << (double)num_data_retrieved_ / (elapsed_time-start_time_).toSec() << '\n';
  std::cout << "       + Total data errors: " << num_data_error_ << '\n';
  std::cout << "       + Total log files created: " << num_log_files_ << std::endl;

  delete private_node_handle_;
}

int GloveCore::checkData() {
  char check_semicolon;
  int int_retrieved;
  last_sample_.clear();

  // for (unsigned int i=0; i<3*num_imus_; i++) {
  //   // every element of each line has 'num_digits_' digits and a terminating semicolon by default
  //   int ret_elements = sscanf(&data_temp_.at(i*(num_digits_ + 1)), "%d%c", &int_retrieved, &check_semicolon);
  //   // checks for error
  //   if (ret_elements != 2 || check_semicolon != ';') {
  //     std::stringstream ss;
  //     std::copy(data_temp_.begin(), data_temp_.end(), std::ostream_iterator<char>(ss, ""));
  //     ROS_ERROR_STREAM("[Glove] Data corrupted! (thus discarted) " + ss.str());
      
  //     num_data_error_++;
  //     // discards this record and clears temporary data
  //     data_temp_.clear();
  //     return -1;
  //   }
  //   last_sample_.push_back(int_retrieved);
  // }

  // // always clears temporary data; last_sample_ cointains the last record correctly stored
  // data_temp_.clear();

  int byte_IMU_ = 14;
  sizeBuffer_ = num_imus_ * byte_IMU_;
  dataBuffer_ = (new uint8_t[sizeBuffer_]);
  boost::asio::write(*serialPort_,boost::asio::buffer(new char('+'),1));
  boost::asio::read(*serialPort_, boost::asio::buffer(dataBuffer_, sizeBuffer_));


  int16_t   acc [num_imus_][3];
  int16_t   gyro[num_imus_][3];

  // int tmpX,tmpY,tmpZ;
  Eigen::Vector3d tmp;
  Eigen::Matrix3d Rx, Rz;
  // Rx(180)
  Rx << 1, 0, 0,
        0,-1, 0,
        0, 0,-1;

  // Rz(-90)
  Rz <<  0,1,0,
        -1,0,0,
         0,0,1;


  for(int i=0; i<num_imus_; i++)
  {
    acc[i][0] = (dataBuffer_[1+(byte_IMU_*i)]<<8 | dataBuffer_[2+(byte_IMU_*i)]);
    acc[i][1] = (dataBuffer_[3+(byte_IMU_*i)]<<8 | dataBuffer_[4+(byte_IMU_*i)]);
    acc[i][2] = (dataBuffer_[5+(byte_IMU_*i)]<<8 | dataBuffer_[6+(byte_IMU_*i)]);


    gyro[i][0] = (dataBuffer_[7 +(byte_IMU_*i)]<<8 | dataBuffer_[8 +(byte_IMU_*i)]);
    gyro[i][1] = (dataBuffer_[9 +(byte_IMU_*i)]<<8 | dataBuffer_[10+(byte_IMU_*i)]);
    gyro[i][2] = (dataBuffer_[11+(byte_IMU_*i)]<<8 | dataBuffer_[12+(byte_IMU_*i)]);

    tmp(0) = (int) acc[i][0];
    tmp(1) = (int) acc[i][1];
    tmp(2) = (int) acc[i][2];

    // the measurements are rotated in order to be homogeneous with the previous glove 
    tmp = Rz*Rx*tmp;

    data_accelerometers_.push_back(tmp(0));
    data_accelerometers_.push_back(tmp(1));
    data_accelerometers_.push_back(tmp(2));
    
    tmp(0) = (int) gyro[i][0];
    tmp(1) = (int) gyro[i][1];
    tmp(2) = (int) gyro[i][2];

    // the measurements are rotated in order to be homogeneous with the previous glove     
    tmp = Rz*Rx*tmp;

    data_gyroscopes_.push_back(tmp(0));
    data_gyroscopes_.push_back(tmp(1));
    data_gyroscopes_.push_back(tmp(2));
  }


  return 0;
}

int GloveCore::communicationControl(char comm_char) {
  if (comm_char == 'd') {  // 'd' means default
    comm_char = start_comm_char_[0];
  }
  // if (write(serial_port_fd_, &comm_char, 1) != 1) {
  // std::size_t flag =  boost::asio::write(*serialPort_,boost::asio::buffer(new char(comm_char),1));

  // std::size_t flag =  boost::asio::write(*serialPort_,comm_char );

  if ( boost::asio::write(*serialPort_,boost::asio::buffer(new char(comm_char),1)) == 0) {
    ROS_FATAL_STREAM("[Glove] communicationControl function (command '" << comm_char << "') error: " << errno);
    return -1;
  }
  return 0;
}

std::map<std::string, std::ofstream*> GloveCore::getLogFilesMap() {
  return log_files_map_;
}

bool GloveCore::getStatus() {
  return node_handle_.ok();
}

void GloveCore::initializeCommunication() {
  // file description for the glove (Arduino) serial port (with error handling)
  // serial_port_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  // if(serial_port_fd_ < 0) {
  //   ROS_FATAL_STREAM("[Glove] Unable to open " << serial_port_ << " port");
  //   throw excp_;
  // }
  // ROS_INFO_STREAM("[Glove] Established connection with the glove through " << serial_port_ << " port");
  // ROS_INFO_STREAM("[Glove] File descriptor correctly opened in read/write synchronized mode");

  // // initializes 'new_port_settings_' struct and sets IO baud rates, 8 bits, no parity, no stop bits
  // memset(&new_port_settings_, 0, sizeof(struct termios));
  // cfsetispeed(&new_port_settings_, B115200);
  // cfsetospeed(&new_port_settings_, B115200);
  // new_port_settings_.c_cflag &= ~(PARENB | PARODD);
  // new_port_settings_.c_cflag &= ~CSTOPB;
  // new_port_settings_.c_cflag &= ~CSIZE;
  // new_port_settings_.c_cflag |= CS8;
  
  // // flushes anything already in the serial buffer and commits the options
  // tcflush(serial_port_fd_, TCIFLUSH);
  // if (tcsetattr(serial_port_fd_, TCSANOW, &new_port_settings_) < 0) {
  //   ROS_FATAL_STREAM("[Glove] tcsetattr function error: " << errno);
  //   throw excp_;
  // }



  serialPort_ = (new boost::asio::serial_port(ioService_));
  serialPort_->close();
  serialPort_->open(serial_port_);
  serialPort_->set_option(boost::asio::serial_port_base::baud_rate(115200));
  serialPort_->set_option(boost::asio::serial_port_base::character_size(8));
  serialPort_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serialPort_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serialPort_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  tcflush(serialPort_->lowest_layer().native_handle(), TCIOFLUSH);

}

void GloveCore::messageCallback(const glove_acquisition::GloveIMUArray::ConstPtr &msg) {
  if (verbose_mode_) {
    printGloveIMUArray(msg->data);  // debug info
  }

  // please, make sure that a node does not use both this function and 'readAndPublish()' one
  num_data_retrieved_++;

  // stores all linear and angular accelerations in log files (first term is acquisition time)
  log_file_accelerometers_ << std::fixed << msg->acquisition_time;
  log_file_gyroscopes_ << std::fixed << msg->acquisition_time;
  for (auto const& imu : msg->data) {
    log_file_accelerometers_ << ";" << imu.linear_acceleration.x 
                             << ";" << imu.linear_acceleration.y 
                             << ";" << imu.linear_acceleration.z;

    log_file_gyroscopes_ << ";" << imu.angular_velocity.x 
                         << ";" << imu.angular_velocity.y 
                         << ";" << imu.angular_velocity.z;
  }
  log_file_gyroscopes_ << std::endl;
  log_file_accelerometers_ << std::endl;
}

void GloveCore::printGloveIMUArray(const std::vector<glove_acquisition::GloveIMU> &data) {
  for (auto const& imu : data) {
    ROS_DEBUG_STREAM("[Glove] accelerometer[" << imu.id << "] = [" 
                                              << imu.linear_acceleration.x << "  " 
                                              << imu.linear_acceleration.y << "  " 
                                              << imu.linear_acceleration.z << "]");
  }
  for (auto const& imu : data) {
    ROS_DEBUG_STREAM("[Glove] gyroscope[" << imu.id << "] = [" 
                                          << imu.angular_velocity.x << "  " 
                                          << imu.angular_velocity.y << "  " 
                                          << imu.angular_velocity.z << "]");
  }

  if (!data.empty()) {
    ROS_DEBUG_STREAM("");  // adds one extra line to space the output
  }
}

void GloveCore::publishMessage() {
  // 'imus' message contains a vector of 'imu' messages
  glove_acquisition::GloveIMU imu;
  glove_acquisition::GloveIMUArray imus;
  
  imus.acquisition_time = ros::Time::now() - start_time_;
  imus.header.stamp = ros::Time::now();
  imus.header.frame_id = "/glove_world";

  // for (unsigned int i=0; i<num_imus_; i++) {
  //   imu.id = i;

  //   imu.linear_acceleration.x = data_accelerometers_.at(0 + i*3);
  //   imu.linear_acceleration.y = data_accelerometers_.at(1 + i*3);
  //   imu.linear_acceleration.z = data_accelerometers_.at(2 + i*3);

  //   imu.angular_velocity.x = data_gyroscopes_.at(0 + i*3);
  //   imu.angular_velocity.y = data_gyroscopes_.at(1 + i*3);
  //   imu.angular_velocity.z = data_gyroscopes_.at(2 + i*3);
    
  //   imus.data.push_back(imu);
  // }

  int c = 0;
  for (int i=num_imus_-1; i>=0; --i) 
  {
    imu.id = num_imus_ - i -1;

    imu.linear_acceleration.x = data_accelerometers_.at(0 + i*3);
    imu.linear_acceleration.y = data_accelerometers_.at(1 + i*3);
    imu.linear_acceleration.z = data_accelerometers_.at(2 + i*3);

    imu.angular_velocity.x = data_gyroscopes_.at(0 + i*3);
    imu.angular_velocity.y = data_gyroscopes_.at(1 + i*3);
    imu.angular_velocity.z = data_gyroscopes_.at(2 + i*3);
    
    imus.data.push_back(imu);
  }

  
  data_accelerometers_.clear();
  data_gyroscopes_.clear();

  if (verbose_mode_){
    printGloveIMUArray(imus.data);  // debug info
  }

  publisher_.publish(imus);
}

int GloveCore::readAndPublish() {
  // if (retrieveData() != 0) {
  //   return -1;  // there is more data to be stored
  // }

  // checks if data is correctly formatted (accelerometers and gyroscopes data have the same format)
  if (checkData() != 0) {
    if (start_comm_char_[0] == '*') {
      // if next == accelerometers: following gyroscpes data has to be invalidated (the flag remains on accelerometers)
      if (is_next_accel_data_) {
        ROS_WARN_STREAM("[Glove] Skipping gyroscopes data and continue the acquisition.");
        while (retrieveData() != 0) ;
      }
      // if next == gyroscopes: switch the flag to accelerometers (data would be lost correctly thanks to return -1)
      else {
        ROS_WARN_STREAM("[Glove] Switch the data type selector and continue the acquisition.");
        is_next_accel_data_ = !is_next_accel_data_;
      }
    }
    return -1;  // data would not be published
  }

  // stores retrieved data into the proper std::vector
  // if (is_next_accel_data_) {
  //   data_accelerometers_ = last_sample_;
  // }
  // else {
  //   data_gyroscopes_ = last_sample_;
  // }

  // if there are both accelerometers and gyroscopes data this function is called twice per acquisition cycle
  // if (start_comm_char_[0] == '*') {
  //   is_next_accel_data_ = !is_next_accel_data_;
  //   if (!is_next_accel_data_) {
  //     return -1;  // also data_gyroscopes_ has to be stored
  //   }
  // }

  // please, make sure that a node does not use both this function and 'gloveMessageCallback(&msg)' one
  num_data_retrieved_++;

  publishMessage();
  return 0;
}

int GloveCore::retrieveData() {
  char char_read;

  do {
    if (read(serial_port_fd_, &char_read, 1) > 0) {
      // data arrives randomly, hence we have to store temporary data in an class variable
      data_temp_.push_back(char_read);
      continue;
    }
    else {
      return -1;  // there is more data to be stored
    }
  } while (char_read != '\n');

  return 0;
}

void GloveCore::updateNumLogFiles(int num_to_add) {
  if (std::abs(num_to_add) > log_files_map_.size()) {
    ROS_ERROR_STREAM("[Glove] The number of log files to be added (" << num_to_add <<
                     ") exceeds the number of actual log files (" << log_files_map_.size() << ")");
    return;
  }
  num_log_files_ += num_to_add;
}
