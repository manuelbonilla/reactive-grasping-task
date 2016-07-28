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

#ifndef GUARD_GLOVE_CORE_H
#define GUARD_GLOVE_CORE_H

// standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <exception>
#include <vector>
#include <map>
// eigen libraries
#include <Eigen/Core>
#include <Eigen/Geometry>
// boost libraries
#include <boost/chrono.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio/serial_port.hpp> 
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
// auto-generated from msg/ directory libraries
#include "glove_acquisition/GloveIMU.h"
#include "glove_acquisition/GloveIMUArray.h"

// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_TOPIC_NAME "glove_topic"
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_NUM_IMUS 5
#define DEFAULT_NUM_DIGITS 6
#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define DEFAULT_START_COMM_CHAR "A"
#define DEFAULT_LOG_FILE_BASE_PATH "logs/"
#define DEFAULT_LOG_FILE_TERMINAL_NAME_ACCEL "_glove_accelerometers"
#define DEFAULT_LOG_FILE_TERMINAL_NAME_GYRO "_glove_gyroscopes"
#define DEFAULT_LOG_MULTI_FILES false

struct GloveCoreException : std::exception {
  const char* what() const noexcept {return "GloveCore exception";}
};

/*  This class purpose is to provide a ROS interface which lets to retrieve data (IMU linear accelerations and/or
 *  angular velocities) from the '5 IMUs Glove' throught the Arduino Micro serial communication and publish it on
 *  a topic using 'glove_acquisition::GloveIMUArray' messages. This data can be collected in log files if necessary.
 *  Each line of data from the glove is as follow: [x1;y1;z1;x2;...;z5], where each field is a 6 digits integer by
 *  default (have a look to the Arduino Sketch to understand better how data is sent over the serial stream). If
 *  both accelerometers and gyroscopes are enabled, two distinct same-format-lines are sent on the stream.
 *  The class provides several parameters which can be set by the user at runtime to handle even distinct hardware
 *  configuration (e.g., different number of IMUs connected to the glove).
 *
 *  If you edit this class, please try to follow these C++ style guidelines: http://wiki.ros.org/CppStyleGuide.
 *
 *  ROS params:
 *    + topic_name
 *    + topic_queue_length
 *    + verbose_mode
 *    + num_imus
 *    + num_digits
 *    + serial_port
 *    + start_comm_char
 *    + log_file_base_path
 *    + log_file_name_accelerometers
 *    + log_file_name_gyroscopes
 *    + log_multi_files
 */
class GloveCore {
 public:
  /*  The constructor retrieves the class parameters if specified by the user (default values otherwise) and can be
   *  used to create two distinct purpose object:
   *    + talker: initializes the 'publisher_' to publish 'glove_acquisition::GloveIMUArray' messages on the
   *      'topic_name_' topic (which is provided as a ROS param) and call the 'initializeCommunication()' method to
   *      start the serial communication with the glove.
   *    + listener: initializes the 'subscriber_' with the 'GloveCore::messageCallback()' method to retrieves messages
   *      on the 'topic_name_' topic (which is provided as a ROS param). Also, initializes the log file structures
   *      where will be stored retrieved message data.
   *  In both cases some statistics variables are initialized.
   *  The destructor prints on screen the statistics.
   *
   *  Parameters:
   *    + mode: specifies if the new istance has to be a publisher node ("talker") or a subscriber one ("listener").
   *  Exceptions:
   *    + directly if mode != "talker" || mode != "listener".
   *    + through 'initializeCommunication()' function.
   *  Other methods called:
   *    + initializeCommunication()
   *  Main private variables modified:
   *    + publisher_ (iff mode == "talker")
   *    + subscriber_ (iff mode == "listener")
   */
  GloveCore(std::string mode);
  ~GloveCore();

  /*  Starts/stops data stream from the glove board. It must be called only by the "talker" in its 'main' before and
   *  after the acquisition loop.
   *
   *  Parameters:
   *    + char_start: 
   *      - 'A': sends only accelerometers data,
   *      - 'G': sends only gyroscopes data (not used in this demo),
   *      - '*': sends both accelerometers and gyroscopes data (not used in this demo),
   *      - 'd': default, gets its value from 'start_comm_char' ROS param (DEFAULT_START_COMM_CHAR = 'A'),
   *      - '!': stop the stream.
   *  Return value:
   *    + 0, on success.
   */
  int communicationControl(char char_start);

  /*  Simple method to retrieve 'log_files_map_' (private variable). Only the 'listener' can use this method.
   *
   *  Return value:
   *    + log_files_map_, each field is made up by the log file terminal name and its std::ofstream pointer.
   */
  std::map<std::string, std::ofstream*> getLogFilesMap();

  /*  Simple method to check the ROS status.
   *
   *  Return value:
   *    + true, if ROS is still running.
   */
  bool getStatus();

  /*  Retrieves data from the glove, checks if it properly formatted and publishes a 'glove_acquisition::GloveIMUArray'
   *  message on the 'topic_name_' topic (which is set in the constructor through the relative ROS param 'topic_name').
   *  Data could be either linear accelerations or angular velocities (and even both together) depending on what has
   *  been passed to the 'communicationControl()' method.
   *  It must be called only by the 'talker' in its main inside the acquisition loop.
   *
   *  Return value:
   *    + 0, on success.
   *  Other methods called:
   *    + retrieveData()
   *    + checkData()
   *    + publishMessage()
   *  Main private variables modified:
   *    + is_next_accel_data_
   *    + num_data_retrieved_
   */
  int readAndPublish();

  /*  Updates the number of log files already generated in a multi logs environment (it must be used iff
   *  'log_multi_files_' is set to true in the constructor through the relative ROS param 'log_multi_files').
   *  Only the 'listener' can use this method.
   *
   *  Parameters:
   *    + num_to_add: number of new files to be added to the existing one.
   *  Main private variables modified:
   *    + num_log_files_
   */
  void updateNumLogFiles(int num_to_add);

 private:
  ros::NodeHandle *private_node_handle_;
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;

  // glove topic variables
  std::string topic_name_;
  int topic_queue_length_;

  // glove configuration variables
  int num_imus_;
  int num_digits_;
  std::string serial_port_;
  std::string start_comm_char_;
  int serial_port_fd_;
  struct termios new_port_settings_;

  // glove configuration variables [PSOC 3 - IMUboard]
   boost::asio::serial_port* serialPort_;
  boost::asio::io_service ioService_;
  int sizeBuffer_;
  uint8_t* dataBuffer_; 


  // glove reading variables
  std::vector<char> data_temp_;  // it could be filled in more acquisition cycles (if the serial stream is empty)
  std::vector<int> last_sample_;
  std::vector<int> data_accelerometers_;
  std::vector<int> data_gyroscopes_;
  bool is_next_accel_data_;  // flags if the next data to be stored derives from accelerometers

  // statistics variables
  ros::Time start_time_;
  int num_data_retrieved_;
  int num_data_error_;
  int num_log_files_;

  // log files
  std::string log_file_name_accelerometers_;
  std::string log_file_name_gyroscopes_;
  std::ofstream log_file_accelerometers_;
  std::ofstream log_file_gyroscopes_;
  bool log_multi_files_;
  std::string date_time_;
  std::string log_file_base_path_;  // used only if !log_multi_files_ (handled automatically otherwise)
  std::map<std::string, std::ofstream*> log_files_map_;  // terminal file name with the proper std::ofstream pointer

  // other stuff
  bool verbose_mode_;
  GloveCoreException excp_;


  /*  Checks if last retrieved data stored in 'data_temp_' is correctly formatted. This function is glove dependent,
   *  thus it is highly recommanded to verify if it matches the actual glove output format as well as the number of
   *  IMUs of which the glove is provided. Those settings are provided as ROS params which are retrieved in the
   *  constructor and can be specified by the user at runtime. Check the ROS param list above.
   *
   *  Return value:
   *    + 0, on success.
   *  Main private variables modified:
   *    + last_sample_
   */
  int checkData();

  /*  Initializes the serial communication with the glove in a synchronized read and write mode with baud rate equals
   *  115200 bit/s. The communication is opened on the 'serial_port_' port which is provided as ROS param (retrieved 
   *  in the constructor) that can be specified by the user at runtime. Check the ROS param list above.
   *
   *  Exceptions:
   *    + directly if 'open(...)' function fails.
   *    + directly if 'tcsetattr(...)' function fails.
   *  Main private variables modified:
   *    + serial_port_fd_
   *    + new_port_settings_
   */
  void initializeCommunication();

  /*  Retrieves data from the messages in the subscriber topic queue (when there are available) and fills the log
   *  files with it. Also, calls the 'printGloveIMUArray()' method if 'verbose_mode_' is true.
   *
   *  Parameters:
   *    + msg: points to the last 'glove_acquisition::GloveIMUArray' message of the subscriber topic queue.
   *  Other methods called:
   *    + printGloveIMUArray(const std::vector<glove_acquisition::GloveIMU> &data)
   *  Main private variables modified:
   *    + num_data_retrieved_
   */
  void messageCallback(const glove_acquisition::GloveIMUArray::ConstPtr &msg);

  /*  Simple method to print on screen (using ROS debug console level) the accelerometer and gyroscope values of the
   *  given 'glove_acquisition::GloveIMU' vector ('data'). Usually the whole set of IMUs is furnished to this method.
   *
   *  Parameters:
   *    + data: accelerometer and gyroscope values grouped per IMU in a std::vector structure.
   */
  void printGloveIMUArray(const std::vector<glove_acquisition::GloveIMU> &data);

  /*  Generates a 'glove_acquisition::GloveIMUArray' message filled with the data stored in 'data_accelerometers_' and
   *  'data_gyroscopes_' (previously retrieved from the IMUs of the glove) and publishes it through the 'publisher_'
   *  (on the 'topic_name_' topic provided as ROS param).
   *
   *  Other methods called:
   *    + printGloveIMUArray(const std::vector<glove_acquisition::GloveIMU> &data)
   */
  void publishMessage();

  /*  Reads one line from the serial stream (character by character until '\n' is found). Each char retrieved is
   *  stored in the std::vector<char> 'data_temp_'. This method has to be called twice if the glove is sending both
   *  accelerometers and gyroscopes data (but it should not be used in this demo).
   *
   *  Return value:
   *    + 0, on success.
   *  Main private variables modified:
   *    + data_temp_
   */
  int retrieveData();
};

#endif
