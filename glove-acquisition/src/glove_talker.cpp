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

int main(int argc, char **argv) {
  std::cout << LICENSE_INFO << std::flush;

  ros::init(argc, argv, "glove_talker");
  GloveCore *talker;
  try {
    talker = new GloveCore("talker");
  } catch (const GloveCoreException& e) {
    ROS_FATAL_STREAM(e.what() << ": failure in 'constructor' function");
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }

  // sends to Glove board the command to start sending data through serial communication ('d' means default)
  if (talker->communicationControl('d') != 0) {
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }


  while (talker->getStatus()) {
    if (talker->readAndPublish() != 0) {
      continue;  // there is no data available
    }
    ros::spinOnce();
  }

  // sends to Glove board the command to stop sending data ('!')
  if (talker->communicationControl('!') != 0) {
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }

  delete talker;
  return 0;
}
