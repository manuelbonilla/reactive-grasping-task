# Reactive Grasping Package
This package provides two synchronized nodes which implement a *reactive grasping* task on a real *KUKA LWR 4+* robot endowed with a *Pisa IIT/SoftHand* and a sensorized *5 IMUs Glove*.

![rviz](media/rviz.png)

## Install

1. Follow the instruction in the README.md at CentroEPiaggio@3b74d1878909c8790d4946acbbc5628675ec70fc to properly clone the *vito_robot* package in your catkin workspace.
2. Compile it through `catikin_make` and check if everything works as expected (i.e. execute `roslaunch vito_description display.launch`).
3. Clone the *reactive_grasping* package in your catkin workspace and compile everything (again with `catikin_make`).

## Usage

1. Be sure that the Glove is properly connected to the */dev/ttyACM0* serial port (i.e. the default one). If the port differs, change it in the Glove settings (check *glove_acquisition* package).  
2. *Enable/Disable* the hardware settings in the `reactive_grasping_description/launch/display.launch` file if you *want/do not want* to use the real robot. Please, be sure to enable the Gazebo simulation if the robot is not physically connected.
3. In a terminal, execute `roslaunch reactive_grasping_description display.launch` and wait for the robot to reach the *home pose* (this make take a while in simulation).
4. If you touch the Glove with an object, the robot should perform a grasp primitive to grab it. When the hand is closed, touch it again to let the robot open it and come back to the home pose. Do it as many times as you want (infinite loop until ROS shutdown).

## Info and Warnings

- This package is not standalone: the Centro E. Piaggio *vito_robot* package (commit CentroEPiaggio@3b74d1878909c8790d4946acbbc5628675ec70fc) has to be compiled on your machine.
- The *ReactiveGraspingDetection* class uses linux-specific commands, e.g. `system("mkdir -p ...")`. 
- This code has been developed for ROS Indigo on ubuntu 14.04. No warranty for other distributions.

## ROS Params
The two classes provides several parameters which can be set by the user at runtime:

#### *ReactiveGraspingDetection*

- verbose_mode
- very_verbose_mode
- glove_topic_name
- accel_map_topic_name
- topic_queue_length
- filter_coeff_a
- filter_coeff_b
- num_imus
- gravity_value
- contact_threshold
- window_size
- tails_scale_factor
- delay_threshold
- action_server
- log_file_base_path
- log_file_name_raw
- log_file_name_filt
- log_file_name_map

#### *ReactiveGraspingMotion*

- verbose_mode
- very_verbose_mode
- hand_synergy_joint
- hand_synergy_topic
- joint_state_topic
- topic_queue_length
- action_server
- move_group
- move_group_home
- frame_base
- frame_ee_kuka
- frame_ee_glove
- time_for_grasp
- arm_velocity_threshold
- arm_distance_threshold
- hand_velocity_threshold
- hand_distance_threshold
