# Reactive Grasping Package
This package provides two synchronized nodes which implement a *reactive grasping* task on a real *KUKA LWR 4+* robot endowed with a *Pisa IIT/SoftHand* and a sensorized *5 IMUs Glove*.

![rviz](media/rviz.png)

## Installation
1. Follow the instruction in the README.md at [CentroEPiaggio/vito-robot@3b74d18](https://github.com/CentroEPiaggio/vito-robot/tree/3b74d1878909c8790d4946acbbc5628675ec70fc) to properly clone the *vito-robot* package in your catkin workspace.

        git clone --recursive git@github.com:CentroEPiaggio/vito-robot.git
        cd vito-robot
        trash kuka-lwr
        git clone http://github.com/manuelbonilla/kuka-lwr.git
        cd kuka-lwr
        git checkout vito_torque_control_devel

        
2. Compile it through `catkin_make` and check if everything works as expected (i.e. execute `roslaunch vito_description display.launch`).
3. Clone the *reactive_grasping* package (use the `--recursive` flag) in your catkin workspace and compile everything (again with `catkin_make`).

        git clone http://github.com/manuelbonilla/reactive-grasping-task.git
        cd rective-grasping-task
        git clone http://github.com/manuelbonilla/glove-acqusition.git

## Simulation usage
1. have the [CentroEPiaggio/vito-robot@3b74d18](https://github.com/CentroEPiaggio/vito-robot/tree/3b74d1878909c8790d4946acbbc5628675ec70fc) and last commit of reactive-grasping-task packages compiled in the catkin workspace;
2. Be sure that the Glove (the only hardware needed) is properly connected to the */dev/ttyACM0* serial port (i.e. the default one). If the port differs, change it in the Glove settings (check [*glove_acquisition*](https://github.com/manuelbonilla/glove-acquisition/tree/2d20483e9ae5e3567afbe426b076ede6963ab48c) package);
3. In a terminal, execute `roslaunch reactive_grasping_description display.launch` and wait for the robot to reach the *home pose* (this may take a while). Set `use_rviz:=false` to speed up the simulation.
4. If you touch the Glove with an object, the simulated robot should perform a grasp primitive to "grab" it. When the hand is closed, touch it again to let the robot open it and come back to the home pose. Do it as many times as you want (infinite loop until ROS shutdown).

## Real KUKA robot usage
1. have the [CentroEPiaggio/vito-robot@3b74d18](https://github.com/CentroEPiaggio/vito-robot/tree/3b74d1878909c8790d4946acbbc5628675ec70fc) and last commit of reactive-grasping-task packages compiled in the catkin workspace;
2. have the PC in the KUKA local network with IP set to 192.168.0.150. Check connection with the right arm: `ping 192.168.0.10`;
3. check the SoftHand ID: use the *qbmove library* in the SoftHand package (`ls /dev | grep USB` to get the current hand port) and set [this value](https://github.com/manuelbonilla/reactive-grasping-task/blob/master/reactive_grasping_description/launch/display.launch#L207) to match it;
4. Be sure that the Glove is properly connected to the */dev/ttyACM0* serial port (i.e. the default one). If the port differs, change it in the [Glove settings](https://github.com/manuelbonilla/glove-acquisition/blob/master/launch/glove_publisher.launch#L9) (check [*glove_acquisition*](https://github.com/manuelbonilla/glove-acquisition/tree/2d20483e9ae5e3567afbe426b076ede6963ab48c) package); 
5. move the robot in a pose near to the [*task home*](https://github.com/manuelbonilla/reactive-grasping-task/blob/master/reactive_grasping_moveit_configuration/config/vito.srdf#L19) using gravity compensation mode;
6. **IMPORTANT:** have a hand on the emergency button from now on;
7. start the [KRL script](https://github.com/CentroEPiaggio/kuka-lwr/blob/b91e1944e3eaa3ac67c4664b4cff1e55c1a237af/lwr_hw/krl/ros_control.src):
  - set KUKA robot in position mode;
  - enter in the script;
  - disable robot brakes;
  - run the code until it waits for communication from the PC;
8. execute the following commands in distinct terminals (leave them open until the end of the task):
  - `roslaunch reactive_grasping_description display.launch use_rviz:=true use_robot_sim:=false load_moveit:=false right_arm_enabled:=true right_hand_enabled:=true`;
  - `roslaunch reactive_grasping_moveit_configuration move_group.launch allow_trajectory_execution:=true fake_execution:=false info:=true debug:=false`;
  - enable the motion planning from *rviz* when the MoveIt library is successfully loaded;
  - [*optional*] check if the KUKA robot and the SoftHand can be moved properly `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`;
  - `roslaunch reactive_grasping task_core.launch`;
9. If you touch the Glove with an object, the KUKA robot should perform a grasp primitive to grab it. When the hand is closed, touch it again to let the robot open it and come back to the home pose. Do it as many times as you want (infinite loop until ROS shutdown).

## Demo usage
The demo can be tested either in the simulated Gazebo scenario or with the real KUKA robot, and it shows the whole sequence of grasp primitives performable by the robot. This can be useful to see if everything work as expected.
To start the demo, it is only necessary to add `use_demo:=true` when launching `display.launch` in both the previous scenarios.

## Calibration
The *comparision_dataset.yaml* can be calibrated by enabling the ROS param *calibration*. Touch the glove several times and interactively choose which acceleration map is is satisfactory (use the MATLAB script *visualize_acceleration_maps.m* to help you visualizing the maps). The chosen acceleration maps has to be searched in the *&#42;_accelerations_map.dat* log file and copied (in the proper position) in the *comparison_dataset.yaml* configuration file. It has to be notice that the robot **won't perform** the grasp primitive while calibrating.

## Info and Warnings
- This package is not standalone: the Centro E. Piaggio *vito_robot* package (commit [CentroEPiaggio/vito-robot@3b74d18](https://github.com/CentroEPiaggio/vito-robot/tree/3b74d1878909c8790d4946acbbc5628675ec70fc)) has to be compiled on your machine.
- The *ReactiveGraspingDetection* class uses linux-specific commands, e.g. `system("mkdir -p ...")`. 
- This code has been developed for ROS Indigo on ubuntu 14.04. No warranty for other distributions.

## ROS Params
The two classes provides several parameters which can be set by the user at runtime:

#### *ReactiveGraspingDetection*
- verbose_mode
- very_verbose_mode
- only_detection
- calibration
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
