# RNMSS19

## How to run:

1.  set up the camera and both robots, mount a checkerboard on the UR3
2.  run the hand eye calibration 
    1.  `roslaunch  hand_eye_calibration hand_eye_auto_requirements.launch`
    2.  `rosrun  hand_eye_calibration hand_eye_calibration_auto.py`
    3.  close both terminals, the hand eye matrix should be saved in the data folder
    3.  don't move the camera or the bases of the robots from now on
3.  start the head tracking
    1. `roslaunch tracking tracking.launch`
    2. open rviz, check if the head is inside the field of view
    3. if necessary, change the thresholds (`rosparam set pcl_threshold_... value`)
    4. in the tracking terminal: type in "start" and hit enter
4.  start the robot driver
    1. `roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=134.28.45.59`
5.  start nodes for direct kinematics
    1. `rosrun forward_kinematics transformation_server.py`
6. start nodes for inverse kinematics
    1. `rosrun forward_kinematics inverse_server.py`
    2. `rosrun forward_kinematics call_inverse_and_move.py`
7. start the translation node, **warning: this will make the robot move**
    1. stand near an emergency stop button
    2. `rosrun camera_to_robot_space camera_to_robot_space.py `
