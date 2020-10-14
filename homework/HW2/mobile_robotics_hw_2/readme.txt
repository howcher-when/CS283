Readme for Robotics Homework 2

Create a new ros workspace.
Get the provided files (you obviously did :) )
Compile the homework using catkin_make in the root of your workspace, e.g:
pushd `pwd`; roscd; cd ..; catkin_make; popd

Run the homework:

rosrun mobile_robotics_hw_2 mobile_robotics_hw_2_node

Test the node by publishing an odometry value using rostopic pub:

rostopic pub /odometry2D mobile_robotics_hw_2/Pose2DStamped "header:                                                              
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose2D:
  x: 0.0
  y: 0.0
  theta: 0.0" 

You may listen to the published result using rostopic echo:

rostopic echo /pose2D

------------------------

Play the odometry bagfile - that published odometry on /odometry2D

For that make sure you use the simulated time from the rosbag (and not the current time). So set the according parameter in the parameterserver of ros:

rosparam set use_time_time true

Also play back with the time from the backfile

roscd mobile_robotics_hw_2
rosbag play odometry.bag --clock


Record the published pose:

rosbag record /pose2D 

The resulting bag file will have the current date as name. Rename the good bagfile to poses.bag:

mv 2019_09_xxxxxx.bag poses.bag

Do the same thing for the speed.bag

------------------------

Once you are finished commit your bagfles and code to the hw2 folder in git and push to the gitlab server.
