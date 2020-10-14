//Wan Haochuan 40726018

#include <ros/ros.h>
#include "mobile_robotics_hw_2/Pose2DStamped.h"
#include "mobile_robotics_hw_2/SpeedStamped.h"

ros::Publisher pub;
ros::Publisher pub_1;
double global_x;
double global_y;
double global_theta;
int last_sec;
int last_nsec;

void odomCallback(const mobile_robotics_hw_2::Pose2DStamped::ConstPtr& odom)
{
  //print information from odom
  ROS_INFO_STREAM("Received odometry2D :"<<*odom);
  //definie odom's theta as theta
  double theta = odom->pose2D.theta;
  //definie odom's x as x
  double x = odom->pose2D.x;
  //definie odom's y as y
  double y = odom->pose2D.y;
  mobile_robotics_hw_2::Pose2DStamped::Ptr posePtr(new mobile_robotics_hw_2::Pose2DStamped());
  //definie posePtr's header
  posePtr->header = odom->header;
  //change posePtr's frame_id to "global"
  posePtr->header.frame_id = "global";  
  //initialize at the begin 
  if (posePtr->header.seq == 1)
  {
    //initialize global_x
    global_x = 0;
    //initialize global_y
    global_y = 0;
    //initialize global_theta
    global_theta = 0;
    //initialize global_sec
    last_sec = 0;
    //initialize global_nsec
    last_nsec = 0;
  } else {
    //create speedPtr to public speed
    mobile_robotics_hw_2::SpeedStamped::Ptr speedPtr(new mobile_robotics_hw_2::SpeedStamped());
    //definie posePtr's header
    speedPtr->header = posePtr->header;
    //caluclate the time difference
    int time = (speedPtr->header.stamp.sec - last_sec) * 1000000000 + (speedPtr->header.stamp.nsec - last_nsec);
    //aviod the condition of time difference = 0
    if (time != 0) 
    {
      //calculate the speed
      speedPtr->speed.data = sqrt(x*x + y*y) / time * 1000000000;
      //record the last stamp.sec
      last_sec = speedPtr->header.stamp.sec;
      //record the last stamp.nsec
      last_nsec = speedPtr->header.stamp.nsec;
      //publish the speedPtr
      pub_1.publish(speedPtr);
    }
  }
  //the x for now
  double new_x = global_x + x * cos(global_theta) + y * sin(global_theta);
  //the y for now
  double new_y = global_y - x * sin(global_theta) + y * cos(global_theta);
  //update value of globle theta
  global_theta = global_theta + theta;
  //update value of globle x
  global_x = new_x;
  //update value of globle y
  global_y = new_y;
  //update value of pose2D.x
  posePtr->pose2D.x = global_x;
  //update value of pose2D.y
  posePtr->pose2D.y = global_y;
  //update value of pose2D.theta
  posePtr->pose2D.theta = global_theta;
  //publish the posePtr
  pub.publish(posePtr);
  
}

int main(int argc, char **argv)
{
  
  ROS_INFO(" Mobile Robotics Homework 2 ");
 
  ros::init(argc, argv, "Odometry_Localization_2D");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odometry2D", 1000, odomCallback);
  ROS_INFO(" Subscribed to odometry2D ... ");

  pub = n.advertise<mobile_robotics_hw_2::Pose2DStamped>("pose2D", 1000, true);
  pub_1 = n.advertise<mobile_robotics_hw_2::SpeedStamped>("speed", 1000, true);
  ROS_INFO(" Created publisher on pose2D... ");
  
  ros::spin();

  ROS_INFO(" Exiting cleanly! ");

  return 0;
}
