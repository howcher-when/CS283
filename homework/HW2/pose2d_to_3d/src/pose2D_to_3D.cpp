#include "ros/ros.h"

#include "mobile_robotics_hw_2/Pose2DStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf/transform_datatypes.h"

#include <random>

using namespace std;


nav_msgs::Path::Ptr g_path_pointer;
ros::Publisher pubPose;
ros::Publisher pubPath;

void poseCallback(const mobile_robotics_hw_2::Pose2DStamped::ConstPtr& pose2D){
  
  geometry_msgs::PoseStamped::Ptr pose3D(new geometry_msgs::PoseStamped());
  
  pose3D->header = pose2D->header;
  pose3D->pose.position.x = pose2D->pose2D.x;
  pose3D->pose.position.y = pose2D->pose2D.y;
  pose3D->pose.position.z = 0.;
  pose3D->pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., pose2D->pose2D.theta);
  
  pubPose.publish(pose3D);
  
  g_path_pointer->poses.push_back(*pose3D);
  g_path_pointer->header = pose3D->header;
  
  pubPath.publish(g_path_pointer);
}

int main(int argc, char **argv)
{
  
  g_path_pointer = nav_msgs::Path::Ptr(new nav_msgs::Path());

  
  ROS_INFO(" Pose2D to 3D and Path ");

  ros::init(argc, argv, "Pose2D_to_3D");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("pose2D", 1000, poseCallback);

  pubPose = n.advertise<geometry_msgs::PoseStamped>("pose3D", 1000, true);
  pubPath = n.advertise<nav_msgs::Path>("path3D", 1000, true);
  
  ROS_INFO(" Created publishers on pose3D and path3D ... ");
  
  
  ros::spin();
  

  ROS_INFO(" Exiting cleanly! ");

  return 0;
}
