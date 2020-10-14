#include "ros/ros.h"
#include "mobile_robotics_hw_2/Pose2DStamped.h"

#include <random>

using namespace std;

int main(int argc, char **argv)
{
  
  ROS_INFO(" Random Odometry 2D ");

  ros::init(argc, argv, "Random_Odometry_2D");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<mobile_robotics_hw_2::Pose2DStamped>("odometry2D", 1000, true);
  ROS_INFO(" Created publisher on odometry2D... ");
  
  std::random_device rd;
  std::default_random_engine generator( rd() );
  std::normal_distribution<double> goalAngleDistribution(0.,M_PI/200.);
  std::uniform_real_distribution<double> goalAngleSpeedDistribution(M_PI/10000., M_PI/1000.);

  std::uniform_real_distribution<double> goalSpeedGen(0.01, 0.001);
  

  double goalAngle = goalAngleDistribution(generator);
  double goalAngleSpeed = goalAngleSpeedDistribution(generator);
  double goalSpeed = goalSpeedGen(generator);
  
  
  double currentAngle = 0.;
  double currentSpeed = 0.;
  
  ros::Rate loop_rate(100);
  
  while(ros::ok()){
    
    mobile_robotics_hw_2::Pose2DStamped::Ptr posePtr(new mobile_robotics_hw_2::Pose2DStamped());
  
    
    if(goalAngle < 0.){ 
      currentAngle -= goalAngleSpeed; 
      if(currentAngle < goalAngle){
	goalAngle = goalAngleDistribution(generator);
	goalAngleSpeed = goalAngleSpeedDistribution(generator);
	goalSpeed = goalSpeedGen(generator);
      }
    }else{
      currentAngle += goalAngleSpeed;
      if(currentAngle > goalAngle){
	goalAngle = goalAngleDistribution(generator);
	goalAngleSpeed = goalAngleSpeedDistribution(generator);
	goalSpeed = goalSpeedGen(generator);
      }
      
    }
    currentSpeed = (currentSpeed * 0.99) + (goalSpeed * 0.01);
    
    posePtr->header.frame_id = "robot_a";
    posePtr->header.stamp = ros::Time::now();
    
    posePtr->pose2D.x = cos(currentAngle) * currentSpeed;
    posePtr->pose2D.y = sin(currentAngle) * currentSpeed;
    posePtr->pose2D.theta = currentAngle;
  
    cout<<" goal speed "<<goalSpeed<<"  current Speed "<<currentSpeed<<" goalAngle "<< goalAngle/M_PI*180. <<" currentAngle "<< currentAngle/M_PI*180.<<endl;
    
    pub.publish(posePtr);
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  ROS_INFO(" Exiting cleanly! ");

  return 0;
}
