/*
Modified by: Kehinde Aina
*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "ball_follower/BallDirection.h"
//#include <ball_follower/find_ball_node>

#include "std_msgs/String.h"

using ball_follower::BallDirection;
using namespace std;

int j=0;
string _ballDirection = "";

//get the distance between two points
void setTwist(ros::Publisher _ballDirection_pub)
{
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
  
      //turn right (yaw) and drive forward at the same time
      if(_ballDirection == "Right")
      {
        base_cmd.angular.z = 0.1;
        //base_cmd.linear.x = 0.25;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(_ballDirection=="Left")
      {
        base_cmd.angular.z = -0.1;
        //base_cmd.linear.x = 0.25;
      } 
      else
      { 
        base_cmd.angular.z = 0; 
      }  

    _ballDirection_pub.publish(base_cmd);
  
    ros::Duration(0.5).sleep(); // sleep for half a second

    _ballDirection = "Center";
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    _ballDirection_pub.publish(base_cmd);
    ROS_INFO("drive wheel publish: %d, %d", j, base_cmd.angular.z);
}


void driveRoboCallback(const ball_follower::BallDirectionConstPtr& msg) //ImageConstPtr
{

  BallDirection _bd = *msg;
  
  if(_bd.direction == "Center")
  _ballDirection = "Center";
  else if(_bd.direction == "Right")
  _ballDirection = "Right";
  else if(_bd.direction == "Left")
  _ballDirection = "Left";

  ROS_INFO("Move [%d], %s, %s",j++, _ballDirection.c_str(), _bd.direction.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_wheels");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  
  ros::Rate loop_rate(1);
  
  ros::Subscriber sub = nh.subscribe("/ball_direction", 10, driveRoboCallback);
  
  while (ros::ok())
  {
    setTwist(cmd_vel_pub_);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}


