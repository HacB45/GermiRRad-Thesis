#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "iostream"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "goalPose");
  ros::NodeHandle n;

  ros::Publisher goalPose_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Rate loop_rate(10);
   
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    double linVelx, angVelz;
    
    n.getParam("/linearVelxUpdate", linVelx);
    n.getParam("/angularVelzUpdate", angVelz);

    msg.linear.x = linVelx;
    msg.angular.z = angVelz;

    ROS_INFO("linear : [x:%f], angular : [z:%f]", msg.linear.x, msg.angular.z);

    goalPose_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}