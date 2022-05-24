#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "iostream"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "goalPose");
  ros::NodeHandle n;

  ros::Publisher goalPose_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Rate loop_rate(10);
  
  float x_lin,y_lin,z_lin;
  float x_ang,y_ang,z_ang;
  std::cout << "Enter the linear x y z:\n";
  std::cin >> x_lin >> y_lin >> z_lin;
  std::cout << "Enter the angular x y z:\n";
  std::cin >> x_ang >> y_ang >> z_ang;
  
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    
    msg.linear.x = x_lin;
    msg.linear.y = y_lin;
    msg.linear.z = z_lin;
    msg.angular.z = x_ang;
    msg.angular.z = y_ang;
    msg.angular.z = z_ang;

    ROS_INFO("linear : [x:%f , y:%f , z:%f]", msg.linear.x, msg.linear.y, msg.linear.z);
    ROS_INFO("angular : [x:%f , y:%f , z:%f]", msg.angular.x, msg.angular.y, msg.angular.z);

    goalPose_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}