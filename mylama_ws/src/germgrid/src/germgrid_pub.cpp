#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "germgrid_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped::ConstPtr>("/acml_pose",1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg;
        
        ROS_INFO("It was published:[%f]",);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}