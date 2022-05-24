#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"


lama::SimpleOccupancyMap *map1;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->pose.pose.position.x);
  map1->setFree(Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
}

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("I heard: [%f]",msg.header.stamp.toSec());
  
  unsigned int width = msg.info.width;
  unsigned int height= msg.info.height;

  for (unsigned int j = 0; j < height; ++j)
  {
    for (unsigned int i = 0; i < width;  ++i)
    {
        Eigen::Vector3d coords;
        coords.x() = msg.info.origin.position.x + i * msg.info.resolution;
        coords.y() = msg.info.origin.position.y + j * msg.info.resolution;

        char value = msg.data[i + j*width];
        if (value == 100) {
            map1->setOccupied(coords);
        }
    }
  }

  lama::sdm::export_to_png(*map1,"map_sub.png");
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  map1  = new lama::SimpleOccupancyMap(0.05);

  ros::Subscriber subPose = n.subscribe("/amcl_pose", 1000, poseCallback);
  ros::Subscriber subMap = n.subscribe("/map", 1000, mapCallback);

  ros::spin();

  lama::sdm::export_to_png(*map1,"germmap.png");


  return 0;
}
