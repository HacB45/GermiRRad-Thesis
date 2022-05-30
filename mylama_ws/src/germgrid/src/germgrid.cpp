#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"
#include "lama/sdm/map.h"
#include "lama/pose2d.h"

#include <tf/tf.h>

lama::SimpleOccupancyMap *map1;

/**
 * @brief Indicates the position of the robot (x,y)
 * 
 * @param msg 
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_INFO("Position of the Robot : [%f , %f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
  
  /**
   * 
   * Initial Conditions:
   * Openning Angle -> 90º
   * Central Angle -> 0º (the sensor is turned to the front part of the robot)
   * Distance of irradiation -> 2m
   * The UV sensor is centered on the top of the robot
   * 
   */

  Eigen::VectorVector3ui fan_pos, occ_poses;
  
  lama::Pose2D pose(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

  Eigen::Vector2d point0 = pose * Eigen::Vector2d(2, 2);
  Eigen::Vector2d point1 = pose * Eigen::Vector2d(2,-2);

  map1->computeRay(map1->w2m(Eigen::Vector3d(point0(0), point0(1), 0.0)), 
                   map1->w2m(Eigen::Vector3d(point1(0), point1(1), 0.0)), fan_pos);

  // pos fan

  const size_t num_fan_pos = fan_pos.size();
  for (size_t i = 0; i < num_fan_pos; ++i){
    
    map1->computeRay(map1->w2m(Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0)),fan_pos[i], occ_poses);
  
  }
    

  const size_t num_occ_poses = occ_poses.size();
  for (size_t i = 0; i < num_occ_poses; ++i){
      map1->setFree(occ_poses[i]);
  }

}


/**
 * @brief Creates a representative map of the coordenates already covered by the robot
 * 
 * @param msg 
 */
void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("I heard: [%f]",msg.header.stamp.toSec());
  ROS_INFO("Width: [%u] , Height: [%u], Resolution: [%f]",msg.info.width,msg.info.height,msg.info.resolution);
  ROS_INFO("x: [%f] , y: [%f], z: [%f]",msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z);


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
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "germgrid");
  ros::NodeHandle n;

  map1  = new lama::SimpleOccupancyMap(0.05);

  ros::Subscriber subPose = n.subscribe("/amcl_pose", 1000, poseCallback);
  ros::Subscriber subMap = n.subscribe("/map", 1000, mapCallback);

  ros::spin();

  lama::sdm::export_to_png(*map1,"germmap.png");
  return 0;
}
