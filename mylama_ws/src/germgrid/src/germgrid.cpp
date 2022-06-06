#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"
#include "lama/sdm/map.h"
#include "lama/pose2d.h"
#include <tf/tf.h>
#include <math.h>

lama::SimpleOccupancyMap *map1;

/**
 * @brief Compute all the coords on a straight line between two points until it reaches one coord that 
 *        corresponds to an obtacle on the map.
 * 
 * @param from 
 * @param to 
 * @param sink 
 */
void computeIrradiatedRay(const Eigen::Vector3ui& from, const Eigen::Vector3ui& to, Eigen::VectorVector3ui& sink)
{
  if ( from == to ) return;

  Eigen::Vector3l error = Eigen::Vector3l::Zero();
  Eigen::Vector3l coord = from.cast<int64_t>();
  Eigen::Vector3l delta = to.cast<int64_t>() - coord;
  
  Eigen::Vector3l step = (delta.array() < 0).select(-1, Eigen::Vector3l::Ones());

  delta = delta.array().abs();
  int n = delta.maxCoeff() - 1;

  // maximum change of any coordinate
  for (int i = 0; i < n; ++i){
    // update errors
    error += delta;

    for (int j = 0; j < 3; ++j)
    {
        if ( (error(j) << 1) >= n ){
            coord(j) += step(j);
            error(j) -= n;
        }
    }
    
    // check if an obstacle is reached
    if ((map1->isOccupied(Eigen::Vector3ui(coord.cast<uint32_t>()))))
    {
      return;
    }       
    else
    {
      
      // save the coordinate
      sink.push_back(coord.cast<uint32_t>());
    }
  }
}


/**
 * @brief Updates the area on the map irradianted by the UV light
 * 
 * Openning Angle - [pi/6 , pi/2] 
 * Direction Angle - [0 , 2*pi[ 
 * Distance of Irradiation - 
 * UV sensor position - Considered that it will be at the center of the robot
 *          
 * @param msg 
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  Eigen::VectorVector3ui fan_pos, occ_poses;
  Eigen::Vector2d point0, point1; 
  Eigen::Vector2d sensorPos;
  double openAngle = M_PI/2;
  double dirAngle = 0;
  double rangeUV = 2; 
  double uvAngleRightLimit, uvAngleLeftLimit;
  
  // Considered that it will be at the center of the robot
  lama::Pose2D poseRobot(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

  uvAngleLeftLimit = dirAngle + (openAngle/2.0);
  uvAngleRightLimit = dirAngle - (openAngle/2.0);

  ROS_INFO("x -> [%f] , y -> [%f]",msg->pose.pose.position.x,msg->pose.pose.position.y);

  // // Point of the left range of the UV light
  // if (uvAngleLeftLimit >= (2*M_PI))
  // {
  //   uvAngleLeftLimit = uvAngleLeftLimit - (2*M_PI); 
  //   //QUADRANT IV
  //   point0 = poseRobot * Eigen::Vector2d(cos(uvAngleLeftLimit*rangeUV), -sin(uvAngleLeftLimit*rangeUV),);
  // }
  // else if ((dirAngle >= ((2*M_PI)/3)) && (dirAngle < (2*M_PI)))
  // {
  //   //QUADRANT IV
  //   point0 = poseRobot * Eigen::Vector2d(cos(uvAngleLeftLimit*rangeUV), -sin(uvAngleLeftLimit*rangeUV));
  // }
  // else if ((dirAngle >= M_PI) && (dirAngle < (2*M_PI)/3))
  // {
  //   //QUADRANT III
  //   point0 = poseRobot * Eigen::Vector2d(-cos(uvAngleLeftLimit*rangeUV), -sin(uvAngleLeftLimit*rangeUV));
  // }
  // else if ((uvAngleLeftLimit >= M_PI/2) && (uvAngleLeftLimit < M_PI))
  // {
     //QUADRANT II
     //ROS_INFO("LEFT ANGEL -> x: [%f] , y: [%f]",cos(-uvAngleLeftLimit),sin(uvAngleLeftLimit));
    point0 = poseRobot * Eigen::Vector2d(cos(-uvAngleLeftLimit)*rangeUV, sin(uvAngleLeftLimit)*rangeUV);
  // }
  // else 
  // {
  //  // QUADRANT I
    // point0 = poseRobot * Eigen::Vector2d(cos(uvAngleLeftLimit*rangeUV), sin(uvAngleLeftLimit*rangeUV));
  // }  

  
  // // Point of the right range of the UV light
  // if (uvAngleRightLimit < 0)
  // {
    // uvAngleRightLimit = (2*M_PI) + uvAngleRightLimit;
    //QUADRANT I
    point1 = poseRobot * Eigen::Vector2d(cos(uvAngleRightLimit)*rangeUV, sin(uvAngleRightLimit)*rangeUV);  

  // }
  // else if ((dirAngle >= 0) && (dirAngle < M_PI/2))
  // {
  //   //QUADRANT I
  //   point1 = poseRobot * Eigen::Vector2d(cos(uvAngleRightLimit*rangeUV), sin(uvAngleRightLimit*rangeUV));  
  // }
  // else if ((dirAngle >= M_PI/2) && (dirAngle < M_PI))
  // {
  //   //QUADRANT II
  //   point1 = poseRobot * Eigen::Vector2d(-cos(uvAngleRightLimit*rangeUV), sin(uvAngleRightLimit*rangeUV));  
  // }
  // else if ((dirAngle >= M_PI) && (dirAngle < ((2*M_PI)/3)))
  // {
  //   //QUADRANT III
  //   point1 = poseRobot * Eigen::Vector2d(-cos(uvAngleRightLimit*rangeUV), -sin(uvAngleRightLimit*rangeUV));  
  // }
  // else
  // {
  //   //QUADRANT IV
  //   point1 = poseRobot * Eigen::Vector2d(cos(uvAngleRightLimit*rangeUV), -sin(uvAngleRightLimit*rangeUV));  
  // }  

  // ROS_INFO("Point 0 -> x: [%f] , y: [%f]",point0(0),point0(1));
  // ROS_INFO("Point 1 -> x: [%f] , y: [%f]",point1(0),point1(1));
  
  // Compute the points that represent the limit of the irradiantion light
  map1->computeRay(map1->w2m(Eigen::Vector3d(point0(0), point0(1), 0.0)), map1->w2m(Eigen::Vector3d(point1(0), point1(1), 0.0)), fan_pos);


  // Compute all the points that represent the area irradianted
  const size_t num_fan_pos = fan_pos.size();
  for (size_t i = 0; i < num_fan_pos; ++i){
    computeIrradiatedRay(map1->w2m(Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0)),fan_pos[i], occ_poses);
  }
    
  // Update the map with the calculated points
  const size_t num_occ_poses = occ_poses.size();
  for (size_t i = 0; i < num_occ_poses; ++i){
      map1->setFree(occ_poses[i]);
  }

}


/**
 * @brief Creates a representative map of the existing obstacles 
 * 
 * @param msg 
 */
void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  // ROS_INFO("I heard: [%f]",msg.header.stamp.toSec());
  // ROS_INFO("Width: [%u] , Height: [%u], Resolution: [%f]",msg.info.width,msg.info.height,msg.info.resolution);
  // ROS_INFO("x: [%f] , y: [%f], z: [%f]",msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z);


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
