#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"
#include "lama/sdm/map.h"
#include "lama/pose2d.h"
#include "lama/image.h"
#include <tf/tf.h>
#include <math.h>

lama::SimpleOccupancyMap *map1 = nullptr;
ros::Publisher map_publisher;
double openAngle, dirAngle, rangeUV;

bool OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg)
{
    Eigen::Vector3ui imin, imax;
    map1->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height != msg.data.size() )
        msg.data.resize(width*height);

    lama::Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    lama::SimpleOccupancyMap map2(*map1);
    map1->visit_all_cells([&image, &map2, &imin](const Eigen::Vector3ui& coords){
        Eigen::Vector3ui adj_coords = coords - imin;

        if (map1->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map1->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map1->resolution;

    Eigen::Vector3d pos = map1->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

    return true;
}


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
  if (map1 == nullptr) return;
  
  Eigen::VectorVector3ui fan_pos, occ_poses;
  Eigen::Vector2d point0, point1, point2; 
  double uvAngleRightLimit, uvAngleLeftLimit;

  // Position of the sensor UV
  lama::Pose2D poseSensorUV(0.0 , 0.0 , 0.0);
  // Considered that it will be at the center of the robot
  lama::Pose2D poseRobot(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

  uvAngleLeftLimit = dirAngle + (openAngle/2.0);
  uvAngleRightLimit = dirAngle - (openAngle/2.0);

  // Points of the range of the UV light
  point0 = poseRobot * (poseSensorUV * Eigen::Vector2d(cos(uvAngleLeftLimit)*rangeUV, sin(uvAngleLeftLimit)*rangeUV));
  point1 = poseRobot * (poseSensorUV * Eigen::Vector2d(cos(dirAngle)*rangeUV, sin(dirAngle)*rangeUV));  
  point2 = poseRobot * (poseSensorUV * Eigen::Vector2d(cos(uvAngleRightLimit)*rangeUV, sin(uvAngleRightLimit)*rangeUV));


  fan_pos.push_back(map1->w2m(Eigen::Vector3d(point0(0), point0(1), 0.0)));
  map1->computeRay(Eigen::Vector3d(point0(0), point0(1), 0.0), Eigen::Vector3d(point1(0), point1(1), 0.0), fan_pos); // Este metodo tem sempre em conta as coords laterais e "pinta-as" enquanto que o anterior permite que estas coords estejam na diagonal.
  fan_pos.push_back(map1->w2m(Eigen::Vector3d(point1(0), point1(1), 0.0)));
  map1->computeRay(Eigen::Vector3d(point1(0), point1(1), 0.0), Eigen::Vector3d(point2(0), point2(1), 0.0), fan_pos);
  fan_pos.push_back(map1->w2m(Eigen::Vector3d(point2(0), point2(1), 0.0)));

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

  nav_msgs::OccupancyGrid msg_map;
  OccupancyMsgFromOccupancyMap(msg_map);
  map_publisher.publish(msg_map);

}


/**
 * @brief Creates a representative map of the existing obstacles 
 * 
 * @param msg 
 */
void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  map1  = new lama::SimpleOccupancyMap(0.05);
  
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
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "germgrid");
  ros::NodeHandle n;
  double openAngleDeg, dirAngleDeg;

  n.getParam("/sensorUV_specs/openAngle", openAngleDeg);
  n.getParam("/sensorUV_specs/dirAngle", dirAngleDeg);
  n.getParam("/sensorUV_specs/rangeUV", rangeUV);

  // Transform degrees to rad
  openAngle = (M_PI*openAngleDeg)/180;
  dirAngle = (M_PI*dirAngleDeg)/180;

  ros::Subscriber subMap = n.subscribe("/map", 1000, mapCallback);
  ros::Subscriber subPose = n.subscribe("/amcl_pose", 1000, poseCallback);

  map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map_irradiated", 10, true);

  ros::spin();

  lama::sdm::export_to_png(*map1,"germmap.png");
  return 0;
}
