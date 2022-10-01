#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace frontier_exploration
{

struct FrontierDivision 
{
  std::vector<geometry_msgs::Point> topRightPoints;
  std::vector<geometry_msgs::Point> topLeftPoints;
  std::vector<geometry_msgs::Point> bottomRightPoints;
  std::vector<geometry_msgs::Point> bottomLeftPoints;
  std::vector<bool> topArea;
  std::vector<bool> leftArea;
  std::vector<bool> rightArea;
  std::vector<bool> bottomArea;
  std::vector<geometry_msgs::Point> topAreaPoints;
  std::vector<geometry_msgs::Point> leftAreaPoints;
  std::vector<geometry_msgs::Point> rightAreaPoints;
  std::vector<geometry_msgs::Point> bottomAreaPoints;
};

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap);

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap);

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap);

/**
 * @brief Find nearest cell of a specified value on a especified area
 * 
 * @param result 
 * @param start 
 * @param val 
 * @param costmap 
 * @param div 
 * @param area 
 * @return true 
 * @return false 
 */
bool nearestCellonArea(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap, const std::vector<bool>& area_idx);

/**
 * @brief 
 * 
 * @param pose 
 * @param costmap Reference to map data
 * @return FrontierDivision 
 */
FrontierDivision divFrontier(geometry_msgs::Pose pose, const costmap_2d::Costmap2D& costmap);

std::vector<geometry_msgs::Point> getAreaPoints(std::vector<bool> area_idx, const costmap_2d::Costmap2D& costmap);

}
#endif
