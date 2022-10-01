#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <explore/costmap_tools.h>

namespace frontier_exploration
{

enum Area {TOP , BOTTOM, LEFT, RIGHT};

/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  double orientation;
  geometry_msgs::Point centroid;
  std::vector<geometry_msgs::Point> points;
  Area relative_area;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch(){}

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale, double gain_scale, double min_frontier_size);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param pose Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::Pose pose);

  FrontierDivision getAreas(geometry_msgs::Pose pose);

protected:

  /**
   * @brief 
   * 
   * @param pos 
   * @param area 
   * @param frontierDiv 
   * @param frontier_flag 
   * @param visited_flag 
   * @param frontier_list 
   */
  void searchFromArea(unsigned int pos, const Area area, const frontier_exploration::FrontierDivision& div, std::vector<Frontier>& frontier_list);
  
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag, const Area area, const std::vector<bool>& area_idx, const frontier_exploration::FrontierDivision& div);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag, const std::vector<bool>& area_idx);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier, const geometry_msgs::Pose& pose);

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
};
}
#endif
