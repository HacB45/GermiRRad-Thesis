#include <explore/frontier_search.h>
#include <mutex>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale, double gain_scale, double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size) {

}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Pose pose)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(pose.position.x, pose.position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  frontier_exploration::FrontierDivision frontierDiv = divFrontier(pose,*costmap_);

  unsigned int pos = costmap_->getIndex(mx, my);

  // TOP AREA
  searchFromArea(pos, TOP, frontierDiv, frontier_list);
  // LEFT AREA
  searchFromArea(pos, LEFT, frontierDiv, frontier_list);
  // BOTTOM AREA
  searchFromArea(pos, BOTTOM, frontierDiv, frontier_list);
  // Right Area
  searchFromArea(pos, RIGHT, frontierDiv, frontier_list);

  // set costs of frontiers
  for (auto& frontier : frontier_list) 
  {
    frontier.cost = frontierCost(frontier, pose);
  }
  std::sort(frontier_list.begin(), frontier_list.end(), [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

void FrontierSearch::searchFromArea(unsigned int pos, const Area area, const frontier_exploration::FrontierDivision& div, std::vector<Frontier>& frontier_list)
{

  map_ = costmap_->getCharMap();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;


  std::vector<bool> area_idx;
  switch (area)
  {
  case TOP:
    area_idx = div.topArea;
    break;
  case BOTTOM:
    area_idx = div.bottomArea; 
    break;
  case LEFT:
    area_idx = div.leftArea;
    break;
  case RIGHT:
    area_idx = div.rightArea;
    break;
  default:
    ROS_WARN("This area doesn't exist !!!");
    break;
  }
  
  // find closest clear cell to start search
  unsigned int clear;
  if (nearestCellonArea(clear, pos, FREE_SPACE, *costmap_, area_idx)) 
  {
    bfs.push(clear);
  } 
  else 
  {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }

  visited_flag[bfs.front()] = true;


  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) 
    {
      if (area_idx[nbr])
      {
        // add to queue all free, unvisited cells, use descending search in case initialized on+ non-free cell
        if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) 
        {
          visited_flag[nbr] = true;
          bfs.push(nbr);
        }

        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
        else if (isNewFrontierCell(nbr, frontier_flag, area_idx)) 
        {
          frontier_flag[nbr] = true;
          Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, area, area_idx, div);

          if (new_frontier.size * costmap_->getResolution()  >= min_frontier_size_) 
          {
            frontier_list.push_back(new_frontier);
          }
        }
      }
    }
  }
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag, const Area area, const std::vector<bool>& area_idx, const frontier_exploration::FrontierDivision& div)
{
  
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();
  output.relative_area = area;


  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference pose.position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) 
  {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) 
    {
      if (area_idx[nbr])
      {
        // check if neighbour is a potential frontier cell
        if (isNewFrontierCell(nbr, frontier_flag, area_idx)) 
        {
          // mark cell as frontier
          frontier_flag[nbr] = true;
          unsigned int mx, my;
          double wx, wy;
          costmap_->indexToCells(nbr, mx, my);
          costmap_->mapToWorld(mx, my, wx, wy);

          geometry_msgs::Point point;
          point.x = wx;
          point.y = wy;
          output.points.push_back(point);

          // update frontier size
          output.size++;

          // update centroid of frontier
          output.centroid.x += wx;
          output.centroid.y += wy;

          // determine frontier's distance from robot, going by closest gridcell
          // to robot
          double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                pow((double(reference_y) - double(wy)), 2.0));
          if (distance < output.min_distance) {
            output.min_distance = distance;
          }
          
          // add to queue for breadth first search
          bfs.push(nbr);
        }
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag, const std::vector<bool>& area_idx)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE && area_idx[nbr]) {
      return true;
    }
  }
  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier, const geometry_msgs::Pose& pose)
{
  auto yaw = pose.orientation.z;
  Area area_flag;
  double orientation;

  if (yaw > (-M_PI/4) && yaw <= (M_PI/4))
  {
    area_flag = RIGHT;
  }
  else if (yaw > (M_PI/4) && yaw <= (3*M_PI/4))
  {
    area_flag = TOP;
  }
  else if (yaw <= (-M_PI/4) && yaw > -(3*M_PI/4))
  {
    area_flag = BOTTOM;
  }
  else if ( M_PI >= yaw > (3*M_PI/4) || -M_PI <= yaw <= -(3*M_PI/4))
  {
    area_flag = LEFT;
  }
  
  if (frontier.relative_area == area_flag)
  {
    orientation = 1.0;
  }
  else
  {
    orientation = 5.0;
  }
  
  return (orientation * costmap_->getResolution())- (frontier.min_distance * costmap_->getResolution());
}

FrontierDivision FrontierSearch::getAreas(geometry_msgs::Pose pose)
{
  return divFrontier(pose,*costmap_);
}

}
