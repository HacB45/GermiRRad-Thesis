/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include <explore/frontier_search.h>
#include <costmap_2d/cost_values.h>
#include "lama/pose2d.h"


#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace frontier_exploration
{
  bool nearestCell(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap);
}


namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 1.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers( const std::vector<frontier_exploration::Frontier>& frontiers, const frontier_exploration::FrontierDivision& areas)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA blue2;
  blue2.r = 0;
  blue2.g = 0;
  blue2.b = 1.0;
  blue2.a = 0.2;  
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA red2;
  red2.r = 1.0;
  red2.g = 0;
  red2.b = 0;
  red2.a = 0.2;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::ColorRGBA green2;
  green2.r = 0;
  green2.g = 1.0;
  green2.b = 0;
  green2.a = 0.2;
  std_msgs::ColorRGBA purple;
  purple.r = 128;
  purple.g = 0;
  purple.b = 128;
  purple.a = 1.0;
  std_msgs::ColorRGBA purple2;
  purple2.r = 128;
  purple2.g = 0;
  purple2.b = 128;
  purple2.a = 1.0;
  std_msgs::ColorRGBA olive;
  purple.r = 128;
  purple.g = 128;
  purple.b = 0;
  purple.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;

/** Visualization of the diagonal divisions
 * 
  m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.topRightPoints;
  m.color = purple;
  markers.push_back(m);
  ++id;

  m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.bottomLeftPoints;
  m.color = green;
  markers.push_back(m);
  ++id;

  m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.bottomRightPoints;
  m.color = blue;
  markers.push_back(m);
  ++id;

  m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.topLeftPoints;
  m.color = red;
  markers.push_back(m);
  ++id;
  **/

/** Visualization of the diferent areas
 * 
    m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.topAreaPoints;
  m.color = red2;
  markers.push_back(m);
  ++id;

    m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.rightAreaPoints;
  m.color = purple2;
  markers.push_back(m);
  ++id;

    m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.leftAreaPoints;
  m.color = green2;
  markers.push_back(m);
  ++id;

    m.type = visualization_msgs::Marker::POINTS;
  m.id = int(id);
  m.pose.position = {};
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.points = areas.bottomAreaPoints;
  m.color = blue2;
  markers.push_back(m);
  ++id;
 **/


  for (auto& frontier : frontiers) 
  {
    // TOP AREA
    if (frontier.relative_area == 0)
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      m.color = red;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.centroid;
      m.scale.x = 0.7;
      m.scale.y = 0.7;
      m.scale.z = 0.7;
      m.points = {};
      m.color = red;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.points[frontier.points.size()-1];
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 0.5;
      m.points = {};
      m.color = red;
      markers.push_back(m);
      ++id;
    }
    // LEFT AREA
    else if (frontier.relative_area == 2)
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      m.color = green;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.centroid;
      m.scale.x = 0.7;
      m.scale.y = 0.7;
      m.scale.z = 0.7;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.points[frontier.points.size()-1];
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 0.5;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
    }
    // BOTTOM AREA
    else if (frontier.relative_area == 1)
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      m.color = blue;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.centroid;
      m.scale.x = 0.7;
      m.scale.y = 0.7;
      m.scale.z = 0.7;
      m.points = {};
      m.color = blue;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.points[frontier.points.size()-1];
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 0.5;
      m.points = {};
      m.color = blue;
      markers.push_back(m);
      ++id;
    }
    // RIGHT AREA
    else if (frontier.relative_area == 3)
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      m.color = purple;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.centroid;
      m.scale.x = 0.7;
      m.scale.y = 0.7;
      m.scale.z = 0.7;
      m.points = {};
      m.color = purple;
      markers.push_back(m);
      ++id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.points[frontier.points.size()-1];
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 0.5;
      m.points = {};
      m.color = purple;
      markers.push_back(m);
      ++id;
    }
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  ROS_INFO("POSE -> x : %f | y : %f",pose.position.x,pose.position.y);

  auto areas = search_.getAreas(pose);
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose);

  auto yaw = tf::getYaw(pose.orientation);

  
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_INFO("frontier %zd centroid: x : %f | y : %f", i, frontiers[i].centroid.x,frontiers[i].centroid.y);
  }
  // get costmap
  auto costmap = costmap_client_.getCostmap();

  if (frontiers.empty()) {
    stop();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers, areas);
  }


  std::vector<frontier_exploration::Frontier>::iterator frontier;
  // double fminDist = begin(frontiers)->min_distance;
  // for (auto f = begin(frontiers); f != end(frontiers); ++f)
  // {
  //   if (fminDist >= f->min_distance && !goalOnBlacklist(f->centroid))
  //   {
  //     fminDist = f->min_distance;
  //     frontier = f;
  //   }          
  // }

  frontier = begin(frontiers);
  while (goalOnBlacklist(frontier->centroid) && frontier != end(frontiers))
  {
    frontier++;
  }
  

  ROS_INFO("CENTROID -> x : %f | y : %f",frontier->centroid.x,frontier->centroid.y);
  ROS_DEBUG("CALCULATING THE GOAL...");

  // // frontier centroid coords on map
  // unsigned int fcentroid_mx, fcentroid_my;
  // costmap->worldToMap(frontier->centroid.x, frontier->centroid.y, fcentroid_mx, fcentroid_my);
  // // Index on the associated map coords of frontier centroid
  // unsigned int fcentroidobst, fcentroidpose = costmap->getIndex(fcentroid_mx,fcentroid_my);
  // frontier_exploration::nearestCell(fcentroidobst, fcentroidpose, costmap_2d::LETHAL_OBSTACLE, *costmap);
  // // Associated map coords of obstacle
  // unsigned int fcentroidobst_mx, fcentroidobst_my;
  // costmap->indexToCells(fcentroidobst,fcentroidobst_mx,fcentroidobst_my);
  // // World coords of obstacle
  // double fcentroidobst_wx, fcentroidobst_wy;
  // costmap->mapToWorld(fcentroidobst_mx,fcentroidobst_my,fcentroidobst_wx,fcentroidobst_wy);
  // ROS_INFO("OBSTACLE -> x : %f | y : %f",fcentroidobst_wx,fcentroidobst_wy);
  // // frontier centroid-Obstacle vector (world coords)
  // geometry_msgs::Point fcentroidObstDiff;
  // fcentroidObstDiff.x = frontier->centroid.x - fcentroidobst_wx;
  // fcentroidObstDiff.y = frontier->centroid.y - fcentroidobst_wy;
  // // frontier centroid-Obstacle distance
  // double dist_fcentroidObstDiff;
  // auto fcentroidobst_dx = std::abs(fcentroidObstDiff.x);
  // auto fcentroidobst_dy = std::abs(fcentroidObstDiff.y);
  // dist_fcentroidObstDiff = hypot(fcentroidobst_dx,fcentroidobst_dy);
  // ROS_INFO("frontier centroid - OBST DISTANCE -> %f",dist_fcentroidObstDiff);

  
  // // Distance between FRONTIER CENTROID - FRONTIER CENTROID NEAREST OBSTACLE less 2m
  // geometry_msgs::Point target_position;
  // if (dist_fcentroidObstDiff >= 2.0) 
  // {
  //   target_position.x = (frontier->centroid.x + fcentroidobst_wx)/2;
  //   target_position.y = (frontier->centroid.y + fcentroidobst_wy)/2;
  //   ROS_INFO("TARGET POSITION -> x : %f | y : %f",target_position.x,target_position.y);

  //   // Goal-Obstacle Distance
  //   double dist_GoalObstDiff;
  //   auto goalObst_dx = std::abs(target_position.x - fcentroidobst_wx);
  //   auto goalObst_dy = std::abs(target_position.y - fcentroidobst_wy);
  //   dist_GoalObstDiff = hypot(goalObst_dx,goalObst_dy);

  //   // the yaw needs to be made


  //   /**
  //    * @brief Distance between the GOAL and the FRONTIER CENTROID CLOSEST OBSTACLE less 2m
  //    */
  //   if(dist_GoalObstDiff <= 2.0) 
  //   {
  //     Eigen::VectorVector3ui points;
  //     computeRay( Eigen::Vector3d(fcentroidobst_mx,fcentroidobst_my,0.0),
  //                 Eigen::Vector3d(fcentroid_mx,fcentroid_my,0.0), points);
  //     size_t num_points = points.size();
  //     geometry_msgs::Point line_point;
  //     for (size_t i = 0; i < num_points; i++)
  //     {
  //       auto point_mx = (unsigned int)points[i](0);
  //       auto point_my = (unsigned int)points[i](1);
  //       double point_wx, point_wy;
  //       costmap->mapToWorld(point_mx,point_my,point_wx,point_wy);
  //       auto dwx = fcentroidobst_wx - point_wx;
  //       auto dwy = fcentroidobst_wy - point_wy;

        
  //       if(hypot(dwx,dwy)>2.0) 
  //       {
  //         line_point.x = point_wx;
  //         line_point.y =  point_wy;
  //         yaw = atan2(dwy,dwx);
  //         if (yaw > 0)
  //         {
  //           yaw = yaw - (M_PI/2);
  //         }
  //         else
  //         {
  //           yaw = yaw + (M_PI/2);
  //         }
  //         target_position = line_point;
  //         ROS_INFO("YAW -> %f",yaw);
  //         break;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     yaw = 0;
  //   }
  // }
  // else 
  // {
  //   frontier_blacklist_.push_back(frontier->centroid); 
  // }


  // float maxf = 0.0;

  // for(int p=0; p < frontier->points.size(); p++) {
  //   geometry_msgs::Point fpoint = frontier->points[p];
  //   geometry_msgs::Point fdiff;
  //   fdiff.x = fpoint.x - pose.position.x;
  //   fdiff.y = fpoint.y - pose.position.y;

  //   unsigned int fmx, fmy;

  //   costmap->worldToMap(fpoint.x, fpoint.y, fmx,fmy);
  //   unsigned int obst, pos = costmap->getIndex(fmx,fmy);
  //   frontier_exploration::nearestCell(obst, pos, costmap_2d::LETHAL_OBSTACLE, *costmap);

     unsigned int mx, my;
     double wx, wy;
     costmap->indexToCells(0,mx,my);
     costmap->mapToWorld(mx,my,wx,wy);
     ROS_WARN("INDEX 0 -- WORLD x: %f, y: %f",wx,wy);

  //   geometry_msgs::Point fdiffobst;
  //   fdiffobst.x = fpoint.x - wx;
  //   fdiffobst.y = fpoint.y - wy;

  //   if (hypot(fdiff.x , fdiff.y) + 10*hypot(fdiffobst.x, fdiffobst.y) > maxf) {

  //       auto adjancent =  std::fabs(fdiff.x-fdiffobst.x);
  //       auto opposite = std::fabs(fdiff.y-fdiffobst.y);
  //       yaw = atan(opposite/adjancent);
  //       maxf = hypot(fdiff.x , fdiff.y) + 10*hypot(fdiffobst.x, fdiffobst.y);
  //   }
  //}

  geometry_msgs::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    makePlan(); 
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

void Explore::computeRay(const Eigen::Vector3d& from, const Eigen::Vector3d& to, Eigen::VectorVector3ui& sink)
{
    // NOTICE: This function is borrowed from
    // https://github.com/OctoMap/octomap/blob/devel/octomap/include/octomap/OcTreeBaseImpl.hxx
    // All credits goes to the authors of Octomap.

    if ( from == to ) return;

    // == Initialization phase ==
    Eigen::Vector3d direction = (to - from);
    double length = direction.norm();
    direction.normalize();

    int    step[3];
    double tMax[3];
    double tDelta[3];

    Eigen::Vector3ui current_key;
    double current_key_mx, current_key_my; 
    auto from_mx = (unsigned int)from(1);
    auto from_my = (unsigned int)from(2);
    costmap_client_.getCostmap()->worldToMap(current_key_mx,current_key_my,from_mx,from_my);
    current_key = Eigen::Vector3ui(current_key_mx,current_key_my,0.0);
    
    
    Eigen::Vector3ui end_key;    
    double end_key_mx, end_key_my;
    auto to_mx = (unsigned int)to(1);
    auto to_my = (unsigned int)to(2);
    costmap_client_.getCostmap()->worldToMap(end_key_mx,end_key_my,to_mx,to_my);
    end_key = Eigen::Vector3ui(end_key_mx,end_key_my,0.0);


    Eigen::Vector3d  vb;
    unsigned int vb_wx, vb_wy;
    costmap_client_.getCostmap()->mapToWorld(vb_wx,vb_wy,current_key_mx,current_key_my);

    for(unsigned int i=0; i < 3; ++i) {
        // compute step direction
        if (direction(i) > 0.0) step[i] =  1;
        else if (direction(i) < 0.0)   step[i] = -1;
        else step[i] = 0;

        // compute tMax, tDelta
        if (step[i] != 0) {
            // corner point of voxel (in direction of ray)
            double voxelBorder = vb(i);
            voxelBorder += (float) (step[i] * costmap_client_.getCostmap()->getResolution() * 0.5);

            tMax[i] = ( voxelBorder - from(i) ) / direction(i);
            tDelta[i] = costmap_client_.getCostmap()->getResolution() / std::fabs( direction(i) );
        }
        else {
            tMax[i] =  std::numeric_limits<double>::max( );
            tDelta[i] = std::numeric_limits<double>::max( );
        }
    }

    // == Incremental phase ==
    bool done = false;
    while (!done) {

        unsigned int dim;

        // find minimum tMax:
        if (tMax[0] < tMax[1]){
            if (tMax[0] < tMax[2]) dim = 0;
            else                   dim = 2;
        }
        else {
            if (tMax[1] < tMax[2]) dim = 1;
            else                   dim = 2;
        }

        // advance in direction "dim"
        current_key[dim] += step[dim];
        tMax[dim] += tDelta[dim];

        // reached endpoint, key equv?
        if (current_key == end_key) {
            done = true;
            break;
        }
        else {

            // reached endpoint world coords?
            // dist_from_origin now contains the length of the ray when
            // traveled until the border of the current voxel.
            double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);

            // if this is longer than the expected ray length, we should have
            // already hit the voxel containing the end point with the code
            // above (key_end). However, we did not hit it due to accumulating
            // discretization errors, so this is the point here to stop the
            // ray as we would never reach the voxel key_end
            if (dist_from_origin > length) {
                done = true;
                break;
            }

            else {  // continue to add freespace cells
                sink.push_back(current_key);
            }
        }
    } // end while
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug)) 
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  explore::Explore explore;
  ros::spin();

  return 0;
}
