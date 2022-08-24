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
#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/GetPlan.h>

//#include <explore/costmap_tools.h>

namespace frontier_exploration {
  bool nearestCell(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap);
  std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap);
}

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one, const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
  Explore::Explore() : private_nh_("~"), tf_listener_(ros::Duration(10.0)), costmap_client_(private_nh_, relative_nh_, &tf_listener_)
    , move_base_client_("move_base"), prev_distance_(0), last_markers_count_(0)
  {
    double timeout;
    double min_frontier_size;
    private_nh_.param("planner_frequency", planner_frequency_, 1.0);
    private_nh_.param("progress_timeout", timeout, 30.0);
    progress_timeout_ = ros::Duration(timeout);
    private_nh_.param("visualize", visualize_, false);
    private_nh_.param("potential_scale", potential_scale_, 1e-3);
    private_nh_.param("orientation_scale", orientation_scale_, 0.0);
    private_nh_.param("gain_scale", gain_scale_, 1.0);
    private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

    search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(), potential_scale_, gain_scale_, min_frontier_size);

    if (visualize_) {
      marker_array_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
    }

    ROS_INFO("Waiting to connect to move_base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");

    ROS_INFO("COSTMAP RESOLUTION : %f ",costmap_client_.getCostmap()->getResolution());

    exploring_timer_ = relative_nh_.createTimer(ros::Duration(1. / planner_frequency_), [this](const ros::TimerEvent&) { makePlan(); });
  }

  Explore::~Explore()
  {
    stop();
  }

  void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
  {
    std_msgs::ColorRGBA blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    std_msgs::ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    std_msgs::ColorRGBA green;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

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
    for (auto& frontier : frontiers) {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      if (goalOnBlacklist(frontier.centroid)) {
        m.color = red;
      } else {
        m.color = blue;
      }
      markers.push_back(m);
      ++id;
    
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.middle;
      // scale frontier according to its cost (costier frontiers will be smaller)
      double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.points = {};
      m.color = red;
      markers.push_back(m);
      ++id;

      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.centroid;
      // scale frontier according to its cost (costier frontiers will be smaller)
      scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
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
    ROS_INFO("ROBOT POSE -> (x,y) : (%f,%f)",pose.position.x,pose.position.y);
    
    // get frontiers sorted according to cost
    auto frontiers = search_.searchFrom(pose.position);
    auto costmap = costmap_client_.getCostmap();

    if (frontiers.empty()) {
      stop();
      return;
    }

    // publish frontiers as visualization markers
    if (visualize_) {
      visualizeFrontiers(frontiers);
    }

    // find non blacklisted frontier
    auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(), [this](const frontier_exploration::Frontier& f) {
                                                                            return goalOnBlacklist(f.centroid);
                                                                          });
    


    geometry_msgs::Point target_position;
    double angle;
    

    float far_fpoint = 0.0;
    
    for(int p=0; p < frontier->points.size(); p++) {

      // Frontier Point
      geometry_msgs::Point fpoint = frontier->points[p];

      // FrontierPoint-RobotPose vector (world coords)
      geometry_msgs::Point fdiff;
      fdiff.x = fpoint.x - pose.position.x;
      fdiff.y = fpoint.y - pose.position.y;

      // frontier point coords on map
      unsigned int fmx, fmy;
      costmap->worldToMap(fpoint.x, fpoint.y, fmx,fmy);
      // Index on the associated map coords of frontier point
      unsigned int fobst, fpose = costmap->getIndex(fmx,fmy);
      frontier_exploration::nearestCell(fobst, fpose, costmap_2d::LETHAL_OBSTACLE, *costmap);
      // Associated map coords of obstacle
      unsigned int fobst_mx, fobst_my;
      costmap->indexToCells(fobst,fobst_mx,fobst_my);
      // World coords of obstacle
      double fobst_wx, fobst_wy;
      costmap->mapToWorld(fobst_mx,fobst_my,fobst_wx,fobst_wy);
      // FrontierPoint-Obstacle vector (world coords)
      geometry_msgs::Point fdiffobst;
      fdiffobst.x = fpoint.x - fobst_wx;
      fdiffobst.y = fpoint.y - fobst_wy; 


      // Sum "Robot-FrontierPoint" distance with "FrontierPoint-Obstacle"
      if (hypot(fdiff.x , fdiff.y) + 10*hypot(fdiffobst.x, fdiffobst.y) > far_fpoint) {
          target_position = fpoint;
          far_fpoint = hypot(fdiff.x , fdiff.y) + 10*hypot(fdiffobst.x, fdiffobst.y);
          //ROS_DEBUG("maxf %f target %f %f", maxf, target_position.x, target_position.y);
      }
    }


    // Goal coords on map
    unsigned int goal_mx, goal_my;
    costmap->worldToMap(target_position.x, target_position.y, goal_mx, goal_my);
    // Index on the associated map coords of Goal
    unsigned int goalobst, goalpose = costmap->getIndex(goal_mx,goal_my);
    frontier_exploration::nearestCell(goalobst, goalpose, costmap_2d::LETHAL_OBSTACLE, *costmap);
    // Associated map coords of obstacle
    unsigned int goalobst_mx, goalobst_my;
    costmap->indexToCells(goalobst,goalobst_mx,goalobst_my);
    // World coords of obstacle
    double goalobst_wx, goalobst_wy;
    costmap->mapToWorld(goalobst_mx,goalobst_my,goalobst_wx,goalobst_wy);
    // Goal-Obstacle vector (world coords)
    geometry_msgs::Point goaldiffobst;
    goaldiffobst.x = target_position.x - goalobst_wx;
    goaldiffobst.y = target_position.y - goalobst_wy;
    // Goal-Obstacle distance
    double dist_goaldiffobst;
    auto goalobst_dx = std::abs(goaldiffobst.x);
    auto goalobst_dy = std::abs(goaldiffobst.y);
    dist_goaldiffobst = hypot(goalobst_dx,goalobst_dy);
    ROS_INFO("GOAL-OBST DISTANCE -> %f",dist_goaldiffobst);


    // If the robot is between 
    if ((1.0 < dist_goaldiffobst) && (dist_goaldiffobst < 2.5))
    {
      // Robto will be turned to the obstacle
      angle = (atan2(goalobst_dy,goalobst_dx));
      ROS_INFO("ROBOT ANGLE -> %f",angle);
    }else {
      // Use goal neighbors bellonging to the frontier

      // Prependicular à tangente fronteira
      angle = 0.0;
    }
    
    

    // time out if we are not making any progress
    bool same_goal = (prev_goal_ == target_position);
    prev_goal_ = target_position;

    if (!same_goal || prev_distance_ > frontier->min_distance) {
      // we have different goal or we made some progress
      last_progress_ = ros::Time::now();
      prev_distance_ = frontier->min_distance;
    }

    // black list if we've made no progress for a long time
    if (ros::Time::now() - last_progress_ > progress_timeout_) {
      frontier_blacklist_.push_back(target_position);
      ROS_DEBUG("Adding current goal to black list");
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
    goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

    move_base_client_.sendGoal(goal, [this, target_position]( const actionlib::SimpleClientGoalState& status,
                                                              const move_base_msgs::MoveBaseResultConstPtr& result) {
          reachedGoal(status, result, target_position);
        });

    // goal.target_pose.pose.position = target_position;   
    // goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    // goal.target_pose.header.stamp = ros::Time::now();
    // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle+M_PI);

    // move_base_client_.sendGoal(goal, [this, target_position]( const actionlib::SimpleClientGoalState& status,
    //                                                           const move_base_msgs::MoveBaseResultConstPtr& result) {
    //       reachedGoal(status, result, target_position);
    //     });
  }

  bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
  {
    constexpr static size_t tolerace = 5;
    costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

    // check if a goal is on the blacklist for goals that we're pursuing
    for (auto& frontier_goal : frontier_blacklist_) {
      double x_diff = fabs(goal.x - frontier_goal.x);
      double y_diff = fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerace * costmap2d->getResolution() && y_diff < tolerace * costmap2d->getResolution())
        return true;
    }
    return false;
  }

  void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status, const move_base_msgs::MoveBaseResultConstPtr& result,
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
    oneshot_ = relative_nh_.createTimer( ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); }, true);
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

}  // namespace explore


int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
   ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
