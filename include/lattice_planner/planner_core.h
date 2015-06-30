/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Marina Kollmitz
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marina Kollmitz
 ******************************************************************************/

#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <lattice_planner/tb_lattice.h>

namespace lattice_planner
{

/**
 * @brief planner core.
 *
 * This class provides the link from move_base to the lattice
 * planner. The class is written as a plugin for the ros navigation stack
 * and inherits from the nav_core::BaseGlobalPlanner class
 * @see http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
 * for more information about how to write a global planner plugin
 */
class TBLatticePlanner : public nav_core::BaseGlobalPlanner
{
public:

  /**
   * @brief default constructor
   */
  TBLatticePlanner();

  /**
   * @brief constructor
   * @param name planner name
   * @param costmap 2D static cost map as provided by the ros navigation stack
   * @param frame_id global frame ID
   */
  TBLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap,
                std::string frame_id);

  /**
   * @brief initialize the global planner
   * @param name planner name
   * @param costmap_ros static cost map as provided by the ros navigation stack
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief generate a plan from a start position to a goal position. Usually
   *        called from move_base
   * @param start start position
   * @param goal goal position
   * @param plan the planned path
   * @return whether a valid path was found
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief generate a plan from a service request
   * @param req service request
   * @param resp service response
   * @return whether a valid path was found
   */
  bool makePlanService(nav_msgs::GetPlan::Request& req,
                       nav_msgs::GetPlan::Response& resp);

  /**
   * @brief publish a plan
   * @param plan plan to publish
   */
  void publishPlan(std::vector<geometry_msgs::PoseStamped> plan);

protected:

  /**
   * @brief initialize the global planner
   * @param name planner name
   * @param costmap static cost map as provided by the ros navigation stack
   * @param frame_id global frame id
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap,
                  std::string frame_id);

  /**
   * @brief remove the cost value from the static cost map in which the robot is in
   * @param costmap map from which to remove the cost value
   * @param mx robot's x position in map coordinates
   * @param my robot's y position in map coordinates
   */
  void clearRobotCell(costmap_2d::Costmap2D *costmap, unsigned int mx,
                      unsigned int my);

  /**
   * @brief outline the map with a specific value
   * @param costmap costmap to outline
   * @param value value with which to outline the map
   */
  void outlineMap(costmap_2d::Costmap2D *costmap, unsigned char value);

  /**
   * @brief service Callback if a replanning request was send from outside the
   *        planning node
   *
   * @param request (empty) replanning request
   * @param response (empty) replanning response
   * @return always true
   */
  bool replanServiceCallback(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response);

  ros::ServiceServer make_plan_srv_; ///< make plan ros service
  ros::ServiceServer replan_service_; ///< replanning ros service
  ros::Publisher path_pub_; ///< ros path publisher

  costmap_2d::Costmap2D* costmap_; ///< 2D static cost map as provided by the ros navigation stack
  AStarLattice* planner_; ///< lattice planner instance
  geometry_msgs::PoseStamped goal_pose_; ///< goal position
  std::string frame_id_; ///< global frame id

  bool replanning_requested_; ///< flag whether a replanning request was send from outside the planning node
  bool initialized_; ///< flag whether the planner is initialized
  boost::mutex mutex_; ///< mutex
};

} //end namespace lattice_planner

#endif
