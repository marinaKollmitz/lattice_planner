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

#ifndef _TBLATTICE_H
#define _TBLATTICE_H

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <lattice_planner/pose.h>
#include <lattice_planner/velocity.h>
#include <lattice_planner/discrete_state.h>
#include <lattice_planner/state.h>

#include <lattice_planner/heuristics.h>
#include <lattice_planner/state_discretizer.h>
#include <lattice_planner/trajectory_rollout.h>
#include <lattice_planner/hasher.h>
#include <lattice_planner/Path.h>
#include <lattice_planner/cost_manager.h>
#include <lattice_planner/dynamic_costmap.h>

#define SQRT2 1.414213562
//#define DEBUG

namespace lattice_planner
{

/**
 * @brief This class implements a time-bounded A* lattice planner for time dependent
 *        planning on a layered, dynamic cost map.
 *
 * The time-bounded lattice planner is designed to plan time dependent, dynamically
 * feasible navigation paths for robots with differential drive constraints.
 * Planning is conducted on a layered dynamic cost map which is based on the
 * ROS costmap representation. The lattice planner uses motion primitives as
 * actions for the tree search. The motion primitives each represent constant
 * acceleration, deceleration and constant motion, respectively, within a constant
 * time interval. The A* lattice planner searches for the optimal path according
 * to a defined cost function. A heuristic is used to estimate the remaining
 * costs to reach the goal. The planner uses a planning timeout to limit the time
 * allocated for planning and ensure a constant planning frequency. Once the planning
 * time runs out, the algorithm returns the path to the best expanded state,
 * which ideally is the goal state. If the goal state was not reached during
 * search, it returns the lowest cost path that reached the goal at any orientation
 * and velocity. In case no state reached the goal position, the algorithm chooses
 * the path to the state that would be expanded next during search. Since A* always
 * expands the state that produces the lowest expected costs to reach the goal,
 * the selected path is the most promising to advance to the goal.
 *
 * The planner is described in more detail for the application of time dependent
 * planning in populated environment in the following paper:
 *
 * M. Kollmitz, K. Hsiao, J. Gaa, W. Burgard: 'Time Dependent Planning on a
 * Layered Social Cost Map for Human-Aware Robot Navigation'. 2015 European
 * Conference on Mobile Robotics (ECMR) (to appear)
 *
 */
class AStarLattice
{
public:

  /**
   * @brief constructor
   * @param name planner name
   * @param costmap 2D static cost map as provided by the ros navigation stack
   */
  AStarLattice(std::string name, costmap_2d::Costmap2DROS *costmap);

  /**
   * @brief destructor
   */
  ~AStarLattice();

  /**
   * @brief generate a path from a start pose to a goal pose.
   *
   * This method is called from the planner core once a goal is send to move_base.
   * If the certain navigation goal is set for the first time, a plan has to be
   * planned from the current robot position and zero velocity. During the frequent
   * replanning, the plan has to be generated from a waypoint in the future, since
   * the planning takes a certain amount of time. The 'replanning' parameter
   * specifies if the planning request is meant to generate an initial, new plan
   * or if it is a replanning request. If it is a replanning request, the start
   * waypoint for planning is calculated according to the specified planning timeout
   * (ros parameter move_base/TBPlanner/planning_timeout)
   *
   * @param start start pose of the robot
   * @param goal desired goal pose
   * @param replanning whether an initial, new plan is requested or replanning
   * @param[out] path the planned navigation path
   * @return whether a plan to the goal position could be found
   */
  bool getPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
               bool replanning, std::vector<geometry_msgs::PoseStamped>& path);

private:

  /**
   * @brief reset the planning procedure: delete pointers and empty containers
   *        needed for one planning procedure.
   */
  void reset();

  /**
   * @brief callback for the ROS timer triggered once the allocated planning time
   *        ran out
   *
   * @param event timer event
   */
  void plannerTimeoutCallback(const ros::TimerEvent& event);

  /**
   * @brief find the waypoint from which to start planning if a plan to the
   *        desired goal was already planned right before this request.
   *
   * Since planning takes some time, the start position for planning should
   * not be the current position of the robot, but the position he will be in
   * once the plan is released. If the robot is already following a plan to a
   * given goal and a replanning request comes in, this method obtaines the
   * starting pose for planning according to the previously planned path and the
   * time at which the new plan is released.
   *
   * @param plan_release_time the time at which the new plan is released
   * @param[out] replanning_start_pose the robot pose from which to start planning
   * @param[out] replanning_start_vel the robot velocity from which to start planning
   * @return whether a valid waypoint for the plan_release_time could be found
   */
  bool findReplanningWaypoint(ros::Time plan_release_time,
                              geometry_msgs::PoseStamped& replanning_start_pose,
                              geometry_msgs::Twist& replanning_start_vel);

  /**
   * @brief expands a circle of states at the beginning of the planning procedure
   *        around the start configuration
   *
   * if the ROS parameter 'move_base/TBLattice/easy_turn_at_start' is set, this
   * method expands one circle of states at no cost at the beginning of the
   * planning procedure. The circle is equal to turning in place at no cost at
   * the starting position. This cheat can accelerate the planning, but might violate
   * the optimality of the path planning procedure.
   *
   * @param start start state
   */
  void expandCircleAtStart(State *start);

  /**
   * @brief add a state to the priority queue for planning
   *
   * @param state state to add
   * @return whether the state is valid and inside the costmap
   */
  bool addState(State* state);

  /**
   * @brief pop the first element from the priority queue of states for expansion
   *
   * @return the first element of the priority queue of states
   */
  State* getFirstPrioElement();

  /**
   * @brief expand the A* lattice search tree
   *
   * The search tree is expanded until the a goal state is found within the specified
   * goal tolerance (ROS param move_base/TBLattice/xy_goal_tolerance and
   * move_base/TBLattice/yaw_goal_tolerance) or until the planning time
   * (ROS param move_base/TBLattice/planning_timeout) ran out. If the planning
   * time ran out before the goal configuration (position, orientation and velocity)
   * was reached, the method returns the best state found so far:
   *
   * - the best state expanded that reaches the goal position at any velocity
   * and orientation, if found
   *
   * - the state that would be expanded next during search. Since A* always expands
   * the state that produces the lowest expected costs to reach the goal,
   * the selected path is the most promising to advance to the goal.
   *
   * @param[out] final best state found so far
   * @return whether the planning found a path to the goal configuration or timed
   *         out before
   */
  bool expandLattice(State *&final);

  /**
   * @brief expand one state according to the defined actions (motion primitives)
   *
   * @param node state to expand
   * @param current_best the best state found so far
   * @param is_goal whether the current_best state is the desired goal state
   */
  void expandNode(State* node, State*& current_best, bool& is_goal);

  /**
   * @brief create one new state from the current state with the new velocity
   *        (which defines the action (motion primitive))
   *
   * @param current parent state
   * @param new_vel new state's velocity
   * @return the new generated state
   */
  State* createState(State* current, Velocity new_vel);

  /**
   * @brief retrace the path from a given state along its ancestors to the start
   *        state
   *
   * @param state state from which to retrace the path
   * @return path from the start state to the given state
   */
  lattice_planner::Path retracePath(State *state);

  //planner preferences
  bool allow_unknown_; ///< whether the planner is allowed to expand into unknown map regions
  double xy_goal_tolerance_; ///< the Euclidean goal tolerance distance
  double yaw_goal_tolerance_; ///< the rotational goal tolerance in rad
  double time_resolution_; ///< time resolution of the discretized configuration space
  double map_index_check_time_inkr_; ///< time increment to slice trajectories for collision checking
  int max_timesteps_; ///< maximum number of time steps of the discretized configuration space
  double planning_timeout_; ///< timeout for the planning after which the best solution found so far is returned
  bool passive_navigation_; ///< flag for planning dynamically for the max number of time steps, not only until the goal is found
  bool publish_expanded_; ///< flag for publishing the expanded search tree for visualization

  //flags for easing path finding - violate optimality but necessary for quick
  //calculation
  bool easy_deceleration_; ///< flag for simplified planning of deceleration at the goal
  bool easy_turn_at_start_; ///< flag for simplified planning of turn in place at the start
  bool easy_turn_at_goal_; ///< flag for simplified planning of turn in place at the goal

  //properties of current path finding run
  Pose start_pose_; ///< start pose for planning
  Pose goal_pose_; ///< desired goal pose
  ros::Time path_time_; ///< time at which the path is released (after planning_timeout)
  lattice_planner::Path current_plan_; ///< last planned path
  nav_msgs::Path expanded_paths_; ///< paths expanded during search
  bool planning_timed_out_; ///< whether the planning is timed out

  //auxiliary classes
  DynamicCostmap* dynamic_costmap_; ///< dynamic costmap instance
  StateDiscretizer* discretizer_; ///< state discretizer instance
  TrajectoryRollout* trajectory_rollout_; ///< trajectory rollout instance
  Heuristics* heuristic_calc_; ///< heuristics calculator
  CostManager* cost_calc_; ///< cost calculater

  //containers for expanded states
  std::vector<State*> queue_; ///<priority queue for expanded states
  std::vector< std::unordered_map<DiscreteState, State*,Hasher>* > expanded_states_; ///< hash map for expanded states

  //ros stuff
  ros::NodeHandle nh_; ///< ROS node handle
  ros::Publisher expanded_paths_pub_; ///< publisher for the search tree
  ros::Publisher vel_path_pub_; ///< publisher for the planned path with planned velocities

  //debug things
  ros::Publisher current_node_pub_; ///< debug publisher for the expanding parent state
  ros::Publisher new_node_pub_; ///< debug publisher for the expanded child state
  ros::Publisher all_expanded_pub_; ///< debug publisher for all already expanded states
  bool debug_no_interrupt_; ///< flag to stop debugging step by step
};

} //namespace lattice_planner
#endif
