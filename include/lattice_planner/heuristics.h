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

#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <lattice_planner/dynamic_costmap.h>
#include <lattice_planner/state.h>
#include <lattice_planner/motion_constraints.h>
#include <lattice_planner/dijkstra.h>
#include <lattice_planner/cost_manager.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace lattice_planner
{

/**
 * @brief calculates and manages the heuristics (estimated costs from a state
 *        to reach the goal) for path planning.
 *
 * The heuristics are estimated based on the following simplifications of the
 * path planning problem:
 * - no dynamic obstacles
 * - robot is holonomic and has no inertial constraints
 * A planar, eight-connected Dijkstra expansion is conducted on the static layer
 * of the dynamic cost map to find the shortest possible path to the goal and
 * the minimal static environment costs in the abscence of dynamic obstacles. The
 * minimum time to reach the goal is estimated by assuming that the robot can
 * execute the minimum distance path at maximum forward velocity.
 *
 * Furthermore, two optional estimations can be included in the heuristic
 * estimation: The minimum time to decelerate from a state to zero velocity and
 * the minimum time to rotate from a state to the goal orientation. Both properties
 * are only estimated correctly close to the goal, outside the goal region
 * always the worst-case time value is returned. Both estimations result in
 * finding the goal (much) quicker. However, the  heuristic will not be
 * consistent and admissible any more with the estimations, therefore
 * the planning algorithm is no longer guaranteed to find the optimal solution.
 * In practise, the optional estimations will cause the robot to advance straight
 * to the goal and turn to the goal orientation on the spot rather than driving
 * a curve to reach the goal orientation, thus maybe taking a bit longer to
 * reach the goal for some goal configurations.
 */
class Heuristics
{

public:

  /**
   * @brief constructor
   *
   * @param costmap dynamic cost map used for planning
   * @param cost_factors weights of the cost aspects in the defined cost function
   *        @see CostManager
   * @param motion_constraints velocity and acceleration limits of the robot
   */
  Heuristics(DynamicCostmap* costmap, CostManager::CostFactors cost_factors,
             MotionConstraints motion_constraints);

  /**
   * @brief to specify whether or not the optional estimations for the deceleration
   *        to zero velocity and the remaining angle to reach the goal orientation
   *        are included in the heuristic. Violates the optimality criterion of
   *        path planning, but is a lot faster
   *
   * @param estimate_deceleration_at_goal whether the necessary time to decelerate
   *        to zero velocity should be considered in the heuristic
   * @param estimate_rotation_to_goal_orientation whether the necessary time to
   *        rotate to the goal orientation should be considered in the heuristic
   * @param estimation_radius radius around the goal inside which the costs are
   *        estimated correctly. Outside the estimation_radius, always the worst-
   *        case costs are estimated
   */
  void estimateAdditional(bool estimate_deceleration_at_goal,
                          bool estimate_rotation_to_goal_orientation,
                          double estimation_radius);

  /**
   * @brief set neutral cost for Dijkstra expansion
   *
   * @param neutral_cost neutral cost
   */
  void setNeutralCost(float neutral_cost)
  {
    grid_expander_->setNeutralCost(neutral_cost);
  }

  /**
   * @brief specify whether the Dijkstra expansion should explore unknown map
   *        regions
   *
   * @param allow_unknown whether or not to explore unknown map regions
   */
  void setHasUnknown(bool allow_unknown)
  {
    grid_expander_->setAllowUnknown(allow_unknown);
  }


  /**
   * @brief set the lethal cost value - the cost value from which states are pruned
   *
   * @param lethal_cost lethal cost value
   */
  void setLethalCost(float lethal_cost)
  {
    grid_expander_->setLethalCost(lethal_cost);
  }

  /**
   * @brief do the Dijkstra expansion on the map
   *
   * expand from the goal outwards, wavefront style, to find the shortest possible
   * path and minimal environment costs from each state in the map to the goal
   *
   * @param goal_x goal x coordinate in map
   * @param goal_y goal y coordinate in map
   * @return whether or not the expansion was successfull (could fail if goal is
   *         inside obstacle etc.)
   */
  bool expand(int goal_x, int goal_y);

  /**
   * @brief after the heuristic was expanded, it returns the estimated costs from
   *        a state to reach the goal
   * @param state state for which the heuristic is queried
   * @param goal_pose the goal pose
   * @return the estimated remaining costs from the state to the goal
   */
  float getHeuristic(State* state, Pose goal_pose);

  /**
   * @brief estimates the costs according to the time the robot would need to turn
   *        to the goal orientation.
   *
   * Optional heuristic estimation, accelerates the search but might confine the
   * optimality of the path search
   *
   * @param state state for which the heuristic is queried
   * @param goal_pose the goal pose
   * @return additional estimated time costs for turning in place at the goal
   */
  double estimateTurnToGoal(State* state, Pose goal_pose);

  /**
   * @brief estimates the costs according to the time the robot would need to
   *        decelerate to zero velocity at the goal.
   *
   * Optional heuristic estimation, accelerates the search but might confine the
   * optimality of the path search
   *
   * @param state state for which the heuristic is queried
   * @param goal_pose the goal pose
   * @return additional estimated time costs for decelerating to zero at the goal
   */
  double estimateDecelerationAtGoal(State* state, Pose goal_pose);

  /**
   * @brief initialize the heuristics publisher, if desired
   * @param topic_name rostopic name on which to publish
   * @param publish_scale maximum value of the published cost map
   */
  void initPublisher(std::string topic_name, int publish_scale = 100);

  /**
   * @brief publish a heuristics map
   *
   * The heuristic map has to be of the same size than the dynamic cost map and
   * have the same global frame id
   *
   * @param heuristic a heuristics map
   */
  void publishHeuristic(float* heuristic);

  /**
   * @brief publish the heuristic managed by this class
   */
  void publishHeuristic();

private:
  DynamicCostmap* costmap_; ///< dynamic cost map used for planning
  CostManager::CostFactors cost_factors_; ///< cost factors, as used in the defined cost function
  MotionConstraints motion_constraints_; ///< velocity and acceleration limits of the robot
  DijkstraExpansion* grid_expander_; ///< Dijkstra expansion instance
  float* heuristics_; ///< the array to store the heuristics values

  //optional estimations
  bool estimate_deceleration_at_goal_; ///< whether to estimate the time to decelerate at the goal
  bool estimate_rotation_to_goal_; ///< whether to estimate the time to rotate to the goal orientation
  double additional_estimation_radius_; ///< radius in which to estimate optional heuristics correctly
  bool pub_initialized_; ///< whether publisher is initialized
  int publish_scale_; ///< maximum value of the published cost map
  ros::Publisher heuristics_pub_; ///< ROS publisher for the heuristics map

};

}//namespace lattice_planner

#endif
