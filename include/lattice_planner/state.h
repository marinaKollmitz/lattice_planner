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

#ifndef _STATE_H
#define _STATE_H

#include<lattice_planner/pose.h>
#include<lattice_planner/velocity.h>
#include<lattice_planner/discrete_state.h>

namespace lattice_planner
{

/**
 * @brief data container for the properties of a planning state in the
 *        configuration space and methods for state-related calculations
 *        and conversions
 */
struct State
{

  /**
   * @brief container for the ordering of priority queues from states
   */
  struct greater
  {
    /**
     * @brief function operator to sort State objects according to their
     *        total cost value
     *
     * @param state_one first state
     * @param state_two second state
     * @return whether the total cost of state_one is bigger than the total cost
     *         of state_two
     */
    bool operator()(State* state_one, State* state_two)
    {
      return state_one->getTotalCost() > state_two->getTotalCost();
    }
  };

  /**
   * @brief constructor
   *
   * @param id state ID
   * @param state_i discrete state representation of the state
   * @param pose the pose of the state
   * @param velocity the velocity of the state
   * @param time the state's time
   * @param ancestor the state's parent
   */
  State(int id, DiscreteState state_i, Pose pose, Velocity velocity, ros::Time time,
        State* ancestor) :
    id(id),
    state_i(state_i),
    pose(pose),
    vel(velocity),
    ancestor(ancestor),
    way_cost(0.0),
    cost_to_goal(0.0),
    in_goal_tolerance(false),
    time(time)
  {
  }

  /**
   * @brief set the cost values for the state
   *
   * @param way_cost the (known) cost from the start to the state along the planned
   *        path
   * @param cost_to_goal the estimated remaining cost from the state to reach the
   *        goal, according to a defined heuristic
   */
  void setCosts(double way_cost, double cost_to_goal)
  {
    this->way_cost = way_cost;
    this->cost_to_goal = cost_to_goal;
  }

  /**
   * @brief get the total cost of the state
   *
   * The total cost of a state is is made up off the way cost from the start to
   * the state along the planned path and the estimated cost to reach the goal,
   * according to a defined heuristic
   *
   * @return the total cost of the state
   */
  double getTotalCost()
  {
    return way_cost + cost_to_goal;
  }

  /**
   * @brief obtain a string description of a state, including its properties.
   *
   * @return string description of a state
   */
  std::string getDescription()
  {
    std::stringstream ss;
    ss << getTotalCost() << "\t" << id << "\t" << state_i.grid_cell << "\t" <<
          state_i.angle_i << "\t" << vel.vel_x << "\t" << vel.vel_phi << "\t" <<
          state_i.time_step << "\t" << way_cost << "\t" << cost_to_goal << std::endl;
    std::string description = ss.str();
    return description;
  }

  int id; ///< state ID
  DiscreteState state_i; ///< discrete state representation of the state
  Pose pose; ///< pose of the state
  Velocity vel; ///< velocity of the state
  ros::Time time; ///< the state's time
  std::vector<unsigned int> trajectory; ///< cells touched by the traversed path to the state from its ancestor
  double way_cost; ///< costs from the start to the state
  double cost_to_goal; ///< estimated costs from a state to reach the goal
  bool in_goal_tolerance; ///< flag whether state is within the defined goal tolerance for planning
  State* ancestor; ///< the state's parent
};

}//namespace lattice_planner

#endif
