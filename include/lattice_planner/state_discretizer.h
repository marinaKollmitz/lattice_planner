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

#ifndef _STATEDISCRETIZER_H
#define _STATEDISCRETIZER_H

#include <ros/ros.h>
#include <lattice_planner/pose.h>
#include <lattice_planner/velocity.h>
#include <lattice_planner/discrete_state.h>
#include <lattice_planner/motion_constraints.h>
#include <lattice_planner/dynamic_costmap.h>

namespace lattice_planner
{

/**
 * @brief calculations and conversions between a continuous and its discrete state
 *        representation
 */
class StateDiscretizer
{
public:

  /**
   * @brief constructor
   *
   * @param costmap dynamic costmap for planning
   * @param motion_constraints velocity and acceleration limits of the robot
   */
  StateDiscretizer(DynamicCostmap* costmap, MotionConstraints motion_constraints);

  /**
   * @brief calculate the number of discrete angles necessary to represent the
   *        configuration space
   *
   * @param acc_w angular acceleration of the robot
   * @param time_delta time resolution of the configuration space
   * @return number of discrete angles necessary to fully represent the
   *         configuration space
   */
  int getNumDiscreteAngles(double acc_w, double time_delta);

  /**
   * @brief find the discrete representation for a given state
   *
   * @param pose the pose of the state
   * @param vel the state's velocity
   * @param time_step the state's discrete time step
   * @return discretized state representation
   */
  DiscreteState discretizeState(Pose pose, Velocity vel, int time_step);

  /**
   * @brief calculate the map coordinates and map index from a continuous position
   *
   * @param x continuous x coordinate
   * @param y continuous y coordinate
   * @param[out] x_i discrete x coordinate in the map
   * @param[out] y_i discrete y coordinate in the map
   * @param[out] index discrete map index
   * @return whether the queried position is inside the map
   */
  bool getCellPosition(double x, double y, unsigned int &x_i, unsigned int &y_i,
                       unsigned int &index);

  /**
   * @brief get discretized orientation from a given continuous representation
   *
   * @param angle continuous orientation
   * @return discretized orientation
   */
  unsigned int getDiscreteOrientation(double angle);

  /**
   * @brief get discretized velocity from a given continuous representation
   *
   * @param vel continuous velocity
   * @param[out] vel_x_i discretized forward velocity component
   * @param[out] vel_w_i discretized angular velocity component
   */
  void getDiscreteVelocity(Velocity vel, unsigned int &vel_x_i, unsigned int &vel_w_i);

  /**
   * @brief get the discretization limits: amount of discrete steps available
   *        for each state dimension to represent the continuous state space
   *
   * @param[out] num_gridcells number of grid cells in the discrete map
   * @param[out] num_orientations number of discrete orientations
   * @param[out] num_vels_x number of discrete forward velocities
   * @param[out] num_vels_w number of discrete angular velocities
   */
  void getLimits(int &num_gridcells, int &num_orientations, int & num_vels_x,
                 int &num_vels_w);

  /**
   * @brief get corresponding continuous velocity to a given discrete representation.
   *
   *        mostly used to avoid rounding errors when calculating velocities inside
   *        the continuous configuration space
   *
   * @param vel_x_i discrete forward velocity
   * @param vel_w_i discrete angular velocity
   * @return continuous velocity
   */
  Velocity getVelocity(unsigned int vel_x_i, unsigned int vel_w_i);

private:

  /**
   * @brief forbidden constructor
   */
  StateDiscretizer();

  MotionConstraints motion_constraints_; ///< velocity and acceleration limits of the robot
  DynamicCostmap* costmap_; ///< dynamic cost map for planning

  int num_gridcells_; ///< number of grid cells in the discrete map
  int num_orientations_; ///< number of discrete orientations
  int num_vel_x_; ///< number of discrete forward velocities
  int num_vel_phi_; ///< number of discrete angular velocities

  double time_resolution_; ///< continuous time difference between two discrete time steps
  double vel_step_x_; ///< continuous velocity difference between two discrete forward velocities
  double vel_step_phi_; ///< continuous velocity difference between two discrete angular velocities

};
} //namespace lattice_planner

#endif
