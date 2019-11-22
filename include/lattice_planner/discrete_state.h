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

#ifndef _DISCRETESTATE_H
#define _DISCRETESTATE_H

namespace lattice_planner
{

/**
 * @brief Data container for the discrete representation of a planning state
 */
struct DiscreteState
{

  /**
   * @brief constructor
   *
   * @param grid_cell state's corresponding map index
   * @param grid_x state's x coordinate in map
   * @param grid_y state's y coordinate in map
   * @param angle_i discrete angle
   * @param time_step discrete time step
   * @param vel_x_i discrete forward velocity
   * @param vel_w_i discrete angular velocity
   * @param in_map whether the state is inside the map
   */
  DiscreteState(int grid_cell, int grid_x, int grid_y, int angle_i, int time_step,
                int vel_x_i, int vel_w_i, bool in_map) :
    grid_cell(grid_cell),
    x_i(grid_x),
    y_i(grid_y),
    angle_i(angle_i),
    time_step(time_step),
    vel_x_i(vel_x_i),
    vel_w_i(vel_w_i),
    in_map(in_map)
  {
  }

  /**
   * @brief default constructor
   */
  DiscreteState() :
    grid_cell(0),
    x_i(0),
    y_i(0),
    angle_i(0),
    time_step(0),
    vel_x_i(0),
    vel_w_i(0),
    in_map(false)
  {
  }

  /**
   * @brief equal to operator for comparing discrete states. Two states are equal
   *        if they share the same grid cell and velocity
   *
   * @param other state to compare
   * @return whether the state is equal to 'other'
   */
  bool operator==(DiscreteState const& other) const
  {
    return grid_cell == other.grid_cell
        && angle_i == other.angle_i
        && vel_x_i == other.vel_x_i
        && vel_w_i == other.vel_w_i;
  }

  /**
   * @brief calculates the manhattan distance to another state. The manhattan
   *        distance is the sum of the absolute differences of both states'
   *        map coordinates and thus the cell distance if only straight motions
   *        are allowed.
   *
   * @param other state to which the manhattan distance is calculated
   * @return the manhattan distance between the state and 'other'
   */
  double getManhattanDistance(DiscreteState other)
  {
    int num_x = abs(int(x_i - other.x_i));
    int num_y = abs(int(y_i - other.y_i));

    return num_x + num_y;
  }

  /**
   * @brief calculates the diagonal distance to another state. The diagonal
   *        distance is comparable to the manhattan distance but includes straight
   *        and diagonal motions in the grid
   *
   * @param other state to which the diagonal distance is calculated
   * @return the diagonal distance between the state and 'other'
   */
  double getDiagonalDistance(DiscreteState other)
  {
    int num_x = abs(int(x_i - other.x_i));
    int num_y = abs(int(y_i - other.y_i));
    int max = std::max(num_x, num_y);
    int min = std::min(num_x, num_y);

    return max - min + sqrt(2) * min;
  }

  unsigned int grid_cell; ///< corresponding map index of a state
  unsigned int x_i; ///< x map coordinate
  unsigned int y_i; ///< y map coordinate
  unsigned int angle_i; ///< discrete angle
  unsigned int time_step; ///< discrete time step
  unsigned int vel_x_i; ///< discrete forward velocity
  unsigned int vel_w_i; ///< discrete angular velocity
  bool in_map; ///< whether the state is inside the map
};

}//namespace lattice_planner

#endif
