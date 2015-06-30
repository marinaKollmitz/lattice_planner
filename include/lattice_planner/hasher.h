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

#ifndef HASHER_H
#define HASHER_H

namespace lattice_planner
{

/**
 * @brief provides methods to generate unique hash values (keys) for
 *        DiscreteState objects
 */
struct Hasher
{
  /**
   * @brief function operator to create a hash value from a DiscreteState object
   *
   * @param index hashed object
   * @return hash value
   */
  std::size_t operator()(const DiscreteState& index) const
  {
    return getHash(index);
  }

  /**
   * @brief setLimits set the limits for the properties of the DiscreteState
   *        objects to generate unique hashes
   *
   * @param num_gridcells total number of grid cells in the map
   * @param num_orientations total number of allowed discrete angles
   * @param num_vels_x total number of allowed discrete forward velocities
   * @param num_vels_w total number of allowed discrete angular velocities
   */
  static void setLimits(int num_gridcells, int num_orientations,
                        int num_vels_x, int num_vels_w)
  {
    num_gridcells_ = num_gridcells;
    num_orientations_ = num_orientations;
    num_vels_x_ = num_vels_x;
    num_vels_w_ = num_vels_w;
  }

  /**
   * @brief generate a (unique) hash value from a DiscreteState object
   *
   * @param index hashed object
   * @return hash value
   */
  static size_t getHash(const DiscreteState& index)
  {
    size_t hash = index.grid_cell
        + index.angle_i * Hasher::num_gridcells_
        + index.vel_x_i * Hasher::num_gridcells_ * Hasher::num_orientations_
        + index.vel_w_i * Hasher::num_gridcells_ * Hasher::num_orientations_
        * Hasher::num_vels_x_;

    return hash;
  }

  static int num_gridcells_; ///< total number of grid cells in the map
  static int num_orientations_; ///< total number of allowed discrete angles
  static int num_vels_x_; ///< total number of allowed discrete forward velocities
  static int num_vels_w_; ///< total number of allowed discrete angular velocities
};

} //namespace lattice_planner

#endif
