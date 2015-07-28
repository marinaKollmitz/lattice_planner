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

#ifndef DYNAMIC_LAYERS_H_
#define DYNAMIC_LAYERS_H_

#include <costmap_2d/costmap_2d_ros.h>

namespace lattice_planner
{

/**
 * @brief The DynamicLayers class defines the plugin interface for the dynamic
 *        layers of the dynamic_costmap used by the lattice_planner.
 */
class DynamicLayers
{
public:
  /**
   * @brief initialize function for the dynamic layers plugin
   * @param static_map the static map
   * @param max_timesteps maximum number of time steps represented by the dynamic layers
   * @param time_resolution time resolution of the dynamic layers
   */
  virtual void initialize(costmap_2d::Costmap2DROS* static_map,
                          unsigned int max_timesteps, ros::Duration time_resolution) = 0;

  /**
   * @brief update the dynamic cost map
   *
   * update is called by the planner right before a new plan is computed. That way
   * the dynamic representation is only updated when needed, not every time a
   * new observation comes in
   */
  virtual void update() = 0;

  /**
   * @brief get the (dynamic) cost at the requested time
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @param time requested time
   * @return dynamic cost according to the defined time dependent constraints
   */
  virtual unsigned int getCost(unsigned int map_ix, unsigned int map_iy, ros::Time time) = 0;

  /**
   * @brief get the (dynamic) cost at the requested time
   * @param map_index map index
   * @param time requested time
   * @return dynamic cost according to the defined time dependent constraints
   */
  virtual unsigned int getCost(unsigned int map_index, ros::Time time) = 0;

protected:
  /**
   * @brief default constructor
   */
  DynamicLayers(){}
};

};

#endif
