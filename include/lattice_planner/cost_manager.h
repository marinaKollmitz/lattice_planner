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

#ifndef _COSTMANAGER_H
#define _COSTMANAGER_H

#include <lattice_planner/dynamic_costmap.h>
#include <lattice_planner/state.h>
#include <lattice_planner/motion_constraints.h>

namespace lattice_planner
{

/**
 * @brief Class for cost calculation for path search
 *
 * Calculates the costs of a path (or path segment) according to a given cost
 * function. The cost function is a weighted sum of different cost aspects:
 *
 * - execution time: cost according to the time it takes to follow the plan.
 * - path length: costs according to the traveled distance along a path.
 * - environmental costs: costs defined in the dynamic environment, obtained
 *                      from a layered dynamic cost map.
 * - necessary turns: costs according to the rotation the robot has to execute to
 *                    follow the path.
 *
 */
class CostManager
{

public:

  /**
   * @brief Container for the cost factors (weights) of the cost components
   */
  struct CostFactors
  {
    double time_cost; ///< cost factor for path execution time
    double step_cost; ///< cost factor for path length
    double environment_cost; ///< cost factor defined in the dynamic environment, according to the dynamic cost map
    double rotation_cost; ///< cost factor for the accumulated turns along a planned path
    int lethal_cost; ///< value from which on a path is pruned
  };

  /**
   * @brief constructor
   *
   * @param dynamic_costmap dynamic cost map with time layers
   * @param motion_constaints velocity and acceleration limits of the robot
   * @param cost_factors weights for the individual cost components of the defined cost function
   */
  CostManager(DynamicCostmap *dynamic_costmap, MotionConstraints motion_constaints,
              CostFactors cost_factors);

  /**
   * @brief calculates the costs produced by following the trajectory to a state
   *        from its ancestor, according to the defined cost function
   *
   * @param state current state
   * @return the cost of the trajectory, according to the defined cost function
   */
  double getTrajectoryCost(State* state);

  /**
   * @brief get cost function weights
   * @return cost function weights
   */
  CostFactors getCostFactors()
  {
    return cost_factors_;
  }

private:

  /**
   * @brief forbidden constructor
   */
  CostManager();

  DynamicCostmap* dynamic_costmap_; ///< dynamic cost map with time layers
  MotionConstraints motion_constraints_; ///< velocity and acceleration limits of the robot
  CostFactors cost_factors_; ///< weights of the cost aspects in the defined cost function
};

} //namespace lattice planner
#endif
