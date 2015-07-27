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

#include<lattice_planner/cost_manager.h>

namespace lattice_planner
{

CostManager::CostManager(DynamicCostmap* dynamic_costmap,
                         MotionConstraints motion_constaints,
                         CostFactors cost_factors) :
  dynamic_costmap_(dynamic_costmap),
  motion_constraints_(motion_constaints),
  cost_factors_(cost_factors)
{
}

double CostManager::getTrajectoryCost(State *state)
{
  double costs = 0.0;
  State* ancestor = state->ancestor;

  //get costmap indeces traversed by the trajectory
  std::vector<unsigned int> map_indeces = state->trajectory;

  //get the distance between the two states according to the grid representation
  double cell_distance = state->state_i.getDiagonalDistance(ancestor->state_i);

  //get the rotational distance between the two states
  double rot_distance = state->pose.getRotDistance(ancestor->pose);

  int cell_cost = 0;

  //costs due to obstacles on the way
  //one state starts at time ancestor->time and ends at time state->time. A
  //dynamic costmaps stamp marks the start of the interval from one people prediction
  //step to the next. Therefore, we need to look up the dynamic cost at the
  //ancestor's time instead of the state's time.
  for(int i=0; i<map_indeces.size(); i++)
  {
    cell_cost = dynamic_costmap_->getCost(map_indeces.at(i), ancestor->time);

    if(cell_cost >= cost_factors_.lethal_cost)
      return -1; //path is not valid because it hits a lethal obstacle
    costs += cost_factors_.environment_cost * cell_cost;
  }

  //add time cost for each time step
  costs += cost_factors_.time_cost;

  //add path cost dependent on the path length
  costs += cell_distance * cost_factors_.step_cost;

  //add rotation cost
  costs += rot_distance * cost_factors_.rotation_cost;

  return costs;
}

} //namespace lattice_planner
