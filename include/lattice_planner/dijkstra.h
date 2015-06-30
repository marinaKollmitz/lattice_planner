/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

// inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++]=n; pending_[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextEnd_<PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++]=n; pending_[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overEnd_<PRIORITYBUFSIZE){    overBuffer_[   overEnd_++]=n; pending_[n]=true; }}
// potential defs
#define POT_HIGH 1.0e10        // unassigned cell potential

namespace lattice_planner
{

/**
 * @brief Class for conducting wavefront style Dijkstra expansions inside a cost
 *        map.
 *
 * The algorithm finds the cheapest path (according to a step cost and an 2D cost
 * map) from each point inside the map to a given goal location. The class is based
 * on the DijkstraExpansion class inside the global_planner package of the ros
 * navigation stack. Their DijkstraExpansion class was modified to enable eight-
 * connected expansion on the grid and expand from one point outwards along the
 * whole map, instead of expanding from a start state to a goal state.
 */
class DijkstraExpansion
{
public:
  DijkstraExpansion(int nx, int ny);

  bool expandToGoal(unsigned char* costs, unsigned int start_x, unsigned int start_y,
                    unsigned int goal_x, unsigned int goal_y, int max_cycles,
                    float* potential);

  bool expand(unsigned char* costs, unsigned int start_x, unsigned int start_y,
              int max_cycles, float* potential);

  void setSize(int nx, int ny);

  void setNeutralCost(float neutral_cost)
  {
    neutral_cost_ = neutral_cost;
    priorityIncrement_ = 2 * neutral_cost_;
  }

  void setLethalCost(float lethal_cost)
  {
    lethal_cost_ = lethal_cost;
  }

  void setFactor(float factor)
  {
    factor_ = factor;
  }

  void setAllowUnknown(bool allow_unknown)
  {
    allow_unknown_ = allow_unknown;
  }

  int toIndex(unsigned int mx, unsigned int my)
  {
    return my * nx_ + mx;
  }

  float calculatePotential(float* potential, float cost, int n);

private:

  bool expand(unsigned char *costs, unsigned int start_x, unsigned int start_y,
              float *potential, int max_cycles, bool has_goal,
              unsigned int goal_x = 0, unsigned int goal_y = 0);

  void updateCell(unsigned char* costs, float* potential, int n);

  void updateCellEightConnect(unsigned char* costs, float* potential, int n);

  float getCost(unsigned char* costs, int n) {
    float c = costs[n];
    if (c < lethal_cost_ - 1 || (allow_unknown_ && c==255)) {
      c = c * factor_ + neutral_cost_;
      if (c >= lethal_cost_)
        c = lethal_cost_ - 1;
      return c;
    }
    return lethal_cost_;
  }

  float getDiagCost(unsigned char* cost, int n, float step_factor)
  {
    float c = cost[n];
    if(c < lethal_cost_ - 1 || (allow_unknown_ && c==255))
    {
      c = c + step_factor * neutral_cost_;
      if(c >= lethal_cost_)
        c = lethal_cost_ - 1;
      return c;
    }
    return lethal_cost_;
  }

  int *buffer1_, *buffer2_, *buffer3_;
  int *currentBuffer_, *nextBuffer_, *overBuffer_;
  int currentEnd_, nextEnd_, overEnd_;
  bool *pending_;

  bool eight_connected_;
  bool allow_unknown_;

  unsigned int nx_, ny_, ns_, cells_visited_;

  float neutral_cost_, lethal_cost_, factor_;

  float threshold_;
  float priorityIncrement_;

};
} //end namespace lattice_planner
#endif
