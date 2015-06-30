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

#include <lattice_planner/dijkstra.h>
#include <algorithm>

namespace lattice_planner {

DijkstraExpansion::DijkstraExpansion(int nx, int ny) :
  pending_(NULL),
  eight_connected_(true),
  factor_(1.0),
  neutral_cost_(0.0),
  lethal_cost_((float) costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
{
  // priority buffers
  buffer1_ = new int[PRIORITYBUFSIZE];
  buffer2_ = new int[PRIORITYBUFSIZE];
  buffer3_ = new int[PRIORITYBUFSIZE];

  priorityIncrement_ = 2 * neutral_cost_;
  setSize(nx, ny);
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys)
{
  nx_ = xs;
  ny_ = ys;
  ns_ = xs * ys;

  if (pending_)
    delete[] pending_;

  pending_ = new bool[ns_];
  memset(pending_, 0, ns_ * sizeof(bool));
}

bool DijkstraExpansion::expand(unsigned char* costs, unsigned int start_x,
                               unsigned int start_y,
                               int max_cycles, float* potential)
{
  return expand(costs, start_x, start_y, potential, max_cycles, false);
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//
bool DijkstraExpansion::expandToGoal(unsigned char *costs, unsigned int start_x,
                                     unsigned int start_y, unsigned int goal_x,
                                     unsigned int goal_y, int max_cycles,
                                     float *potential)
{
  return expand(costs, start_x, start_y, potential, max_cycles, true, goal_x, goal_y);
}

bool DijkstraExpansion::expand(unsigned char *costs, unsigned int start_x,
                               unsigned int start_y, float *potential,
                               int max_cycles, bool has_goal,
                               unsigned int goal_x, unsigned int goal_y)
{
  cells_visited_ = 0;
  // priority buffers
  threshold_ = lethal_cost_;
  currentBuffer_ = buffer1_;
  currentEnd_ = 0;
  nextBuffer_ = buffer2_;
  nextEnd_ = 0;
  overBuffer_ = buffer3_;
  overEnd_ = 0;
  memset(pending_, 0, ns_ * sizeof(bool));
  std::fill(potential, potential + ns_, POT_HIGH);

  // set goal
  int start = toIndex(start_x, start_y);

  potential[start] = 0;
  push_cur(start+1);
  push_cur(start-1);
  push_cur(start-nx_);
  push_cur(start+nx_);

  if(eight_connected_)
  {
    push_cur(start-nx_-1);
    push_cur(start-nx_+1);
    push_cur(start+nx_-1);
    push_cur(start+nx_+1);
  }

  int nwv = 0;            // max priority block size
  int nc = 0;            // number of cells put into priority blocks
  int cycle = 0;        // which cycle we're on

  // set up start cell
  int goal = toIndex(goal_x, goal_y);

  for (; cycle < max_cycles; cycle++) // go for this many cycles, unless interrupted
  {
    //
    if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
      return false;

    // stats
    nc += currentEnd_;
    if (currentEnd_ > nwv)
      nwv = currentEnd_;

    // reset pending_ flags on current priority buffer
    int *pb = currentBuffer_;
    int i = currentEnd_;
    while (i-- > 0)
      pending_[*(pb++)] = false;

    // process current priority buffer
    pb = currentBuffer_;
    i = currentEnd_;
    while (i-- > 0)
    {
      if(eight_connected_)
        updateCellEightConnect(costs, potential, *pb++);
      else
        updateCell(costs, potential, *pb++);
    }

    // swap priority blocks currentBuffer_ <=> nextBuffer_
    currentEnd_ = nextEnd_;
    nextEnd_ = 0;
    pb = currentBuffer_;        // swap buffers
    currentBuffer_ = nextBuffer_;
    nextBuffer_ = pb;

    // see if we're done with this priority level
    if (currentEnd_ == 0) {
      threshold_ += priorityIncrement_;    // increment priority threshold
      currentEnd_ = overEnd_;    // set current to overflow block
      overEnd_ = 0;
      pb = currentBuffer_;        // swap buffers
      currentBuffer_ = overBuffer_;
      overBuffer_ = pb;
    }

    // check if we've hit the goal
    if(has_goal)
    {
      if (potential[goal] < POT_HIGH)
        break;
    }
  }
  //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
  if (cycle < max_cycles)
    return true; // finished up here
  else
    return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define SQRT2 1.414213562

float DijkstraExpansion::calculatePotential(float *potential, float cost, int n)
{
  float prev_potential;

  //get min of neighbors
  float min_horizontal = std::min( potential[n - 1], potential[n + 1] );
  float min_vertical = std::min( potential[n - nx_], potential[n + nx_]);
  float min_diag_up = std::min( potential[n - nx_ -1], potential[n - nx_ + 1]);
  float min_diag_down = std::min( potential[n + nx_ -1], potential[n + nx_ + 1]);

  float min_streight = std::min(min_horizontal, min_vertical);
  float min_diag = std::min(min_diag_up, min_diag_down);

  //check if min potential value is in diagonal or streight cell
  if(min_streight < min_diag || !eight_connected_)
    prev_potential = min_streight + neutral_cost_ + cost;

  else
    prev_potential = min_diag + SQRT2 * neutral_cost_ + cost;

  return prev_potential;
}

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCellEightConnect(unsigned char *costs, float *potential, int n)
{
  cells_visited_++;

  // do planar wave update
  float c = costs[n];
  if (c >= costmap_2d::LETHAL_OBSTACLE)    // don't propagate into obstacles
    return;

  if(c == costmap_2d::NO_INFORMATION && !allow_unknown_)
    return;

  //calculate potential of current cell based on the costmap value, the
  //neutral cost and the smallest potential of all neighbours
  float pot = calculatePotential(potential, c, n);

  // now add affected neighbors to priority blocks
  if (pot < potential[n])
  { //should always be true
    float le = costs[n-1] + neutral_cost_;
    float re = costs[n + 1] + neutral_cost_;
    float ue = costs[n - nx_] + neutral_cost_;
    float de = costs[n + nx_] + neutral_cost_;
    float diag_lue = costs[n - nx_ - 1] + SQRT2 * neutral_cost_;
    float diag_rue = costs[n - nx_ + 1] + SQRT2 * neutral_cost_;
    float diag_lde = costs[n + nx_ - 1] + SQRT2 * neutral_cost_;
    float diag_rde = costs[n + nx_ + 1] + SQRT2 * neutral_cost_;

    potential[n] = pot;
//    ROS_INFO("UPDATE %d %d %d %f, %f", n, n%nx_, n/nx_, potential[n], threshold_);
    if (pot < threshold_)    // low-cost buffer block
    {
//      ROS_INFO("pot < threshold_");
      if (potential[n - 1] > pot + le)
        push_next(n-1);
      if (potential[n + 1] > pot + re)
        push_next(n+1);
      if (potential[n - nx_] > pot + ue)
        push_next(n-nx_);
      if (potential[n + nx_] > pot + de)
        push_next(n+nx_);
      if (potential[n - nx_ - 1] > pot + diag_lue)
        push_next(n - nx_ - 1);
      if (potential[n - nx_ + 1] > pot + diag_rue)
        push_next(n - nx_ + 1);
      if (potential[n + nx_ - 1] > pot + diag_lde)
        push_next(n + nx_ - 1);
      if (potential[n + nx_ + 1] > pot + diag_rde)
        push_next(n + nx_ + 1);
    }
    else            // overflow block
    {
//      ROS_ERROR("potential >= threshold_");
      if (potential[n - 1] > pot + le)
        push_over(n-1);
      if (potential[n + 1] > pot + re)
        push_over(n+1);
      if (potential[n - nx_] > pot + ue)
        push_over(n-nx_);
      if (potential[n + nx_] > pot + de)
        push_over(n+nx_);
      if (potential[n - nx_ - 1] > pot + diag_lue)
        push_over(n - nx_ - 1);
      if (potential[n - nx_ + 1] > pot + diag_rue)
        push_over(n - nx_ + 1);
      if (potential[n + nx_ - 1] > pot + diag_lde)
        push_over(n + nx_ - 1);
      if (potential[n + nx_ + 1] > pot + diag_rde)
        push_over(n + nx_ + 1);
    }
  }
//  else
//    ROS_ERROR("pot >= potential[n]");
}

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) {
  cells_visited_++;

  // do planar wave update
  float c = getCost(costs, n);
  if (costs[n] >= costmap_2d::LETHAL_OBSTACLE)    // don't propagate into obstacles
    return;

  float pot = calculatePotential(potential, c, n);

  // now add affected neighbors to priority blocks
  if (pot < potential[n]) {
    float le = INVSQRT2 * (float)getCost(costs, n - 1);
    float re = INVSQRT2 * (float)getCost(costs, n + 1);
    float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
    float de = INVSQRT2 * (float)getCost(costs, n + nx_);
    potential[n] = pot;
//    ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
    if (pot < threshold_)    // low-cost buffer block
    {
      if (potential[n - 1] > pot + le)
        push_next(n-1);
      if (potential[n + 1] > pot + re)
        push_next(n+1);
      if (potential[n - nx_] > pot + ue)
        push_next(n-nx_);
      if (potential[n + nx_] > pot + de)
        push_next(n+nx_);
    } else            // overflow block
    {
      if (potential[n - 1] > pot + le)
        push_over(n-1);
      if (potential[n + 1] > pot + re)
        push_over(n+1);
      if (potential[n - nx_] > pot + ue)
        push_over(n-nx_);
      if (potential[n + nx_] > pot + de)
        push_over(n+nx_);
    }
  }
}

} //end namespace lattice_planner
