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

#include<lattice_planner/heuristics.h>

namespace lattice_planner
{

Heuristics::Heuristics(DynamicCostmap *costmap,
                       CostManager::CostFactors cost_factors,
                       MotionConstraints motion_constraints) :
  costmap_(costmap),
  pub_initialized_(false),
  heuristics_(NULL),
  cost_factors_(cost_factors),
  estimate_deceleration_at_goal_(false),
  estimate_rotation_to_goal_(false),
  motion_constraints_(motion_constraints)
{
  unsigned int nx = costmap->getStaticROSCostmap()->getCostmap()->getSizeInCellsX();
  unsigned int ny = costmap->getStaticROSCostmap()->getCostmap()->getSizeInCellsY();
  double resolution = costmap->getStaticROSCostmap()->getCostmap()->getResolution();

  double time_factor =
      (resolution/(motion_constraints.max_vel_x * costmap->getTimeResolution().toSec()))
      *cost_factors.time_cost;

  grid_expander_ = new DijkstraExpansion(nx, ny);
  setLethalCost(cost_factors.lethal_cost);
  setNeutralCost(cost_factors.step_cost + time_factor);

  heuristics_ = new float[nx * ny];
}

bool Heuristics::expand(int goal_x, int goal_y)
{
  unsigned int nx = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsX();
  unsigned int ny = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsY();
  unsigned char* char_costmap = costmap_->getStaticROSCostmap()->getCostmap()->getCharMap();

  int num_cells = nx * ny;
  return grid_expander_->expand(char_costmap, goal_x, goal_y, num_cells, heuristics_);
}

void Heuristics::estimateAdditional(bool estimate_deceleration_at_goal,
                                    bool estimate_rotation_to_goal_orientation,
                                    double estimation_radius)
{
  estimate_deceleration_at_goal_ = estimate_deceleration_at_goal;
  estimate_rotation_to_goal_ = estimate_rotation_to_goal_orientation;
  additional_estimation_radius_ = estimation_radius;
}

float Heuristics::getHeuristic(State *state, Pose goal_pose)
{
  if(heuristics_ != NULL)
  {
    unsigned int nx = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsX();
    unsigned int ny = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsY();
    int num_cells = nx * ny;

    if(state->state_i.grid_cell < num_cells)
    {
      float heuristics = heuristics_[state->state_i.grid_cell];

      if(estimate_deceleration_at_goal_)
      {
        heuristics += estimateDecelerationAtGoal(state,goal_pose);
      }
      if(estimate_rotation_to_goal_)
      {
        heuristics += estimateTurnToGoal(state, goal_pose);
      }
      return heuristics;
    }

    else
    {
      ROS_ERROR("getHeuristic: index out of bounds");
      return -1;
    }
  }
  else
  {
    ROS_ERROR("heuristic has not been calculated yet");
    return -1;
  }
}

double Heuristics::estimateTurnToGoal(State* state, Pose goal_pose)
{

  //time to rotate the remaining angle. Only estimate correctly within a certain
  //distance of the goal. Add maximum angle outside. Encourages turn in place
  //at the goal

  double time_delta = costmap_->getTimeResolution().toSec();

  double rot_cost = 0;
  double distance = state->pose.getDistance(goal_pose);

  if(distance <= additional_estimation_radius_)
  {
    rot_cost = (fabs(state->pose.getRotDistance(goal_pose)) /
                (std::max(fabs(motion_constraints_.min_vel_phi),
                          fabs(motion_constraints_.max_vel_phi))
                 * time_delta)) * cost_factors_.time_cost;
  }
  else
  {
    rot_cost = (M_PI / (std::max(fabs(motion_constraints_.min_vel_phi),
                                 fabs(motion_constraints_.max_vel_phi))
                        * time_delta)) * cost_factors_.time_cost;
  }

  return rot_cost;
}

double Heuristics::estimateDecelerationAtGoal(State* state, Pose goal_pose)
{
  //time to decelerate to zero velocity. Only estimate correctly within a certain
  //distance to goal. Add maximum deceleration time outside. Encourages
  //decelerating close to the goal

  double time_delta = costmap_->getTimeResolution().toSec();

  double distance = state->pose.getDistance(goal_pose);
  double decel_radius = time_delta * motion_constraints_.max_vel_x;
  double decel_cost = 0;

  if(distance <= decel_radius)
  {
    decel_cost += fabs(state->vel.vel_x) /
        (motion_constraints_.acc_x * time_delta) * cost_factors_.time_cost;
    decel_cost += fabs(state->vel.vel_phi) /
        (motion_constraints_.acc_phi * time_delta) * cost_factors_.time_cost;
  }

  else
  {
    decel_cost += fabs(motion_constraints_.max_vel_x) /
        (motion_constraints_.acc_x * time_delta) * cost_factors_.time_cost;
    decel_cost += fabs(motion_constraints_.max_vel_phi) /
        (motion_constraints_.acc_phi * time_delta) * cost_factors_.time_cost;
  }

  return decel_cost;
}

void Heuristics::initPublisher(std::string topic_name, int publish_scale)
{
  ros::NodeHandle nh;
  heuristics_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
  publish_scale_ = publish_scale;
  pub_initialized_ = true;
}

void Heuristics::publishHeuristic(float* heuristics)
{
  if(pub_initialized_)
  {
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = costmap_->getStaticROSCostmap()->getGlobalFrameID();
    grid.header.stamp = ros::Time::now();

    grid.info.resolution = costmap_->getStaticROSCostmap()->getCostmap()->getResolution();
    grid.info.width = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsX();
    grid.info.height = costmap_->getStaticROSCostmap()->getCostmap()->getSizeInCellsY();

    grid.info.origin.position.x = costmap_->getStaticROSCostmap()->getCostmap()->getOriginX();
    grid.info.origin.position.y = costmap_->getStaticROSCostmap()->getCostmap()->getOriginY();
    grid.info.origin.position.z = 0.05;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);

    //find the max grid entry
    int max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
      int potential = heuristics[i];
      if (potential < POT_HIGH)
      {
        if (potential > max)
          max = potential;
      }
    }

    //scale all entries according to max value
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
      if (heuristics[i] >= POT_HIGH)
      {
        grid.data[i] = -1;
      }
      else
        grid.data[i] = heuristics[i] * publish_scale_ / max;
    }
    heuristics_pub_.publish(grid);
  }
  else
  {
    ROS_WARN("heuristics publisher has not been initialized, not publishing "
             "heuristic");
  }
}

void Heuristics::publishHeuristic()
{
  publishHeuristic(heuristics_);
}

}//namespace lattice_planner
