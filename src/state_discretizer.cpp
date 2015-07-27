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

#include <lattice_planner/state_discretizer.h>

namespace lattice_planner
{

StateDiscretizer::StateDiscretizer(DynamicCostmap *costmap,
                                   MotionConstraints motion_constraints) :
  costmap_(costmap),
  time_resolution_(costmap->getTimeResolution().toSec()),
  motion_constraints_(motion_constraints)
{
  double nx = costmap->getStaticROSCostmap()->getCostmap()->getSizeInCellsX();
  double ny = costmap->getStaticROSCostmap()->getCostmap()->getSizeInCellsY();
  num_gridcells_ = nx * ny;

  //calculate number of discrete angles for dynamic settings
  num_orientations_ = getNumDiscreteAngles(motion_constraints.acc_phi,
                                           time_resolution_);

  //calculate number of discrete forward velocities
  num_vel_x_ = 0;
  for(double vel = motion_constraints.min_vel_x;
      vel <= motion_constraints.max_vel_x;
      vel += motion_constraints.acc_x * time_resolution_)
    num_vel_x_ ++;

  //calculate number of discrete rotational velocities
  num_vel_phi_ = 0;
  for(double vel = motion_constraints.min_vel_phi;
      vel <= motion_constraints.max_vel_phi;
      vel += motion_constraints.acc_phi * time_resolution_)
    num_vel_phi_ ++;

  vel_step_x_ = motion_constraints_.acc_x * time_resolution_;
  vel_step_phi_ = motion_constraints_.acc_phi * time_resolution_;
}

int StateDiscretizer::getNumDiscreteAngles(double acc_w, double time_delta)
{
  //calculate the number of orientations that should be represented.
  //needs to be able to jump from one discrete angle to the other at minimum
  //turning speed. otherwise it will expand states that are already there
  //and therefore never be able to turn in place
  int max_angle = (int) ((4 * M_PI) / (acc_w * time_delta) + 1);
  int num_angles = max_angle + 1; //need to be able to represent max_angles + zero
  ROS_INFO("discretizing to %d angles", num_angles);

  return num_angles;
}

DiscreteState StateDiscretizer::discretizeState(Pose pose, Velocity vel, int time_step)
{
  DiscreteState state_i;

  //find cell position
  state_i.in_map = getCellPosition(pose.getX(), pose.getY(), state_i.x_i,
                                   state_i.y_i, state_i.grid_cell);

  //find the nearest discrete orientation
  state_i.angle_i = getDiscreteOrientation(pose.getTheta());

  //find the discrete representation of the velocity
  getDiscreteVelocity(vel, state_i.vel_x_i, state_i.vel_w_i);

  state_i.time_step = time_step;

  return state_i;
}

bool StateDiscretizer::getCellPosition(double x, double y, unsigned int &x_i,
                                       unsigned int &y_i, unsigned int &index)
{
  //find the corresponding grid cell and x y coordinate
  if(costmap_->getStaticROSCostmap()->getCostmap()->worldToMap(x, y, x_i, y_i))
  {
    index = costmap_->getStaticROSCostmap()->getCostmap()->getIndex(x_i, y_i);
    return true;
  }

  else
    return false;
}

unsigned int StateDiscretizer::getDiscreteOrientation(double angle)
{
  if(angle < 0 || angle > 2 * M_PI)
  {
    ROS_ERROR("angle has to be between 0 and 2_PI for calculating the nearest "
              "discrete angle");
    return 0;
  }

  //get next discrete angle
  unsigned int orientation = angle / (2 * M_PI / (num_orientations_-1)) + 0.5;

  if(orientation == num_orientations_)
    orientation = 0;

  return orientation;
}

void StateDiscretizer::getDiscreteVelocity(Velocity vel, unsigned int &vel_x_i,
                                           unsigned int &vel_w_i)
{
  vel_x_i = int ((vel.vel_x / vel_step_x_) -
                 (motion_constraints_.min_vel_x / vel_step_x_) + 0.5);
  vel_w_i = int ((vel.vel_phi / vel_step_phi_) -
                 (motion_constraints_.min_vel_phi / vel_step_phi_) + 0.5);
}

void StateDiscretizer::getLimits(int &num_gridcells, int &num_orientations,
                                 int & num_vels_x, int &num_vels_w)
{
  num_gridcells = num_gridcells_;
  num_orientations = num_orientations_;
  num_vels_x = num_vel_x_;
  num_vels_w = num_vel_phi_;
}

Velocity StateDiscretizer::getVelocity(unsigned int vel_x_i, unsigned int vel_w_i)
{
  return Velocity(vel_x_i * vel_step_x_, vel_w_i * vel_step_phi_);
}

} //namespace lattice_planner
