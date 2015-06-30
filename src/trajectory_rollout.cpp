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

#include<lattice_planner/trajectory_rollout.h>

namespace lattice_planner
{

TrajectoryRollout::TrajectoryRollout(MotionConstraints motion_constraints) :
  motion_constraints_(motion_constraints)
{
}

std::vector<Velocity> TrajectoryRollout::getReachableVelocities(Velocity current_velocity,
                                                           double time_delta)
{
  std::vector<Velocity> vels;
  Velocity vel;

  //set of next velocities: velocity component stays the same, decelerates or
  //accelerates within max and min velocity

  //construct the set of possible velocities for the current velocity
  //x velocity component
  for(int j=0; j<3; j++)
  {
    if(j==0)
      vel.vel_x = current_velocity.vel_x;
    else if(j==1)
      vel.vel_x = current_velocity.vel_x + motion_constraints_.acc_x * time_delta;
    else if(j==2)
      vel.vel_x = current_velocity.vel_x - motion_constraints_.acc_x * time_delta;
    //phi velocity component
    for(int h=0; h<3; h++)
    {
      if(h==0)
        vel.vel_phi = current_velocity.vel_phi;
      else if(h == 1)
        vel.vel_phi = current_velocity.vel_phi + motion_constraints_.acc_phi * time_delta;
      else if(h == 2)
        vel.vel_phi = current_velocity.vel_phi - motion_constraints_.acc_phi * time_delta;

      //check if new velocity is within velocity limits
      if(vel.vel_x >= motion_constraints_.min_vel_x &&
         vel.vel_x <= motion_constraints_.max_vel_x &&
         vel.vel_phi >= motion_constraints_.min_vel_phi &&
         vel.vel_phi <= motion_constraints_.max_vel_phi)
      {
        //            if(vel.v_x_ != 0) - only if vehicle is not able to turn in place
        vels.push_back(vel);
      }
    }
  }
  return vels;
}

MotionConstraints TrajectoryRollout::getMotionConstraints()
{
  return motion_constraints_;
}

Pose TrajectoryRollout::getNextPose(Pose current, Velocity travel_vel,
                                    double time_delta)
{
  double x, y, theta;

  if(travel_vel.vel_phi != 0)
  { //angular velocity is not zero, the trajectory is an arc

    double arc_radius = travel_vel.vel_x / travel_vel.vel_phi;

    x = current.getX() + arc_radius * (- sin(current.getTheta())
                                        + sin(current.getTheta()
                                              + travel_vel.vel_phi * time_delta));
    y = current.getY() + arc_radius * (cos(current.getTheta())
                                        - cos(current.getTheta()
                                              + travel_vel.vel_phi * time_delta));
    theta = current.getTheta() + travel_vel.vel_phi * time_delta;
  }
  else
  {//angular velocity is zero, robot is only moving forward
    x = current.getX()
        + travel_vel.vel_x * cos(current.getTheta()) * time_delta;
    y = current.getY()
        + travel_vel.vel_x * sin(current.getTheta()) * time_delta;
    theta = current.getTheta();
  }

  return Pose(x, y, theta);
}

std::vector<unsigned int>
TrajectoryRollout::findTrajectoryIndeces(State *begin, State *end, Velocity travel_vel,
                                         StateDiscretizer *discretizer,
                                         double time_inkr, double time_delta)
{
  std::vector<unsigned int> indeces;
  indeces.push_back(begin->state_i.grid_cell);
  double time = time_inkr;
  Pose next = begin->pose;
  unsigned int x_i, y_i, index;

  while(time < time_delta)
  {
    next = getNextPose(next, travel_vel, time_inkr);
    if(discretizer->getCellPosition(next.getX(), next.getY(), x_i, y_i, index))
      indeces.push_back(index);

    time += time_inkr;
  }

  indeces.push_back(end->state_i.grid_cell);

  return indeces;
}

}//namespace lattice_planner
