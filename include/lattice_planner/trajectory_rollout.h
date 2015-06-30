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

#ifndef TRAJECTORYROLLOUT_H
#define TRAJECTORYROLLOUT_H

#include <lattice_planner/pose.h>
#include <lattice_planner/velocity.h>
#include <lattice_planner/motion_constraints.h>
#include <lattice_planner/state.h>
#include <lattice_planner/state_discretizer.h>

namespace lattice_planner
{

/**
 * @brief trajectory rollout class for a non-holonomic robot with differential
 *        drive constraints
 *
 * The TrajectoryRollout class provides methods to calculate the next position
 * and velocity from an initial position and velocity, according to the robot's
 * acceleration and velocity limits.
 *
 */
class TrajectoryRollout
{
public:

  /**
   * @brief constructor
   *
   * @param motion_constraints velocity and acceleration limits of the robot
   */
  TrajectoryRollout(MotionConstraints motion_constraints);

  /**
   * @brief get the defined velocity and acceleration limits of the robot
   *
   * @return the robot's motion constraints
   */
  MotionConstraints getMotionConstraints();

  /**
   * @brief calculate the next reachable velocities within a time interval, given
   *        the robot's acceleration and its velocity limits
   *
   * @param current_velocity robot's current velocity
   * @param time_delta time interval
   * @return a set of the velocities that are reachable within the given time
   *         interval
   */
  std::vector<Velocity> getReachableVelocities(Velocity current_velocity, double time_delta);

  /**
   * @brief obtain the next robot pose after traveling with a certain velocity
   *        from a current position for a certain time, assuming constant motion
   *        during the time interval
   *
   * @param current the current robot position
   * @param travel_vel the constant travel velocity
   * @param time_delta time interval for the motion
   * @return The next pose after the defined motion
   */
  Pose getNextPose(Pose current, Velocity travel_vel, double time_delta);

  /**
   * @brief get the map indeces of the cells that are traversed by the trajectory
   *        from one state to another, according to a defined motion
   *
   * @param begin start state of the trajectory
   * @param end end state of the trajectory
   * @param travel_vel constant travel velocity of the trajectory
   * @param discretizer state discretizer with discrete state properties
   * @param time_inkr increment to devide the trajectory into smaller steps to
   *        obtain the underlying map index
   * @param time_delta time interval for the motion
   * @return a set of map indeces that are traversed by the trajectory between
   *         two states
   */
  std::vector<unsigned int> findTrajectoryIndeces(State* begin, State* end,
                                                  Velocity travel_vel,
                                                  StateDiscretizer* discretizer,
                                                  double time_inkr,
                                                  double time_delta);

private:

  /**
   * @brief forbidden constructor
   */
  TrajectoryRollout();

  MotionConstraints motion_constraints_; ///< velocity and acceleration limits of the robot
};

}//namespace lattice_planner

#endif
