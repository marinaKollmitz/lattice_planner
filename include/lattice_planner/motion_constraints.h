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

#ifndef MOTIONCONSTRAINTS_H_
#define MOTIONCONSTRAINTS_H_

namespace lattice_planner
{

/**
 * @brief container for the motion constraints of the robot
 */
struct MotionConstraints
{
  /**
   * @brief default constructor
   */
  MotionConstraints():
    min_vel_x(0.0),
    max_vel_x(0.0),
    acc_x(0.0),
    min_vel_phi(0.0),
    max_vel_phi(0.0),
    acc_phi(0.0)
  {

  }

  /**
   * @brief constructor
   * @param min_vel_x minimum forward velocity of the robot (can be negative)
   * @param max_vel_x maximum forward velocity of the robot
   * @param acc_x forward acceleration
   * @param min_vel_phi minimum angular velocity of the robot (can be negative)
   * @param max_vel_phi maximum angular velocity of the robot
   * @param acc_phi angular acceleration
   */
  MotionConstraints(double min_vel_x, double max_vel_x, double acc_x,
                    double min_vel_phi, double max_vel_phi, double acc_phi) :
    min_vel_x(min_vel_x),
    max_vel_x(max_vel_x),
    acc_x(acc_x),
    min_vel_phi(min_vel_phi),
    max_vel_phi(max_vel_phi),
    acc_phi(acc_phi)
  {
  }

  double min_vel_x; ///< minimum forward velocity, can be negative
  double max_vel_x; ///< maximum forward velocity
  double acc_x; ///< forward acceleration
  double min_vel_phi; ///< minimum angular velocity, can be negative
  double max_vel_phi; ///< maximum angular velocity
  double acc_phi; ///< angular acceleration
};

} //namespace lattice_planner

#endif
