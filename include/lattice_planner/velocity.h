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

#ifndef _VELOCITY_H
#define _VELOCITY_H

namespace lattice_planner
{

/**
 * @brief data structure for velocity information
 */
struct Velocity
{

  /**
   * @brief default constructor
   */
  Velocity() :
    vel_x(0.0),
    vel_phi(0.0)
  {
  }

  /**
   * @brief constructor
   * @param vel_x forward velocity
   * @param vel_phi angular velocity
   */
  Velocity(double vel_x, double vel_phi) :
    vel_x(vel_x),
    vel_phi(vel_phi)
  {
  }

  double vel_x; ///< forward velocity
  double vel_phi; ///< angular velocity
};

}//namespace lattice_planner

#endif
