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

#ifndef _POSE_H
#define _POSE_H

#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>

namespace lattice_planner
{

/**
 * @brief container for the (3D planar) robot pose and pose related properties,
 * conversions and calculations
 */
class Pose
{
public:

  /**
   * @brief constructor
   *
   * @param x x position
   * @param y y position
   * @param theta orientation
   */
  Pose(double x, double y, double theta) :
    x_(x),
    y_(y)
  {
    setTheta(theta);
  }

  /**
   * @brief default constructor
   */
  Pose() :
    x_(0.0),
    y_(0.0)
  {
    setTheta(0.0);
  }

  /**
   * @brief specify the x position
   *
   * @param x x position
   */
  void setX(double x)
  {
    this->x_ = x;
  }

  /**
   * @brief specify the y position
   *
   * @param y y position
   */
  void setY(double y)
  {
    this->y_ = y;
  }

  /**
   * @brief specify the orientation
   *
   * if necessary, the orientation is forced to be between 0 and 2PI
   *
   * @param theta orientation
   */
  void setTheta(double theta)
  {
    this->theta_ = theta;

    //make sure theta is between 0 and 2PI
    while(this->theta_ < 0)
      this->theta_ += 2 * M_PI;

    while(this->theta_ >= 2 * M_PI)
      this->theta_ -= 2 * M_PI;
  }

  /**
   * @brief get x position
   *
   * @return x position
   */
  double getX()
  {
    return x_;
  }

  /**
   * @brief get y position
   *
   * @return y position
   */
  double getY()
  {
    return y_;
  }

  /**
   * @brief get orientation around z-axis
   *
   * @return orientation around z-axis, between 0 and 2PI
   */
  double getTheta()
  {
    return theta_;
  }

  /**
   * @brief get geometry_msgs::Pose version of the pose
   *
   * @return pose wrapped into geometry_msgs::Pose type
   */
  geometry_msgs::Pose getGeomPose()
  {
    geometry_msgs::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta_),
                          pose.orientation);
    return pose;
  }

  /**
   * @brief get geometry_msgs::PoseStamped version of the pose
   *
   * @param frame_id frame id for the pose
   * @param time time stamp
   * @return pose wrapped into geometry_msgs::PoseStamped type
   */
  geometry_msgs::PoseStamped getStampedPose(std::string frame_id,
                                            ros::Time time)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = getGeomPose();
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = time;

    return pose_stamped;
  }

  /**
   * @brief calculates the Cartesian distance between two poses
   *
   * @param other pose to which the distance is calculated
   * @return distance between the poses
   */
  double getDistance(Pose& other)
  {
    return hypot(x_ - other.getX(), y_ - other.getY());
  }

  /**
   * @brief calculates the difference in orientation between two poses
   *
   * returned orientation difference is always positive and reflects the minimum
   * angular distance between the poses.
   *
   * @param other pose to which the orientation distance is calculated
   * @return orientation difference between the poses
   */
  double getRotDistance(Pose& other)
  {
    return std::min(fabs(theta_ - other.getTheta()),
                    fabs(fabs(theta_ - other.getTheta()) - 2 * M_PI));
  }

private:
  double x_; ///< x position
  double y_; ///< y position
  double theta_; ///< orientation around the z-axis, in radians, between 0 and 2PI
};

}//namespace lattice_planner

#endif
