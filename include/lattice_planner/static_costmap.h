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

#ifndef DYNAMIC_SOCIAL_LAYERS_H_
#define DYNAMIC_SOCIAL_LAYERS_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <lattice_planner/timed_costmap.h>
#include <lattice_planner/dynamic_layers.h>

// #include <people_msgs/People.h>
// #include <people_msgs/PersonStamped.h>
// #include <people_msgs/PeoplePrediction.h>

#include <dynamic_reconfigure/server.h>
#include <SocialCostmapConfig.h>

#include <pluginlib/class_list_macros.h>

namespace dynamic_social_costmap
{

/**
 * @brief The StaticLayers class provides a plugin for the dynamic layers of the
 *        dynamic costmap used by the lattice planner
 *
 * The social layers represent time dependent costs related to social navigation
 * constraints according to a Gaussian cost model and the predicted future
 * trajectories of humans in the environment.
 */
class StaticLayers : public lattice_planner::DynamicLayers
{
public:

  /**
   * @brief constructor
   */
  StaticLayers(){}

  /**
  * @brief destructor
  */
  ~StaticLayers();

  /**
   * @brief initialize the social layers
   * @param static_map the static map
   * @param max_timesteps maximum number of time steps represented by the dynamic layers
   * @param time_resolution time resolution of the dynamic layers
   */
  void initialize(costmap_2d::Costmap2DROS *static_map,
                  unsigned int max_timesteps, ros::Duration time_resolution);

  /**
   * @brief update the dynamic layers according to the most recent observations
   *        of humans in the environment
   *
   * update is called by the planner right before a new plan is computed. That way
   * the dynamic representation is only updated when needed, not every time a
   * new observation comes in
   */
  void update();

  /**
   * @brief get the socially induced cost for the requested time
   *
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @param time requested time
   * @return cost according to the defined time dependent social constraints
   */
  unsigned int getCost(unsigned int map_ix, unsigned int map_iy, ros::Time time);

  /**
   * @brief get the socially induced cost for the requested time
   *
   * @param map_index map index
   * @param time requested time
   * @return cost according to the defined time dependent social constraints
   */
  unsigned int getCost(unsigned int map_index, ros::Time time);

  /**
   * @brief publish the costmap layer that represents the requested time
   *        for visualization
   *
   * @param time requested time
   */
  void visualizeCostmapLayer(ros::Time time);

  /**
   * @brief save all costmap layers as image files for debugging
   *
   * @param path path to folder
   */
  void saveTimedCostmaps(std::string path);

protected:

  /**
   * @brief dynamic reconfigure callback
   *
   * @param config new configuration
   * @param level level
   */
  void reconfigureCB(dynamic_social_costmap::SocialCostmapConfig &config, uint32_t level);

  /**
   * @brief callback for predicted trajectories of observed people
   *
   * @param people predicted trajectories of observed people
   */
  // void peopleCallback(const people_msgs::PeoplePredictionConstPtr people);

  /**
   * @brief calculate value of 2D Gaussian function
   *
   * @param pos_x x coordinate
   * @param pos_y y coordinate
   * @param origin_x x origin of Gaussian
   * @param origin_y y origin of Gaussian
   * @param amplitude amplitude of Gaussian
   * @param variance_x variance in x
   * @param variance_y variance in y
   * @param skew skew to zero angle
   * @return value of 2D Gaussian function
   */
  double calcGaussian(double pos_x, double pos_y, double origin_x, double origin_y,
                      double amplitude, double variance_x, double variance_y,
                      double skew);

  /**
   * @brief calculate the distance to the origin of the Gaussian from which the
   *        values are lower than the cutoff_value
   *
   * @param cutoff_value Gaussian value bound
   * @param amplitude amplitude of Gaussian
   * @param variance variance
   * @return cutoff distance from origin
   */
  double calcCutoffRadius(double cutoff_value, double amplitude, double variance);

  /**
   * @brief find the (unbounded) coordinates in the cost map that correspond to
   *        a given pose in an arbitrary reference frame
   *
   * @param pose requested pose
   * @param[out] cell_in_costmap_x corresponding map cell x coordinate
   * @param[out] cell_in_costmap_y corresponding map cell y coordinate
   * @param[out] angle_in_costmap corresponding angle in map frame
   * @return whether the transformation into map frame was successfull
   */
  bool getCostmapCoordinates(geometry_msgs::PoseStamped *pose, int &cell_in_costmap_x,
                             int &cell_in_costmap_y, double &angle_in_costmap);

  /**
   * @brief update the relevant grid cells in the cost map according to the given
   *        pose of a person
   *
   * @param human_in_costmap_x map cell x coordinate of human
   * @param human_in_costmap_y map cell y coordinate of human
   * @param angle angle of human in map
   * @param costmap costmap to update
   */
  void markHumanInCostmap(int human_in_costmap_x, int human_in_costmap_y, double angle,
                          lattice_planner::TimedCostmap *costmap);

  std::vector<lattice_planner::TimedCostmap*> timed_costmap_; ///< the actual costmap layers
  // people_msgs::PeoplePrediction predicted_people_; ///< container for predicted human trajectories
  costmap_2d::Costmap2DROS* static_map_; ///< the underlying static map

  //gaussian parameters
  double amplitude_; ///< amplitude multiplication factor for gaussian
  double cutoff_amplitude_; ///< smallest cost value marked in map
  double variance_x_; ///< x variance for gaussian function
  double variance_y_; ///< y variance for gaussian function
  double forbidden_radius_; ///< radius around humans with untraversible cost
  double offset_x_; ///< x offset of Gaussian peak and person center (to the front)
  double offset_y_; ///< y offset of Gaussian peak and person center (to the side)

  //decay factors to capture the growing uncertainty of the predicted trajectory
  //with time
  double amplitude_time_factor_; ///< fraction of how much the amplitude changes with time
  double variance_time_factor_; ///< fraction of how much the variance changes with time
  double forbidden_radius_time_factor_; ///< fraction of how much the forbidden radius changes with time

  //configurations
  ros::Duration interpolation_time_step_; ///< time step to interpolate between two prediction steps to capture the whole trajectory
  ros::Duration time_resolution_; ///< time resolution between the cost map layers
  unsigned int max_layers_; ///< maximum number of cost map layers
  std::string frame_id_; ///< fixed frame ID of the cost map

  //ros related members
  ros::NodeHandle nh_; ///< ROS node handle
  tf::TransformListener* tf_; ///< tf transform listener
  ros::Subscriber people_sub_; ///< subscriber for predicted human trajectories
  dynamic_reconfigure::Server<dynamic_social_costmap::SocialCostmapConfig>* drsv_; ///< dynamic reconfigure server
};

} //namespace dynamic_social_costmap

#endif
