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

#ifndef DYNAMIC_COSTMAP_H_
#define DYNAMIC_COSTMAP_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_loader.h>
#include <lattice_planner/dynamic_layers.h>

namespace lattice_planner
{

/**
 * @brief The DynamicCostmap class defines a layered dynamic cost map representation
 *        for time dependent planning.
 *
 * It contains of one static layer that follows the 2D cost map representation
 * also used by the ROS navigation stack. On top of that, dynamic layers account
 * for the time dependent navigation constraints. The dynamic layers are are
 * defined as plugins to allow for a custom definition of time dependent navigation
 * constraints. @see DynamicLayers
 */
class DynamicCostmap
{
public:

  /**
   * @brief constructor
   *
   * @param static_map static cost map
   * @param max_layers maximum number of layers (represented time steps)
   * @param time_resolution time resolution of the dynamic layers
   * @param dynamic_layers_plugin fully qualified plugin type for dynamic layers to be used
   */
  DynamicCostmap(costmap_2d::Costmap2DROS *static_map, int max_layers,
                 ros::Duration time_resolution, std::string dynamic_layers_plugin) :
    static_map_(static_map),
    time_resolution_(time_resolution),
    dl_loader_("lattice_planner", "lattice_planner::DynamicLayers")

  {
    //load the plugin for the dynamic layers
    try
    {
      ROS_INFO("dynamic_costmap: Using dynamic layers plugin %s",
               dynamic_layers_plugin.c_str());

      dynamic_layers_ =
          dl_loader_.createInstance(dynamic_layers_plugin);

      dynamic_layers_->initialize(static_map, max_layers, time_resolution);
    }

    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("dynamic_costmap: failed to load dynamic layers plugin: %s", ex.what());
    }
  }

  ~DynamicCostmap()
  {
    //clean up
    delete static_map_;
    dynamic_layers_.reset();
  }

  /**
   * @brief update the dynamic layers according to the most recent environment
   *        observations
   */
  void update()
  {
    dynamic_layers_->update();
  }

  /**
   * @brief get cost value from static layer
   *
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @return cost value from static layer
   */
  unsigned char getStaticCost(unsigned int map_ix, unsigned int map_iy)
  {
    return static_map_->getCostmap()->getCost(map_ix, map_iy);
  }

  /**
   * @brief get cost value from static layer
   *
   * @param map_index map index
   * @return cost value from static layer
   */
  unsigned char getStaticCost(unsigned int map_index)
  {
    //find corresponding 2D map indeces in x and y from 1D index
    unsigned int map_ix, map_iy;
    static_map_->getCostmap()->indexToCells(map_index, map_ix, map_iy);

    return getStaticCost(map_ix, map_iy);
  }

  /**
   * @brief get cost value from dynamic layers at the specified time
   *
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @param time requested time
   * @return cost value from dynamic layer
   */
  unsigned char getDynamicCost(unsigned int map_ix, unsigned int map_iy,
                               ros::Time time)
  {
    return dynamic_layers_->getCost(map_ix, map_iy, time);
  }

  /**
   * @brief get cost value from dynamic layers at the specified time
   *
   * @param map_index map index
   * @param time requested time
   * @return cost value from dynamic layer
   */
  unsigned char getDynamicCost(unsigned int map_index, ros::Time time)
  {
    return dynamic_layers_->getCost(map_index, time);
  }

  /**
   * @brief get cost at a specific time as a combination of the static and dynamic
   *        cost map layer at the specified time
   *
   * The returned cost is the maximum value of static and dynamic cost
   *
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @param time requested time
   * @return combined cost from static and dynamic cost map layer
   */
  unsigned char getCost(unsigned int map_ix, unsigned int map_iy,
                        ros::Time time)
  {
    return std::max(getStaticCost(map_ix, map_iy),
                    getDynamicCost(map_ix, map_iy, time));
  }

  /**
   * @brief get cost at a specific time as a combination of the static and dynamic
   *        cost map layer at the specified time
   *
   * The returned cost is the maximum value of static and dynamic cost
   *
   * @param map_index map index
   * @param time requested time
   * @return combined cost from static and dynamic cost map layer
   */
  unsigned char getCost(unsigned int map_index, ros::Time time)
  {
    return std::max(getStaticCost(map_index),
                    getDynamicCost(map_index, time));
  }

  /**
   * @brief get static layer of the dynamic cost map
   *
   * @return static cost map layer
   */
  costmap_2d::Costmap2DROS* getStaticROSCostmap()
  {
    return static_map_;
  }

  /**
   * @brief get the time resolution of the dynamic layers
   *
   * @return time resolution of dynamic layers
   */
  ros::Duration getTimeResolution() const
  {
    return time_resolution_;
  }

protected:

  costmap_2d::Costmap2DROS* static_map_; ///< static layer
  pluginlib::ClassLoader<lattice_planner::DynamicLayers> dl_loader_;
  boost::shared_ptr<lattice_planner::DynamicLayers> dynamic_layers_; ///< dynamic layers, defined as plugin
  ros::Duration time_resolution_; ///< time resolution of dynamic layers

private:

  /**
   * @brief default constructor
   */
  DynamicCostmap();
};

} //namespace lattice_planner
#endif
