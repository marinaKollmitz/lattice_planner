#ifndef DYNAMIC_LAYERS_H_
#define DYNAMIC_LAYERS_H_

#include <costmap_2d/costmap_2d_ros.h>

namespace lattice_planner
{

/**
 * @brief The DynamicLayers class defines the plugin interface for the dynamic
 *        layers of the dynamic_costmap used by the lattice_planner.
 */
class DynamicLayers
{
public:
  /**
   * @brief initialize function for the dynamic layers plugin
   * @param static_map the static map
   * @param max_timesteps maximum number of time steps represented by the dynamic layers
   * @param time_resolution time resolution of the dynamic layers
   */
  virtual void initialize(costmap_2d::Costmap2DROS* static_map,
                          unsigned int max_timesteps, ros::Duration time_resolution) = 0;

  /**
   * @brief update the dynamic cost map
   *
   * update is called by the planner right before a new plan is computed. That way
   * the dynamic representation is only updated when needed, not every time a
   * new observation comes in
   */
  virtual void update() = 0;

  /**
   * @brief get the (dynamic) cost at the requested time
   * @param map_ix map cell index in x
   * @param map_iy map cell index in y
   * @param time requested time
   * @return dynamic cost according to the defined time dependent constraints
   */
  virtual unsigned int getCost(unsigned int map_ix, unsigned int map_iy, ros::Time time) = 0;

  /**
   * @brief get the (dynamic) cost at the requested time
   * @param map_index map index
   * @param time requested time
   * @return dynamic cost according to the defined time dependent constraints
   */
  virtual unsigned int getCost(unsigned int map_index, ros::Time time) = 0;

protected:
  /**
   * @brief default constructor
   */
  DynamicLayers(){}
};

};

#endif
