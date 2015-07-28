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

#ifndef TIMED_COSTMAP_H_
#define TIMED_COSTMAP_H_

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<costmap_2d/costmap_2d.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>

namespace lattice_planner
{

/**
 * @brief Container for one dynamic layer of the dynamic cost map
 */
class TimedCostmap
{
public:
  /**
   * @brief constructor
   *
   * @param size_x number of cells in x
   * @param size_y number of cells in y
   * @param origin_x map origin in x
   * @param origin_y map origin in y
   * @param resolution map resolution
   * @param costmap the char array costmap
   * @param frame_id global frame id
   */
  TimedCostmap(int size_x, int size_y, double origin_x, double origin_y,
               double resolution, unsigned char* costmap, std::string frame_id) :
    time(ros::Time(0)),
    size_x(size_x),
    size_y(size_y),
    origin_x(origin_x),
    origin_y(origin_y),
    resolution(resolution),
    costmap(costmap),
    frame_id(frame_id)
  {
    //initialize the publishers
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_costmap", 1);
    cost_cloud_pub_.advertise(nh_, "/social_cost_cloud", 1);
  }

  /**
   * @brief calculate 1D cell index from 2D indeces in x and y
   *
   * @param mx map cell index in x
   * @param my map cell index in y
   * @return map index
   */
  int getIndex(int mx, int my)
  {
    return my * size_x + mx;
  }

  /**
   * @brief calculate 2D cell indeces from 1D index
   *
   * @param index map index
   * @param[out] mx map cell index in x
   * @param[out] my map cell index in y
   */
  void indexToCells(unsigned int index, int &mx, int &my)
  {
    my = index / size_x;
    mx = index - (my * size_x);
  }

  /**
   * @brief get cost value from cost map
   *
   * @param mx map cell index in x
   * @param my map cell index in y
   * @return cost value
   */
  unsigned char getCost(int mx, int my)
  {
    if(mx < 0 || my < 0 || mx >= size_x || my >= size_y)
      return 0;
    else
      return getCost(getIndex(mx, my));
  }

  /**
   * @brief get cost value from cost map
   *
   * @param index map index
   * @return cost value
   */
  unsigned char getCost(int index)
  {
    return costmap[index];
  }

  /**
   * @brief set the cost value of a cell
   *
   * @param index map index
   * @param cost cost value
   */
  void setCost(int index, unsigned char cost)
  {
    costmap[index] = cost;
  }

  /**
   * @brief set the cost value of a cell
   *
   * @param mx map cell index in x
   * @param my map cell index in y
   * @param cost cost value
   */
  void setCost(int mx, int my, unsigned char cost)
  {
    costmap[getIndex(mx, my)] = cost;
  }

  /**
   * @brief fill the costmap with a specific value
   *
   * @param value value to fill
   */
  void fill(unsigned char value)
  {
    std::fill(costmap, costmap + (size_x * size_y), value);
  }

  /**
   * @brief publish the cost map for visualization
   */
  void publishCostmap()
  {
    //wrap the cost map in OccupancyGrid form for rviz
    nav_msgs::OccupancyGrid one_costmap;
    one_costmap.header.frame_id = frame_id;
    one_costmap.header.stamp = time;
    one_costmap.info.width = size_x;
    one_costmap.info.height = size_y;
    one_costmap.info.origin.position.x = origin_x;
    one_costmap.info.origin.position.y = origin_y;
    one_costmap.info.origin.position.z = 0.0;
    one_costmap.info.resolution = resolution;

    //scale the entries with a factor and fill the OccupancyGrid
    double scale_factor = 100;

    for(int j=0; j<one_costmap.info.height * one_costmap.info.width; j++)
      one_costmap.data.push_back(getCost(j) * scale_factor/costmap_2d::LETHAL_OBSTACLE);

    //publish
    costmap_pub_.publish(one_costmap);
  }

  /**
   * @brief publish the cost cloud (3D bell curves) for visualization. The height
   *        of the 3D points represents the cost value of the cell
   */
  void publishCostCloud()
  {
    //create new point cloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cost_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    double max_height = 1.5; //max height of costmap in meters

    //fill the point cloud
    for(int j=0; j<size_x * size_y; j++)
    {
        int mx, my;
        indexToCells(j, mx, my);
        pcl::PointXYZRGB pnt;
        pnt.x = mx * resolution + origin_x;
        pnt.y = my * resolution + origin_y;
        //height of the point is determined by the cost value of the cell
        pnt.z = max_height * ((double) getCost(j) / (double) costmap_2d::LETHAL_OBSTACLE);
        double z_normalized = ((double) getCost(j) / (double) costmap_2d::LETHAL_OBSTACLE);
        //color of the point is also determined by the cost value of the cell
        pnt.r = 255 * (z_normalized);
        pnt.b = 255 * (1 - z_normalized);
        pnt.g = 255 * (1 - (fabs(z_normalized - 0.5) * 2));

        if(z_normalized == 0)
        {
          pnt.z = -1; //move under ground plane for viz
          pnt.r = 255;
          pnt.g = 255;
          pnt.b = 255;
        }

        cost_cloud->push_back(pnt);
    }

    //specify some general properties of the point cloud
    cost_cloud->is_dense = false;

    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = time;
    cost_cloud->width = size_x;
    cost_cloud->height = size_y;
    cost_cloud->header = pcl_conversions::toPCL(header);

    //publish
    cost_cloud_pub_.publish(*cost_cloud);
  }

  /**
   * @brief save the cost map as an image for visualization
   * @param path path to save
   */
  void saveAsMapImage(std::string path)
  {
    //wrap the costmap in costmap_2d::Costmap2D form
    costmap_2d::Costmap2D map(size_x, size_y, resolution, origin_x, origin_y);
    for(unsigned int i = 0; i<size_x * size_y; i++)
    {
      //fill the map
      int idx_x, idx_y;
      indexToCells(i, idx_x, idx_y);
      map.setCost(idx_x,idx_y,getCost(i));
    }
    //save map as image
    map.saveMap(path);
  }

  ros::Time time; ///< time represented by layer
  int time_index; ///< time index (time step) of layer
  int size_x; ///< number of cells in x
  int size_y; ///< number of cells in y
  double origin_x; ///< map origin in x
  double origin_y; ///< map origin in y
  double resolution; ///< map resolution
  unsigned char* costmap; ///< array costmap
  std::string frame_id; ///< global frame id

private:
  ros::NodeHandle nh_; ///< node handle for publishing
  ros::Publisher costmap_pub_; ///< cost map publisher
  pcl_ros::Publisher<pcl::PointXYZRGB> cost_cloud_pub_; ///< cost cloud publisher
};

} //namespace lattice_p;la

#endif
