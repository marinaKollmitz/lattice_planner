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

#include<static_costmap.h>

PLUGINLIB_EXPORT_CLASS(dynamic_social_costmap::StaticLayers, lattice_planner::DynamicLayers)

namespace dynamic_social_costmap
{

  StaticLayers::~StaticLayers()
  {
    delete tf_;
    delete drsv_;
  }

void StaticLayers::initialize(costmap_2d::Costmap2DROS* static_map,
                              unsigned int max_timesteps, ros::Duration time_resolution)
{
  static_map_ = static_map;
  max_layers_ = max_timesteps;
  frame_id_ = static_map->getGlobalFrameID();
  time_resolution_ = time_resolution;

  //initialize publishers and subscribers
  // people_sub_ = nh_.subscribe("/people_prediction", 1, &StaticLayers::peopleCallback, this);

  //initialize dynamic reconfigure server
  ros::NodeHandle private_nh("dynamic_social_costmap");
  drsv_ = new dynamic_reconfigure::Server<dynamic_social_costmap::SocialCostmapConfig>(private_nh);
  dynamic_reconfigure::Server<dynamic_social_costmap::SocialCostmapConfig>::CallbackType cb;
  cb = boost::bind(&StaticLayers::reconfigureCB, this, _1, _2);
  drsv_->setCallback(cb);

  //initialize one empty layer for each represented time step
  for(int i=0; i<max_layers_; i++)
  {
    unsigned int size_x = static_map->getCostmap()->getSizeInCellsX();
    unsigned int size_y = static_map->getCostmap()->getSizeInCellsY();
    double origin_x = static_map->getCostmap()->getOriginX();
    double origin_y = static_map->getCostmap()->getOriginY();
    double resolution = static_map->getCostmap()->getResolution();

    unsigned char* costmap = new unsigned char[size_x * size_y];

    lattice_planner::TimedCostmap* onemap =
        new lattice_planner::TimedCostmap(size_x, size_y, origin_x, origin_y,
                                          resolution, costmap, frame_id_);
    onemap->time_index = i;
    onemap->fill(0);

    timed_costmap_.push_back(onemap);
  }
}

void StaticLayers::update()
{
  people_msgs::People people_k; //people in one timestep
  people_msgs::People people_kplus1; //people in the next timestep

  people_msgs::Person person_k; //person in one timestep
  people_msgs::Person person_kplus1; //person in the next timestep
  geometry_msgs::PoseStamped person_pose; //person with header

  double person_x, person_y; //coordinates of one person
  double angle_k; //angle in one timestep
  double angle_kplus1; //angle in the next timestep
  double person_angle; //angle of the person

  lattice_planner::TimedCostmap* timed_map; //one layer
  ros::Duration t_inkr; //time inkrement for the interpolation between two time steps

  //fill all costmaps with zero value
  for(int i=0; i<timed_costmap_.size(); i++)
  {
    timed_map = timed_costmap_.at(i);
    std::fill(timed_map->costmap, timed_map->costmap +
              (timed_map->size_x * timed_map->size_y), 0);
    timed_map->time = ros::Time(0);
  }

  int timedmaps_index = 0;

  //loop through the array of the predicted people and mark each prediction
  //in the corresponding dynamic cost map layer
  for(int i = 0; i < ((int) predicted_people_.predicted_people.size() - 1); i++)
  {

    if(timedmaps_index < max_layers_)
    {
      //find the predictions from the requested time step until the next
      people_k = predicted_people_.predicted_people.at(i);
      people_kplus1 = predicted_people_.predicted_people.at(i+1);
      ros::Duration delta_t_steps = people_kplus1.header.stamp - people_k.header.stamp;

      /**
       * @todo enforce the specified time for the dynamic cost map layers. Right
       *       now the time resolution is defined by the people prediction rate
       **/
      if(fabs((delta_t_steps -  time_resolution_).toSec()) > 0.1)
      {
        ROS_WARN_ONCE("dynamic costmap: the time resolution of the people prediction "
                      "does not match the specified resolution for the social navigation "
                      "planner.");
        ROS_WARN_ONCE("Planner: %f, prediction: %f", time_resolution_.toSec(), delta_t_steps.toSec());
      }

      //get the corresponding dynamic cost map layer
      timed_map = timed_costmap_.at(timedmaps_index);
      if(timed_map->time == ros::Time(0))
        timed_map->time = people_k.header.stamp; //stamp it, if necessary


      //loop through all predicted people in the time interval to mark them
      //in the cost map layer
      for(int j=0; j<people_k.people.size(); j++)
      {
        person_k = people_k.people.at(j);
        person_kplus1 = people_kplus1.people.at(j);
        //calculate the angle for a person by assuming that they walk forward
        angle_k = atan2(person_k.velocity.y, person_k.velocity.x);
        angle_kplus1 = atan2(person_kplus1.velocity.y, person_kplus1.velocity.x);

        //mark the human in the cost map layer. To capture the predicted trajectory,
        //we interpolate between the pose at time step k and the pose at step (k+1)
        //by using a finer time resolution t_step_resolution_
        t_inkr = ros::Duration(0.0);
        if(delta_t_steps > ros::Duration(0.0))
        {
          //the linear interpolation
          while(t_inkr <= delta_t_steps)
          {
            //find the intermediate interpolated pose of a person
            person_x = person_k.position.x
                + ( (person_kplus1.position.x - person_k.position.x)
                    * (t_inkr.toSec() / delta_t_steps.toSec()) );
            person_y = person_k.position.y
                + ( (person_kplus1.position.y - person_k.position.y)
                    * (t_inkr.toSec() / delta_t_steps.toSec()) );
            person_angle = angle_k
                + ( (angle_kplus1 - angle_k) * (t_inkr.toSec() / delta_t_steps.toSec()) );

            person_pose.header = people_k.header;
            person_pose.pose.position.x = person_x;
            person_pose.pose.position.y = person_y;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(person_angle),
                                  person_pose.pose.orientation);

            //get the position of the person in the map
            int person_in_map_x, person_in_map_y;
            double person_in_map_angle;

            if(getCostmapCoordinates(&person_pose, person_in_map_x,
                                     person_in_map_y, person_in_map_angle));
            {
              //if position is in the map, mark it in the map (put Gaussian cost
              //values around position
              markHumanInCostmap(person_in_map_x, person_in_map_y,
                                 person_in_map_angle, timed_map);
            }

            //enlarge time inkrement for next interpolation step
            t_inkr += interpolation_time_step_;
          }
        }
      }

      //take the next cost map layer
      if(delta_t_steps >= time_resolution_)
        timedmaps_index ++;
    }
  }

  //publish the cost map and cost cloud at the current positions of the predicted
  //people for visualization
  if(timed_costmap_.size() > 0)
  {
    timed_costmap_.front()->publishCostmap();
    timed_costmap_.front()->publishCostCloud();
  }

  //uncomment the following to save the cost map layers as map images for
  //debugging. Make sure the specified path is valid (create maps folder)
  /*
  std::string path = ros::package::getPath("dynamic_social_costmap");
  path.append("/maps/");
  saveTimedCostmaps(path);
  exit(0);
  */
}

unsigned int StaticLayers::getCost(unsigned int map_ix, unsigned int map_iy,
                                   ros::Time time)
{
  unsigned int cost = 0;

  for(int i=0; i<timed_costmap_.size(); i++)
  {
    if(timed_costmap_.at(i)->time > time)
      break;

    cost = timed_costmap_.at(i)->getCost(map_ix, map_iy);
  }

  return cost;
}

unsigned int StaticLayers::getCost(unsigned int map_index, ros::Time time)
{
  unsigned int cost = 0;

  for(int i=0; i<timed_costmap_.size(); i++)
  {
    if(timed_costmap_.at(i)->time > time)
      break;

    cost = timed_costmap_.at(i)->getCost(map_index);
  }

  return cost;
}

void StaticLayers::visualizeCostmapLayer(ros::Time time)
{
  lattice_planner::TimedCostmap* map = NULL;
  ros::Time target_time = ros::Time(0);

  int i = 0;

  for(i=0; i<timed_costmap_.size(); i++)
  {
    if(timed_costmap_.at(i)->time > time)
      break;

    map = timed_costmap_.at(i);
    target_time = map->time;
  }

  if(map != NULL)
  {
    ROS_DEBUG_STREAM_ONCE("dynamic_costmap: visualizing costmap at time " << target_time
                          << " for requested time " << time << " with index " << i);
    map->publishCostmap();
  }

  else
  {
    ROS_DEBUG_STREAM_ONCE("dynamic_costmap: cannot visualize costmap at time " << time
                          << "because it is outside the map time bounds");
  }
}

void StaticLayers::saveTimedCostmaps(std::string path)
{
  lattice_planner::TimedCostmap* timed_map;

  ROS_INFO("saving %lu costmaps to path %s", timed_costmap_.size(), path.c_str());

  //safe one map picture for every cost map layer
  for(unsigned int i=0; i<timed_costmap_.size(); i++)
  {
    timed_map = timed_costmap_.at(i);

    std::stringstream ss;
    ss << path << "map_" << i << ".pgm";
    timed_map->saveAsMapImage(ss.str().c_str());
  }

  //save the static map
  std::stringstream ss;
  ss << path << "static_map" << ".png";
  static_map_->getCostmap()->saveMap(ss.str().c_str());
}

void StaticLayers::reconfigureCB(dynamic_social_costmap::SocialCostmapConfig &config,
                                 uint32_t level)
{
  //update configuration
  amplitude_ = config.amplitude_multiplicator;
  cutoff_amplitude_ = config.cutoff_amplitude;
  variance_x_ = config.variance_x;
  variance_y_ = config.variance_y;
  forbidden_radius_ = config.forbidden_radius;
  offset_x_ = config.offset_x;
  offset_y_ = config.offset_y;
  amplitude_time_factor_ = config.amplitude_time_factor;
  variance_time_factor_ = config.variance_time_factor;
  forbidden_radius_time_factor_ = config.forbidden_radius_time_factor;

  int interpolation_steps = config.interpolation_steps;
  interpolation_time_step_ = ros::Duration(time_resolution_.toSec() / (interpolation_steps + 1));

  //visualize the new configuration
  if (timed_costmap_.size() > 0)
  {
    lattice_planner::TimedCostmap* timed_costmap =
        new lattice_planner::TimedCostmap(*timed_costmap_.front());
    timed_costmap->fill(0);

    for(int i=0; i<max_layers_; i++)
    {
      //mark the resulting cost function at different positions inside the cost map
      markHumanInCostmap(timed_costmap->size_x / 8 + (i * 1.0 / timed_costmap->resolution),
                         timed_costmap->size_y / 2, M_PI_4, timed_costmap);
    }

    //publish the marked costmap for visualization
    timed_costmap->publishCostCloud();
    ROS_INFO("updated social space params");

    delete timed_costmap;
  }
}

void StaticLayers::peopleCallback(const people_msgs::PeoplePredictionConstPtr people)
{
  ROS_DEBUG("dynamic costmap: received people callback");
  predicted_people_ = *people;
}

double StaticLayers::calcGaussian(double pos_x, double pos_y, double origin_x,
                                  double origin_y, double amplitude, double variance_x,
                                  double variance_y, double skew)
{
  double dx = pos_x-origin_x;
  double dy = pos_y-origin_y;

  double distance = sqrt(dx*dx+dy*dy);
  double angle = atan2(dy,dx);
  double mx = cos(angle-skew) * distance;
  double my = sin(angle-skew) * distance;

  double f1 = pow(mx, 2.0)/(2.0 * variance_x);
  double f2 = pow(my, 2.0)/(2.0 * variance_y);

  return amplitude * exp(-(f1 + f2));
}

double StaticLayers::calcCutoffRadius(double cutoff_value, double amplitude,
                                      double variance)
{
  return sqrt(-2*variance * log(cutoff_value/amplitude) );
}

bool StaticLayers::getCostmapCoordinates(geometry_msgs::PoseStamped* pose,
                                         int &cell_in_costmap_x, int &cell_in_costmap_y,
                                         double &angle_in_costmap)
{
  geometry_msgs::PoseStamped pose_in_map;

  //transform human position in fixed frame coordinates, if necessary
  if(pose->header.frame_id.compare(frame_id_) != 0)
  {
    try
    {
      tf_->waitForTransform(frame_id_, pose->header.frame_id,
                            pose->header.stamp, ros::Duration(5.0));
      tf_->transformPose(frame_id_, *pose, pose_in_map);
    }
    catch(tf::TransformException ex)
    {
      ROS_WARN("dynamic costmal: could not transform pose to map, tf answered: %s",
               ex.what());
      return false;
    }
  }

  else //position is already specified in map
    pose_in_map = *pose;

  //find cell coordinates of pose in costmap
  static_map_->getCostmap()->worldToMapNoBounds(pose->pose.position.x,
                                                pose->pose.position.y,
                                                cell_in_costmap_x,
                                                cell_in_costmap_y);

  //find the angle in the costmap
  angle_in_costmap = tf::getYaw(pose_in_map.pose.orientation);

  return true;
}

void StaticLayers::markHumanInCostmap(int human_in_costmap_x, int human_in_costmap_y,
                                      double angle, lattice_planner::TimedCostmap *costmap)
{
  //calculate the Gaussian params for the time index of the cost map layer
  double amplitude =
      (1 + costmap->time_index * amplitude_time_factor_) * amplitude_;
  double cutoff_amplitude =
      (1 - costmap->time_index * variance_time_factor_) * cutoff_amplitude_;
  double variance_x =
      (1 + costmap->time_index * variance_time_factor_) * variance_x_;
  double variance_y =
      (1 + costmap->time_index * variance_time_factor_) * variance_y_;
  double lethal_radius =
      (1 + costmap->time_index * forbidden_radius_time_factor_) * forbidden_radius_;

  //clamp the values
  amplitude = amplitude > 0.0 ? amplitude : 0.0;
  amplitude = amplitude < (double) costmap_2d::LETHAL_OBSTACLE ? amplitude : (double) costmap_2d::LETHAL_OBSTACLE;
  cutoff_amplitude = cutoff_amplitude > 0.0 ? cutoff_amplitude : 0.0;
  variance_x = variance_x > 0.0 ? variance_x : 0.0;
  variance_y = variance_y > 0.0 ? variance_y : 0.0;
  lethal_radius = lethal_radius > 0.0 ? lethal_radius : 0.0;

  //calculate gaussian around human
  if(amplitude > 0)
  {
    double resolution = static_map_->getCostmap()->getResolution();

    //calculate radius around human for cutoff amplitude
    double cutoff_radius = std::max(calcCutoffRadius(cutoff_amplitude,
                                                     amplitude,variance_x),
                                    calcCutoffRadius(cutoff_amplitude,
                                                     amplitude,variance_y));

    //calculate corresponding number of grid cells
    int cutoff_radius_grid = (int) std::min((cutoff_radius / resolution),
                                            (double) std::max(costmap->size_y,
                                                              costmap->size_x));

    //calculate the origin of the gaussian
    int origin_ix = (human_in_costmap_x + offset_x_ / resolution * cos(angle)
                     + offset_y_ / resolution * -sin(angle));
    int origin_iy = (human_in_costmap_y + offset_x_ / resolution * sin(angle)
                     + offset_y_ / resolution * cos(angle));

    //iterate through relevant grid cells and calculate the amplitude of the
    //gaussian for every cell. Mind costmap contraints
    int x_map_min = std::max(0, origin_ix - cutoff_radius_grid);
    int x_map_max = std::min(costmap->size_x, origin_ix +
                             cutoff_radius_grid);
    int y_map_min = std::max(0, origin_iy - cutoff_radius_grid);
    int y_map_max = std::min(costmap->size_y, origin_iy +
                             cutoff_radius_grid);

    for(unsigned int i=x_map_min; i<x_map_max; i++)
    {
      for(unsigned int j=y_map_min; j<y_map_max; j++)
      {
        unsigned char cost;

        //check if the cell is inside the lethal radius around human
        if((hypot(i*resolution - human_in_costmap_x*resolution,
                  j*resolution - human_in_costmap_y*resolution)) < lethal_radius)
          //assign lethal cost
          cost = costmap_2d::LETHAL_OBSTACLE;

        //calculate Gaussian value of cell
        else
        {
          double gauss_ampl = calcGaussian(i*resolution, j*resolution,
                                           origin_ix*resolution,
                                           origin_iy*resolution,
                                           amplitude, variance_x, variance_y, angle);

          cost = (unsigned char) gauss_ampl;

          //if the value is outside the cutoff value, set to zero
          if(gauss_ampl < cutoff_amplitude)
            cost = 0;

          //if the cost is larger than the lethal obstacle radius, clamp it
          else if (cost > costmap_2d::LETHAL_OBSTACLE)
            cost = costmap_2d::LETHAL_OBSTACLE;
        }

        costmap->setCost(i, j, std::max(cost, costmap->getCost(i, j)));
      }
    }
  }
}

} //namespace dynamic_costmap