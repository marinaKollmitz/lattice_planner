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

#include <lattice_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>

namespace lattice_planner
{

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(lattice_planner::TBLatticePlanner,
                       nav_core::BaseGlobalPlanner)

TBLatticePlanner::TBLatticePlanner() :
  costmap_(NULL),
  initialized_(false)
{
}

TBLatticePlanner::TBLatticePlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap,
                                   std::string frame_id) :
  costmap_(NULL),
  initialized_(false)
{
  //initialize the planner
  initialize(name, costmap, frame_id);
}

void TBLatticePlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros, costmap_ros->getGlobalFrameID());
}

void TBLatticePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap,
                               std::string frame_id)
{
  if (!initialized_)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~/" + name);

    //for calling the path planning service without execution of the plan
    make_plan_srv_ =
        private_nh.advertiseService("make_plan",
                                    &TBLatticePlanner::makePlanService, this);

    //init replanning service
    replan_service_ =
        nh.advertiseService("move_base/replan",
                            &TBLatticePlanner::replanServiceCallback,this);

    path_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);

    costmap_ = costmap->getCostmap();
    frame_id_ = frame_id;
    planner_ = new AStarLattice(name, costmap);

    initialized_ = true;
    replanning_requested_ = false;
  }
  else
    ROS_WARN("This planner has already been initialized");

}

//create a border at the edges of the costmap with cost value (e.g. lethal
//cost for the planner expander not to expand beyond the costmap bounds)
void TBLatticePlanner::outlineMap(costmap_2d::Costmap2D *costmap,
                               unsigned char value)
{
  unsigned char* map_tmp = costmap->getCharMap();
  unsigned char* map = costmap->getCharMap();
  unsigned int nx = costmap->getSizeInCellsX();
  unsigned int ny = costmap->getSizeInCellsY();

  for (int i = 0; i < nx; i++)
    *map++ = value;
  map = map_tmp + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)
    *map++ = value;
  map = map_tmp;
  for (int i = 0; i < ny; i++, map += nx)
    *map = value;
  map = map_tmp + nx - 1;
  for (int i = 0; i < ny; i++, map += nx)
    *map = value;
}

bool TBLatticePlanner::replanServiceCallback(std_srvs::Empty::Request &request,
                                             std_srvs::Empty::Response &response)
{
  ROS_INFO("PlannerCore: replanning service call received");
  ros::Duration(0.25).sleep();
  replanning_requested_ = true;
  return true;
}

void TBLatticePlanner::clearRobotCell(costmap_2d::Costmap2D* costmap,
                                   unsigned int mx, unsigned int my)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, "
              "please call initialize() before use");
    return;
  }

  //set the associated costs in the cost map to be free
  costmap->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool TBLatticePlanner::makePlanService(nav_msgs::GetPlan::Request& req,
                                    nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);

  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

bool TBLatticePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, "
              "please call initialize() before use");
    return false;
  }

  //clear the plan, just in case
  plan.clear();

  geometry_msgs::PoseStamped start_fixed_frame;
  geometry_msgs::PoseStamped goal_fixed_frame;

  tf::TransformListener tfl;

  //transform start pose into fixed frame
  try
  {
    ros::Duration timeout(0.5);
    tfl.waitForTransform(frame_id_, start.header.frame_id, start.header.stamp,
                         timeout);
    tfl.transformPose(frame_id_, start, start_fixed_frame);
    tfl.waitForTransform(frame_id_, goal.header.frame_id, goal.header.stamp,
                         timeout);
    tfl.transformPose(frame_id_, goal, goal_fixed_frame);

  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("lattice_planner: could not transform pose into fixed "
              "frame: %s", ex.what());
  }

  //set the time step to now. important for the timeout for the dynamic planner
  start_fixed_frame.header.stamp = ros::Time::now();

  //draw border of lethal obstacle around costmap
  outlineMap(costmap_, costmap_2d::LETHAL_OBSTACLE);

  //check if replanning from zero vel was requested
  ros::spinOnce();

  //check if the goal is the same and we can plan continuously
  bool continuous = true;

  if(goal_fixed_frame.pose.position.x != goal_pose_.pose.position.x
     || goal_fixed_frame.pose.position.y != goal_pose_.pose.position.y
     || replanning_requested_)
  {
    continuous = false;
    replanning_requested_ = false;
  }

  goal_pose_ = goal_fixed_frame;

  bool planning_ok = planner_->getPath(start_fixed_frame, goal_fixed_frame,
                                       continuous, plan);

  if(!planning_ok)
    plan.clear();

  publishPlan(plan);

  return true;
}

void TBLatticePlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> plan)
{
  nav_msgs::Path path;

  if(!plan.empty())
  {
    path.header.stamp = plan.at(0).header.stamp;
    path.header.frame_id = plan.at(0).header.frame_id;
    path.poses = plan;
  }

  path_pub_.publish(path);
}

} //namespace lattice_planner
