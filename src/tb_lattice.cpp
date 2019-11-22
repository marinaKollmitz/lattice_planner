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

#include<lattice_planner/tb_lattice.h>

namespace lattice_planner
{

typedef std::unordered_map < DiscreteState, State*, Hasher > state_map;
typedef std::pair < DiscreteState, State* > state_pair;

int Hasher::num_gridcells_ = 0;
int Hasher::num_orientations_ = 0;
int Hasher::num_vels_w_ = 0;
int Hasher::num_vels_x_ = 0;

AStarLattice::AStarLattice(std::string name,costmap_2d::Costmap2DROS *costmap)
{
  //init a private ros node handle
  ros::NodeHandle private_nh("~/" + name);

  //init ros publishers
  vel_path_pub_ =
      private_nh.advertise<lattice_planner::Path>("plan", 1);
  expanded_paths_pub_ =
      private_nh.advertise<nav_msgs::Path>("expanded_paths", 1);

  //init debug publishers
  current_node_pub_ =
      private_nh.advertise<geometry_msgs::PoseStamped>("current_node", 1);
  new_node_pub_ =
      private_nh.advertise<geometry_msgs::PoseStamped>("new_node", 1);
  all_expanded_pub_ =
      private_nh.advertise<geometry_msgs::PoseArray>("all_expanded", 1);
  debug_no_interrupt_ = false;

  //get planner behavior params from ros param server

  std::string dynamic_layers_plugin_name;

  //costs
  CostManager::CostFactors cost_factors;
  private_nh.param("lethal_cost", cost_factors.lethal_cost,
                   (int)costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  private_nh.param("time_cost_factor", cost_factors.time_cost, 200.0);
  private_nh.param("step_cost_factor", cost_factors.step_cost, 20.0);
  private_nh.param("rotation_cost_factor", cost_factors.rotation_cost, 5.0);
  private_nh.param("environment_cost_factor", cost_factors.environment_cost, 1.0);
  private_nh.param("dynamic_layers_plugin", dynamic_layers_plugin_name, std::string("undefined"));

  //planner preferences
  double collision_check_time_res;
  private_nh.param("allow_unknown", allow_unknown_, false);
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.15);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
  private_nh.param("time_resolution", time_resolution_, 0.5);
  private_nh.param("collision_check_time_resolution", collision_check_time_res, 0.1);
  private_nh.param("time_steps_lookahead", max_timesteps_, 15);
  private_nh.param("planning_timeout", planning_timeout_, 0.48);
  private_nh.param("passive_navigation", passive_navigation_, false);
  private_nh.param("publish_expanded", publish_expanded_, false);

  //motion constraints
  MotionConstraints motion_constraints;
  private_nh.param("min_vel_x", motion_constraints.min_vel_x, 0.0);
  private_nh.param("max_vel_x", motion_constraints.max_vel_x, 0.4);
  private_nh.param("acceleration_x", motion_constraints.acc_x, 0.8);
  private_nh.param("min_vel_phi", motion_constraints.min_vel_phi, -0.8);
  private_nh.param("max_vel_phi", motion_constraints.max_vel_phi, 0.8);
  private_nh.param("acceleration_phi", motion_constraints.acc_phi, 1.6);

  //flags for easier path finding
  private_nh.param("easy_deceleration", easy_deceleration_, false);
  private_nh.param("easy_turn_at_start", easy_turn_at_start_, false);
  private_nh.param("easy_turn_at_goal", easy_turn_at_goal_, false);

  //create a hash map for every time step
  for(int i=0; i<max_timesteps_; i++)
  {
    state_map* time_map = new state_map();
    time_map->rehash(10000);
    time_map->reserve(10000);
    expanded_states_.push_back(time_map);
  }

  //dynamic costmap for costs associated with static and dynamic obstacles
  dynamic_costmap_ = new DynamicCostmap(costmap, max_timesteps_,
                                        ros::Duration(time_resolution_),
                                        dynamic_layers_plugin_name);

  double resolution = dynamic_costmap_->getStaticROSCostmap()->getCostmap()->getResolution();
  map_index_check_time_inkr_ = 0.5 * resolution / motion_constraints.max_vel_x;

  //cost manager for cost calculation
  cost_calc_ = new CostManager(dynamic_costmap_, motion_constraints, cost_factors);

  //state discretizer for discretizing the configuration space
  discretizer_ = new StateDiscretizer(dynamic_costmap_, motion_constraints);

  //hasher for generating unique hashes for the hash map
  int num_gridcells, num_orientations, num_vels_x, num_vels_w;
  discretizer_->getLimits(num_gridcells, num_orientations, num_vels_x, num_vels_w);
  Hasher::setLimits(num_gridcells, num_orientations, num_vels_x, num_vels_w);

  //trajectory rollout for robot motion calculation
  trajectory_rollout_ = new TrajectoryRollout(motion_constraints);

  //heuristics calculator for estimating the remaining cost to reach the goal
  heuristic_calc_ = new Heuristics(dynamic_costmap_, cost_factors, motion_constraints);
  heuristic_calc_->estimateAdditional(easy_deceleration_,easy_turn_at_goal_,
                                      xy_goal_tolerance_);
  heuristic_calc_->setHasUnknown(allow_unknown_);
  heuristic_calc_->initPublisher("lattice_planner/heuristics", 100);

}

AStarLattice::~AStarLattice()
{
  reset();

  std::vector< state_map* >::const_iterator it;
  for(it = expanded_states_.begin(); it != expanded_states_.end(); ++it)
  {
    delete (*it);
  }

  delete discretizer_;
  delete trajectory_rollout_;
  delete heuristic_calc_;
  delete cost_calc_;
}

//clean up everything
void AStarLattice::reset()
{
  debug_no_interrupt_ = false;
  //clean the expanded states
  std::vector< state_map* >::const_iterator it;
  for(it = expanded_states_.begin(); it != expanded_states_.end(); ++it)
  {
    state_map* time_map = *it;
    state_map::const_iterator states_it;

    for(states_it = time_map->begin(); states_it != time_map->end(); ++states_it)
      delete states_it->second;

    time_map->clear();
  }

  //empty priority queue
  queue_.clear();

  //empty the expanded paths
  expanded_paths_.poses.clear();
}

void AStarLattice::plannerTimeoutCallback(const ros::TimerEvent &event)
{
  planning_timed_out_ = true;
}

bool AStarLattice::findReplanningWaypoint(ros::Time conti_time,
                                        geometry_msgs::PoseStamped& conti_pose,
                                        geometry_msgs::Twist& replanning_start_vel)
{
  if(current_plan_.poses.empty())
  {
    ROS_WARN("AStarLattice: error while trying to find the next waypoint for "
             "continuous planning: plan is empty");
    return false;
  }

  conti_pose = current_plan_.poses.back();
  replanning_start_vel = current_plan_.velocities.back();

  int i=0;

  //check array for the first element that is at conti_time or later
  while(i < current_plan_.poses.size() &&
        fabs((conti_pose.header.stamp - conti_time).toSec()) > time_resolution_ / 2)
  {
    conti_pose = current_plan_.poses.at(i);
    replanning_start_vel = current_plan_.velocities.at(i);
    ++i;
  }

  //correct planning position for turning in place:
  //when turning in place, amcl's estimated robot pose can deviate a lot so that
  //after turning the robot's estimated position does not match the one from the
  //current plan any more. To overcome this problem, it is checked here whether the
  //robot is only turning and if so, the robot position for the next planning is
  //updated with the currently estimated position from amcl instead of the
  //waypoint from the current plan.
  try
  {
    geometry_msgs::Twist prev_vel = current_plan_.velocities.at(i - 2);
    if(prev_vel.linear.x == 0 && replanning_start_vel.linear.x == 0)
    {
      ROS_DEBUG("turn in place. update pose with amcl");
      geometry_msgs::PoseStamped robot_pose;

      if(dynamic_costmap_->getStaticROSCostmap()->getRobotPose(robot_pose))
      {
        conti_pose.pose.position.x = robot_pose.pose.position.x;
        conti_pose.pose.position.y = robot_pose.pose.position.y;
      }
    }
  }
  catch(std::out_of_range ex)
  {
    ROS_ERROR("could not adjust next waypoint for turning in place, out of range");
  }

  //check if element is the closest possible to conti time
  if(fabs((conti_pose.header.stamp - conti_time).toSec()) < time_resolution_ / 2)
    return true;

  else
  {
    ROS_ERROR_STREAM("AStarLattice: could not find next waypoint for continuous "
                     "planning for time " << conti_time << std::endl <<
                     current_plan_);
    return false;
  }
}

void AStarLattice::expandCircleAtStart(State* start)
{
  MotionConstraints mot_constr = trajectory_rollout_->getMotionConstraints();
  int prefix = 1;

  //loop twice: once for going clockwise and one anti clockwise
  for(int i = 0; i < 2; i++)
  {
    //start from the start node
    State* next_state = start;
    Pose next_pose = start->pose;
    Velocity next_vel = start->vel;

    Velocity travel_vel;
    Velocity vel_tmp;

    double turned_angle = 0;
    int time_step = 0;

    while(turned_angle < M_PI && time_step < max_timesteps_ -1)
    {
      //calculate the next velocity from the current one with the
      vel_tmp.vel_phi = next_vel.vel_phi + prefix * mot_constr.acc_phi * time_resolution_;
      if(vel_tmp.vel_phi > mot_constr.max_vel_phi)
        vel_tmp.vel_phi = mot_constr.max_vel_phi;
      else if(vel_tmp.vel_phi < mot_constr.min_vel_phi)
        vel_tmp.vel_phi = mot_constr.min_vel_phi;

      //traveling velocity is the average of the current and the next
      travel_vel.vel_phi = (next_vel.vel_phi + vel_tmp.vel_phi) / 2;
      next_vel = vel_tmp;

      //create the next state
      next_pose = trajectory_rollout_->getNextPose(next_pose, travel_vel, time_resolution_);
      DiscreteState next_state_i = discretizer_->discretizeState(next_pose, next_vel,
                                                                 time_step);

      next_state = new State(Hasher::getHash(next_state_i), next_state_i, next_pose,
                            next_vel, next_state->time + ros::Duration(time_resolution_),
                            next_state);

      //add state to priority queue
      addState(next_state);
    }
    //change the sign to turn negative
    prefix = prefix * -1;
  }
}

bool AStarLattice::addState(State *state)
{
  if(!state->state_i.in_map)
    return false;

  else
  {
    queue_.push_back(state);
    std::push_heap(queue_.begin(), queue_.end(), State::greater());

    std::pair < DiscreteState, State* > pair (state->state_i, state);
    expanded_states_.at(state->state_i.time_step)->insert(pair);

    return true;
  }
}

State* AStarLattice::getFirstPrioElement()
{
  State* first;
  if(queue_.empty())
    first = NULL;

  else
  {
    first = queue_[0];
    //delete current node from open list
    std::pop_heap(queue_.begin(), queue_.end(), State::greater());
    queue_.pop_back();
  }

  return first;
}

bool AStarLattice::getPath(geometry_msgs::PoseStamped start,
                           geometry_msgs::PoseStamped goal, bool replanning,
                           std::vector<geometry_msgs::PoseStamped>& path)
{
  reset();

  ros::Time begin = start.header.stamp;
  path_time_ = begin + ros::Duration(planning_timeout_);

  planning_timed_out_ = false;

  geometry_msgs::Twist start_vel;
  if(replanning && !current_plan_.poses.empty())
  {
    geometry_msgs::PoseStamped conti_pose;
    geometry_msgs::Twist conti_vel;

    //if the next conti waypoint for continuous path generation was found,
    //plan from there. If not, plan from the static start position
    if(findReplanningWaypoint(path_time_, conti_pose, conti_vel))
    {
      //start pose and start vel for continuous path generation
      start = conti_pose;
      start_vel = conti_vel;
      //time we want to release the new path
      path_time_ = start.header.stamp;
    }
    else
      ROS_ERROR("conti waypoint not found");
  }

#ifndef DEBUG
  ros::Timer planning_timer = nh_.createTimer(path_time_ - ros::Time::now(),
                                             &AStarLattice::plannerTimeoutCallback,
                                             this, true);
#endif

  dynamic_costmap_->update();

  //fill start and end pose
  start_pose_.setX(start.pose.position.x);
  start_pose_.setY(start.pose.position.y);
  start_pose_.setTheta(tf::getYaw(start.pose.orientation));

  goal_pose_.setX(goal.pose.position.x);
  goal_pose_.setY(goal.pose.position.y);
  goal_pose_.setTheta(tf::getYaw(goal.pose.orientation));

  //create start node
  DiscreteState state_i =
      discretizer_->discretizeState(start_pose_,Velocity(start_vel.linear.x,
                                                   start_vel.angular.z), 0);

  State* start_state = new State(Hasher::getHash(state_i), state_i, start_pose_,
                               Velocity(start_vel.linear.x, start_vel.angular.z),
                               path_time_, NULL);

  if(!addState(start_state))
  {
    ROS_ERROR("AStarLattice: start pose is not in costmap");
    return false;
  }

  //find goal position in map
  unsigned int goal_idx, goal_x_i, goal_y_i;
  if(!discretizer_->getCellPosition(goal_pose_.getX(), goal_pose_.getY(),
                                    goal_x_i, goal_y_i, goal_idx))
  {
    ROS_ERROR("AStarLattice: goal pose is not in costmap");
  }

  //calculate heuristic
  heuristic_calc_->expand(goal_x_i, goal_y_i);
  heuristic_calc_->publishHeuristic();

  //expand all orientations at the start to make the search quicker. Only for
  //initial path generation
  if(easy_turn_at_start_ && !replanning)
    expandCircleAtStart(start_state);

  State* final;
  expandLattice(final);

  current_plan_ = retracePath(final);
  vel_path_pub_.publish(current_plan_);

  path = current_plan_.poses;

  //publish search tree
  if(publish_expanded_)
  {
    expanded_paths_.header.frame_id =
        dynamic_costmap_->getStaticROSCostmap()->getGlobalFrameID();
    expanded_paths_.header.stamp = path_time_;
    expanded_paths_pub_.publish(expanded_paths_);
  }

  if(!path.empty())
    return true;

  else
    return false;
}

bool AStarLattice::expandLattice(State*& final)
{
  int cycles = 0;
  //loop for expanding the lattice until the goal region is reached

  State* current_node;
  State* current_best = queue_[0];
  bool is_goal = false;

  while(queue_.size() > 0 && ros::ok() && !planning_timed_out_)
  {
    //take the element with the lowest costs from the open list
    current_node = getFirstPrioElement();

    //expand current node. returns final node if this node expands the goal state
    expandNode(current_node, current_best, is_goal);
    final = current_best;

    if(is_goal)
    {
      //wait until the planning period timed out
      ros::Duration time_left = path_time_ - ros::Time::now();
      ROS_INFO("found path in %d cycles, sleeping for %f sec",
               cycles, time_left.toSec());
      time_left.sleep();
      return is_goal;
    }

    cycles++;
    ros::spinOnce();
  }

  final = current_best;

  ROS_INFO("lattice_planner: no path found within %d cycles", cycles);
  return is_goal;
}

void AStarLattice::expandNode(State* node, State*& current_best, bool& is_goal)
{
  //get vector with all primitives (admissible velocity combinations) for this node
  std::vector<Velocity> primitives =
      trajectory_rollout_->getReachableVelocities(node->vel,time_resolution_);

  //loop for all primitives
  for(int i=0; i<primitives.size(); i++)
  {
    //create a new state for each motion primitive
    State* state = createState(node, primitives.at(i));

    if(state != NULL) //check if the new state is valid
    {
      //find the best node for the planning so far
      double distance = state->pose.getDistance(goal_pose_);
      double rot_distance = state->pose.getRotDistance(goal_pose_);

      if(distance < xy_goal_tolerance_)
      {
        //if passive navigation is set we have to account for the lookahead too
        if(!passive_navigation_ || state->state_i.time_step >= max_timesteps_ - 1)
          state->in_goal_tolerance = true;

        //if state is in goal configuration
        if(fabs(rot_distance) < yaw_goal_tolerance_ &&
           fabs(state->vel.vel_x) < 0.01 && fabs(state->vel.vel_phi) < 0.01)
        {
          //lower cost of waiting in goal configuration
          state->way_cost -= 0.05 * cost_calc_->getCostFactors().time_cost;

          //if state is in goal configuration at the lookahead time
          if(!passive_navigation_ || state->state_i.time_step >= max_timesteps_ - 1)
          {
            //state is goal
            current_best = state;
            is_goal = true;
            return;
          }

          //state is the current best if none before has reached the goal
          //or if this is the one with the lowest total cost
          if(!current_best->in_goal_tolerance ||
             current_best->getTotalCost() > state->getTotalCost())
            current_best = state;
        }
      }

      //add state to priority queue
      addState(state);

      //if no node has hit the goal tolerance so far, take the one with the best
      //cost to goal which is the first in the priority queue
      if(!current_best->in_goal_tolerance)
      {
        current_best = queue_.at(0);
        is_goal = false;
      }

      if(publish_expanded_)
      {
        lattice_planner::Path vel_path = retracePath(state);
        std::vector<geometry_msgs::PoseStamped> poses = vel_path.poses;
        expanded_paths_.poses.insert(expanded_paths_.poses.end(), poses.begin(),
                                     poses.end());
        std::reverse(poses.begin(), poses.end());
        expanded_paths_.poses.insert(expanded_paths_.poses.end(), poses.begin(),
                                     poses.end());
      }

#ifdef DEBUG
      std::string frame_id = dynamic_costmap_->getFixedFrameId();

      //publish pose of the new expanded node
      new_node_pub_.publish(state->pose.getStampedPose(frame_id, state->time));

      //publish checked dynamic costmap
      dynamic_costmap_->visualizeCostmap(node->time);
#endif
    }
#ifdef DEBUG
    //publish pose of the expanding node
    current_node_pub_.publish(node->pose.getStampedPose(frame_id, node->time));

    //publish all expanded nodes
    geometry_msgs::PoseArray all_expanded;
    all_expanded.header.frame_id = frame_id;
    all_expanded.header.stamp = path_time_;
    std::vector<state_map*>::iterator map_it;
    for(map_it = expanded_states_.begin(); map_it != expanded_states_.end(); ++map_it)
    {
      state_map::iterator states_it;
      for(states_it = (*map_it)->begin(); states_it != (*map_it)->end(); ++states_it)
      {
        all_expanded.poses.push_back(states_it->second->pose.getGeomPose());
      }
    }

    all_expanded_pub_.publish(all_expanded);

    //write current and new node properties, prio queue and properties of all
    //expanded states to file
    std::string path = ros::package::getPath("lattice_planner");

    //write current priority queue to file
    std::ofstream queue_file (path + "/debug/queue.txt");
    if (queue_file.is_open())
    {
      queue_file << "cost \t ID \t idx \t ang \t v_x \t v_w \t t \t WC \t CTG \t ROT \t DEC" << std::endl;
      std::vector<State*>::iterator it;
      for(it = queue_.begin(); it != queue_.end(); ++it)
      {
        State* node = *it;
        queue_file << node->getDescription();
      }
      queue_file.close();
    }
    else
      ROS_ERROR("unable to open file");

    //write all expanded paths to file
    std::ofstream expanded_file (path + "/debug/expanded.txt");
    if (expanded_file.is_open())
    {
      expanded_file << "cost \t ID \t idx \t ang \t v_x \t v_w \t t \t WC \t CTG \t ROT \t DEC" << std::endl;
      std::vector<state_map*>::iterator map_it;
      for(map_it = expanded_states_.begin(); map_it != expanded_states_.end(); ++map_it)
      {
        state_map::iterator states_it;
        for(states_it = (*map_it)->begin(); states_it != (*map_it)->end(); ++states_it)
        {
          expanded_file << states_it->second->getDescription();
        }
      }
      expanded_file.close();
    }
    else
      ROS_ERROR("unable to open file");

    //write current and next node to file
    std::ofstream current_file (path + "/debug/current.txt");
    if (current_file.is_open())
    {
      current_file << " \t cost \t ID \t idx \t ang \t v_x \t v_w \t t \t WC \t CTG \t ROT \t DEC" << std::endl;
      current_file << "C \t" << node->getDescription();

      if(state != NULL)
        current_file << "N \t" << state->getDescription();

      else
        current_file << "N \t" << "state not valid" << std::endl;

      current_file.close();
    }
    else
      ROS_ERROR("unable to open file");

    //init user input to continue planning
    if(!debug_no_interrupt_)
    {
    std::string input;
    std::cout << "press enter to continue, c + enter to finish planning without interrupt: ";

    std::getline(std::cin, input);
    if(input.compare("c") == 0)
      debug_no_interrupt_ = true;
    }
#endif
  }
}

State* AStarLattice::createState(State *current, Velocity new_vel)
{
  //new state's time
  int next_time_step;
  ros::Time next_time;
  //bool in_timebound;

  if(current->state_i.time_step < max_timesteps_ - 1)
  {
    next_time_step = current->state_i.time_step + 1;
    //in_timebound = true;
  }
  else
  {
    next_time_step = current->state_i.time_step;
    //in_timebound = false;
  }

  next_time = current->time + ros::Duration(time_resolution_);

  //velocity from the current to the next state. average of both
  Velocity travel_vel((current->vel.vel_x + new_vel.vel_x)/2,
                  (current->vel.vel_phi + new_vel.vel_phi)/2);

  //next position
  Pose next_pos = trajectory_rollout_->getNextPose(current->pose, travel_vel,
                                                  time_resolution_);

  //create new state
  DiscreteState state_i = discretizer_->discretizeState(next_pos, new_vel,
                                                        next_time_step);
  State* new_state = new State(Hasher::getHash(state_i), state_i, next_pos,
                             new_vel, next_time, current);

  new_state->trajectory =
      trajectory_rollout_->findTrajectoryIndeces(current, new_state, travel_vel,
                                                 discretizer_, map_index_check_time_inkr_,
                                                 time_resolution_);

  //check if state is in map
  if(!new_state->state_i.in_map)
  {
    ROS_DEBUG("lattice_planner: new state not in map");
    delete new_state;
    return NULL;
  }

  //check if state is already expanded
  state_map* expanded_states = expanded_states_.at(next_time_step);
  state_map::const_iterator hit = expanded_states->find(new_state->state_i);
  if(hit != expanded_states->end())
  {
    ROS_DEBUG("lattice_planner: state is already expanded");
    //state is already expanded
    delete new_state;
    return NULL;
  }

  //calculate costs of state

  //calculate the cost to go from one state to the other
  double trajectory_cost = cost_calc_->getTrajectoryCost(new_state);

  //calculate the cost from the start to the new state
  double way_cost = current->way_cost + trajectory_cost;

  //estimate the remaining cost to goal for the state
  double cost_to_goal = heuristic_calc_->getHeuristic(new_state,goal_pose_);

  if(trajectory_cost == -1)
  {//trajectory will hit a lethal obstacle
    ROS_DEBUG("lattice_planner: trajectory will hit a lethal obstacle");
    delete new_state;
    return NULL;
  }

  new_state->setCosts(way_cost, cost_to_goal);

  return new_state;
}

lattice_planner::Path AStarLattice::retracePath(State* state)
{
  lattice_planner::Path path;

  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<geometry_msgs::Twist> velocities;

  geometry_msgs::PoseStamped one_pose;
  geometry_msgs::Twist one_vel;

  while(state != 0)
  {
    std::string fixed_frame =
        dynamic_costmap_->getStaticROSCostmap()->getGlobalFrameID();
    one_pose = state->pose.getStampedPose(fixed_frame, state->time);

    one_vel.linear.x = state->vel.vel_x;
    one_vel.angular.z = state->vel.vel_phi;

    poses.push_back(one_pose);
    velocities.push_back(one_vel);

    state = state->ancestor;
  }

  //reverse the poses and velocities
  std::reverse(poses.begin(), poses.end());
  std::reverse(velocities.begin(), velocities.end());

  path.header.stamp = poses.front().header.stamp;
  path.header.frame_id = poses.front().header.frame_id;

  path.poses = poses;
  path.velocities = velocities;

  return path;
}

}//namespace lattice_planner
