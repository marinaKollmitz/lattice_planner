# lattice_planner
The lattice_planner package provides a move_base global planner plugin for a time-bounded A* lattice planner. The planner is designed to plan time dependent, dynamically feasible navigation paths for robots with differential drive constraints. It uses a dynamic cost map which is based on the ROS costmap representation from the costmap_2d package.

Please have a look at the [human_aware_navigation package](https://github.com/marinaKollmitz/human_aware_navigation) for an example application including a toy example in the context of human-aware navigation.

If you use our lattice planner code for your research, please consider citing our paper:

```
@INPROCEEDINGS{kollmitz15ecmr,
  author = {Marina Kollmitz and Kaijen Hsiao and Johannes Gaa and Wolfram Burgard},
  title = {Time Dependent Planning on a Layered Social Cost Map for Human-Aware Robot Navigation},
  booktitle = {Proc.~of the IEEE Eur.~Conf.~on Mobile Robotics (ECMR)},
  year = {2015},
  doi = {10.1109/ECMR.2015.7324184},
  url = {http://ais.informatik.uni-freiburg.de/publications/papers/kollmitz15ecmr.pdf}
}
```

the following ROS Parameters can be specified for the lattice_planner:

prefix for all ROS params: move_base/TBLattice/ 

cost factors:
- lethal_cost --> cost associated with forbidden areas (e.g. obstacles)
- time_cost_factor --> cost factor for path execution time
- step_cost_factor --> cost factor for path length
- rotation_cost_factor --> cost factor for the accumulated turns along a planned path 
- environment_cost_factor --> cost factor for constraints defined in the dynamic environment, according to the dynamic cost map
- dynamic_layers_plugin --> fully qualified plugin type for dynamic layers to be used (must adhere to lattice_planner::DynamicLayers interface. See pluginlib doc on ROS wiki for help)

planner preferences:
- allow_unknown --> whether the planner is allowed to expand into unknown map regions
- xy_goal_tolerance --> the Euclidean goal tolerance distance in meters
- yaw_goal_tolerance --> the rotational goal tolerance in rad
- time_resolution --> time resolution of the discretized configuration space in seconds
- collision_check_time_resolution --> time increment to slice trajectories for collision checking in seconds
- time_steps_lookahead --> maximum number of time steps for planning
- planning_timeout --> timeout for the planning after which the best solution found so far is returned
- passive_navigation --> flag for planning dynamically for the max number of time steps, not only until the goal is found
- publish_expanded --> flag for publishing the expanded search tree for visualization

flags for simplifying the path search - violate optimality but accelerate the search procedure:
- easy_deceleration --> flag for simplified planning of deceleration at the goal
- easy_turn_at_start --> flag for simplified planning of turn in place at the start
- easy_turn_at_goal --> flag for simplified planning of turn in place at the goal

motion constraints of the robot:
- min_vel_x --> minimum forward velocity of the robot in meters per second (can be negative)
- max_vel_x --> maximum forward velocity of the robot in meters per second
- acceleration_x --> forward acceleration in meters per second^2
- min_vel_phi --> minimum angular velocity of the robot in radians per second (can be negative)
- max_vel_phi --> maximum angular velocity of the robot in radians per second
- acceleration_phi --> angular acceleration in radians per second^2
