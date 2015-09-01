cmake_minimum_required(VERSION 2.8.3)
project(lattice_planner)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    message_generation
)

add_message_files(
  FILES
    Path.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    src/planner_core.cpp
    src/tb_lattice.cpp
    src/cost_manager.cpp
    src/trajectory_rollout.cpp
    src/heuristics.cpp
    src/dijkstra.cpp
    src/state_discretizer.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
