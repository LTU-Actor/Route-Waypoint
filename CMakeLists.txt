cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_route_waypoint)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  piksi_multi_cpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(waypoint src/waypoint.cpp)
add_dependencies(waypoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint ${catkin_LIBRARIES})
