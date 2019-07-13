cmake_minimum_required(VERSION 2.8.3)
project(rr_path_planner)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  roscpp 
  rospy 
  std_msgs 
  nav_msgs 
  geometry_msgs
  roslint
  rr_common
  rr_computer_vision
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  nav_msgs 
  geometry_msgs
  rr_common
  rr_computer_vision
  visualization_msgs
  DEPENDS system_lib
)

roslint_cpp(src/path_planner_main.cpp src/path_planner.cpp include/path_planner.h)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# include_directories(rr_path_planner ${catkin_EXPORTED_TARGETS})
add_compile_options(-std=c++11)
add_library(path_planner src/path_planner.cpp include/path_planner.h)
add_executable(rr_path_planner src/path_planner_main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rr_path_planner path_planner ${catkin_LIBRARIES})
