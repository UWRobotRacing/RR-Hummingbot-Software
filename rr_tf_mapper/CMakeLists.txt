cmake_minimum_required(VERSION 2.8.3)
project(rr_tf_mapper)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  roscpp 
  rospy 
  std_msgs 
  nav_msgs 
  geometry_msgs
  roslint
  rr_common
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
  visualization_msgs
  DEPENDS system_lib
)

roslint_cpp(src/tf_mapper_main.cpp src/tf_mapper.cpp include/tf_mapper.h)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rr_tf_mapper src/tf_mapper_main.cpp)
add_library(tf_mapper src/tf_mapper.cpp include/tf_mapper.h)

## Specify libraries to link a library or executable target against
target_link_libraries(rr_tf_mapper tf_mapper ${catkin_LIBRARIES})
