cmake_minimum_required(VERSION 2.8.3)
project(rr_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  roslint
  message_generation
)

add_message_files(
  FILES
  Transmitter.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(rr_interface src/interface_main.cpp src/interface.cpp include/interface.hpp)
target_link_libraries(rr_interface ${catkin_LIBRARIES}) 
