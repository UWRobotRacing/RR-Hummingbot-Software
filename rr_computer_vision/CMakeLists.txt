cmake_minimum_required(VERSION 2.8.3)
project(rr_computer_vision)


set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  roslint
  rr_common
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  TrafficSign.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(rr_computer_vision ${catkin_EXPORTED_TARGETS})

roslint_cpp(src/computer_vision.cpp include/computer_vision.hpp 
src/lane_detection.cpp include/lane_detection.hpp
src/sign_detection.cpp include/sign_detection.hpp
src/traffic_light.cpp include/traffic_light.hpp
src/endline_detection.cpp include/endline_detection.hpp)

add_compile_options(-std=c++11)
add_executable(rr_computer_vision src/computer_vision_main.cpp src/computer_vision.cpp include/computer_vision.hpp 
src/lane_detection.cpp include/lane_detection.hpp
src/sign_detection.cpp include/sign_detection.hpp
src/traffic_light.cpp include/traffic_light.hpp
src/endline_detection.cpp include/endline_detection.hpp)
add_dependencies(rr_computer_vision rr_computer_vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rr_computer_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})