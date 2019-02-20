cmake_minimum_required(VERSION 2.8.3)
project(rr_traffic_light)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  pluginlib
  rosconsole
  roscpp
  roslib
  sensor_msgs
  roslint
  std_srvs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(rr_traffic_light src/traffic_light.cpp src/traffic_light_main.cpp)
target_link_libraries(rr_traffic_light ${catkin_LIBRARIES} ${OpenCV_LIBS})
roslint_cpp(src/main.cpp src/rr_traffic_light.cpp include/rr_traffic_light.h)