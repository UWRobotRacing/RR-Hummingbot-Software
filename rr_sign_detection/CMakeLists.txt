cmake_minimum_required(VERSION 2.8.3)
project(rr_sign_detection)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sign_detection src/sign_detection.cpp src/sign_detection_main.cpp)
target_link_libraries(sign_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})