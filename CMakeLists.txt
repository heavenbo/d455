cmake_minimum_required(VERSION 3.0.2)
project(D455_r_ros)

# set(OpenCV_DIR ~/opencv/build)
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  cv_bridge
  tf
  image_transport
)

# list(APPEND thirdparty_libraries
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

catkin_package(
)
include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)
add_executable(d455_ros src/d455_ros.cpp)
target_link_libraries(d455_ros ${catkin_LIBRARIES})

# add_executable(d455 src/d455.cpp)
# target_link_libraries(d455 ${catkin_LIBRARIES})
