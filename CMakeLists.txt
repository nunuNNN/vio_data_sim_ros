cmake_minimum_required(VERSION 2.8.3)
project(vio_data_sim_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vio_data_sim_ros
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_node 
              src/vio_data_sim_ros_node.cpp
              src/param.cpp 
              src/utilities.cpp
              src/imu.cpp
)

target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
)
