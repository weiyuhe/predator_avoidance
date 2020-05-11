cmake_minimum_required(VERSION 2.8.3)
project(mcl_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS message_filters roscpp sensor_msgs tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(run_mcl src/main.cpp)
target_link_libraries(run_mcl ${catkin_LIBRARIES})

add_library(kdtree src/kdtree.cpp)
add_library(get_map src/get_map.cpp)
add_library(mclcpp src/mcl.cpp)
add_library(mclsub src/mcl_sub.cpp)

target_link_libraries(mclsub ${catkin_LIBRARIES})
target_link_libraries(mclcpp ${catkin_LIBRARIES})
target_link_libraries(get_map kdtree)
target_link_libraries(mclcpp get_map)
target_link_libraries(mclsub mclcpp)
target_link_libraries(run_mcl kdtree)
target_link_libraries(run_mcl get_map)
target_link_libraries(run_mcl mclcpp)
target_link_libraries(run_mcl mclsub)

