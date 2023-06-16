cmake_minimum_required(VERSION 3.0.2)
project(downSampleAndMerge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES downSampleAndMerge
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# Add compile options

add_definitions(${PCL_DEFINITIONS})

# Declare catkin package
catkin_package()

# Build executable
add_executable(downSampleAndMerge src/downSampleAndMerge.cpp)

# Link libraries
target_link_libraries(downSampleAndMerge
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
