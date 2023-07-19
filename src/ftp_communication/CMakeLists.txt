cmake_minimum_required(VERSION 3.0.2)
project(ftp_communication)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath=/usr/lib/gcc/x86_64-linux-gnu/9")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# include_directories(${PCL_INCLUDE_DIRS})
add_executable(passLS src/passLS.cpp)
target_link_libraries(passLS
  ${catkin_LIBRARIES}
)

