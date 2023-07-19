cmake_minimum_required(VERSION 3.0.2)
project(modbus_communication)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath=/usr/lib/gcc/x86_64-linux-gnu/9")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# find_package(Open3D REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# include_directories(${PCL_INCLUDE_DIRS})
add_executable(modbus_test src/modbus_test.cpp)
target_compile_options(modbus_test PRIVATE -lmodbus)
target_include_directories(modbus_test PRIVATE /usr/include/modbus)
target_link_libraries(modbus_test
  modbus
  ${catkin_LIBRARIES}
)

