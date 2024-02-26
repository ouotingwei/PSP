cmake_minimum_required(VERSION 3.5)
project(communication)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath=/usr/lib/gcc/x86_64-linux-gnu/9")

find_library(LIBMODBUS_LIBRARY NAMES modbus)
if(NOT LIBMODBUS_LIBRARY)
    message(FATAL_ERROR "libmodbus library not found")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_service_files(
  FILES
  Ftp.srv
  ModbusPLC.srv
  ModbusRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS roscpp
)

# include_directories(${PCL_INCLUDE_DIRS})
add_executable(passLS_srv src/passLS_srv.cpp)
target_link_libraries(passLS_srv
  ${catkin_LIBRARIES}
)

add_executable(modbus_robot_srv src/modbus_robot_srv.cpp)
target_link_libraries(modbus_robot_srv
    ${catkin_LIBRARIES}
    ${LIBMODBUS_LIBRARY}  
)

add_executable(modbus_plc_srv src/modbus_plc_srv.cpp)
target_link_libraries(modbus_plc_srv
    ${catkin_LIBRARIES}
    ${LIBMODBUS_LIBRARY}  
)


