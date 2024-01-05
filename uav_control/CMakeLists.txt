cmake_minimum_required(VERSION 2.8.3)
project(uav_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES uav_lee_position_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs mavros_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(uav_lee_position_controller
  src/library/lee_position_controller.cpp
)

target_link_libraries(uav_lee_position_controller ${catkin_LIBRARIES})
add_dependencies(uav_lee_position_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  uav_lee_position_controller ${catkin_LIBRARIES})

install(TARGETS uav_lee_position_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lee_position_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 这三个install的区别是什么啊

# 可能跟install的位置有关系
# liblee_position_controller.so => /home/scarlett/Workspaces/rotors_integration_ws/devel/lib/liblee_position_controller.so (0x00007fed7dcff000)
# liblee_position_controller.so => /home/scarlett/Workspaces/rotors_integration_ws/devel/lib/liblee_position_controller.so (0x00007f1721906000)