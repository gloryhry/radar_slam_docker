cmake_minimum_required(VERSION 3.0.2)
project(radar_canfd_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(radar_canfd_node
  src/radar_canfd_node.cpp
  src/ars620_decoder.cpp
  src/cycle_assembler.cpp
  src/pointcloud.cpp
)

add_dependencies(radar_canfd_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(radar_canfd_node
  ${catkin_LIBRARIES}
  dl
  pthread
)

# Point cloud merger (standalone, no hardware dependency)
add_executable(radar_merger_node
  src/radar_merger_node.cpp
)
add_dependencies(radar_merger_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(radar_merger_node
  ${catkin_LIBRARIES}
)

install(TARGETS radar_canfd_node radar_merger_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib
)
