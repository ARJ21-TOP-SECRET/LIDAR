cmake_minimum_required(VERSION 2.8.3)
project(path_navi_client)

find_package(catkin REQUIRED
  COMPONENTS
  nav_core
  nav_msgs
  roscpp
  actionlib_msgs
  tf
  actionlib
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES
  PathNavi.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs nav_msgs geometry_msgs # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  nav_msgs
  roscpp
  actionlib_msgs
)

#add_library(PointNavi src/path_navi_client.cpp src/icp.cpp)
add_executable(path_navi_client src/path_navi_client.cpp src/icp.cpp)

target_link_libraries(path_navi_client ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so)


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS path_navi_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS path_navi_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch action msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

