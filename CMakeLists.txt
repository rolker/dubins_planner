cmake_minimum_required(VERSION 2.8.3)
project(dubins_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  project11_navigation
  pluginlib
  roscpp
  tf2
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES dubins_planner
  CATKIN_DEPENDS pluginlib project11_navigation roscpp tf2
  DEPENDS OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


set(PLUGIN_SOURCES
  src/dubins_planner.cpp
)

add_library(dubins_planner ${PLUGIN_SOURCES})
add_dependencies(dubins_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dubins_planner
    ${catkin_LIBRARIES}
    project11_navigation
    ${OMPL_LIBRARIES}
    )

install(TARGETS dubins_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
