cmake_minimum_required(VERSION 2.8.12)

project(voronoi_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  pcl_conversions
  ackermann_msgs
  cv_bridge
)

#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

add_library(
  voronoi_planner_lib
  src/voronoi_planner.cpp
)

add_dependencies(voronoi_planner_lib 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(voronoi_planner_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}

)

add_executable(voronoi_planner_node 
src/voronoi_planner_node.cpp)

add_dependencies(voronoi_planner_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(voronoi_planner_node
   voronoi_planner_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}

)

link_directories(
	include
  ${PCL_LIBRARY_DIRS}
)

install(
  TARGETS 
  voronoi_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

