cmake_minimum_required(VERSION 2.8.12)

project(costmap_based_local_planner)

#find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  grid_map_ros
  object_map
  pcl_ros
  roscpp
  tf
  vector_map
  lanelet2_extension
  op_planner
  op_ros_helpers
  op_utility	
)


set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++1z -DNDEBUG")

find_package(Boost REQUIRED COMPONENTS system)


#CUDA setting
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE STATIC)
set(CUDA_KERNEL_LIB CollisionCheck)
cuda_add_library(${CUDA_KERNEL_LIB} ${LIB_TYPE} src/collision_check_wth_path/collision_check.cu)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  collision_check_with_path
  src/collision_check_wth_path/collision_check_with_path.cpp
)

add_executable(collision_check_with_path_node 
src/collision_check_wth_path/collision_check_with_path_node.cpp)

add_dependencies(collision_check_with_path_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
   ${CUDA_KERNEL_LIB}
)

target_link_libraries(collision_check_with_path_node
   collision_check_with_path
   ${catkin_LIBRARIES}
   ${CUDA_KERNEL_LIB}
)


add_library(
  behavior_planner
  src/behavior_planner/behavior_planner.cpp
)

add_executable(behavior_planner_node 
src/behavior_planner/behavior_planner_node.cpp)

add_dependencies(behavior_planner_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
   ${CUDA_KERNEL_LIB}
)

target_link_libraries(behavior_planner_node
   behavior_planner
   ${catkin_LIBRARIES}
   ${CUDA_KERNEL_LIB}
)


add_library(
  trajectory_generate
  src/trajectory_generate/trajectory_generate.cpp
)

add_executable(trajectory_generate_node 
src/trajectory_generate/trajectory_generate_node.cpp)

add_dependencies(trajectory_generate_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trajectory_generate_node
   trajectory_generate
   ${catkin_LIBRARIES}
)


install(
  TARGETS 
    collision_check_with_path_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS 
    behavior_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS 
  trajectory_generate_node
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

