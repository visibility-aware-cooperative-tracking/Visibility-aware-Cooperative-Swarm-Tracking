cmake_minimum_required(VERSION 2.8.3)
project(st_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
  tf
  swarm_msgs
  rog_map
  ciri
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES st_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils rog_map ciri
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(st_planner_node
  src/st_planner_node.cpp 
  src/st_replan_fsm.cpp
  src/SFCgen_ciri.cpp
)
target_link_libraries(st_planner_node 
  ${catkin_LIBRARIES}
  )

  add_executable(target_planner_node
  src/target_planner_node.cpp 
  src/target_replan_fsm.cpp
  src/SFCgen_ciri.cpp
)
target_link_libraries(target_planner_node 
  ${catkin_LIBRARIES}
  )

  add_executable(ssdf_example_node
  src/ssdf_example_node.cpp
)
target_link_libraries(ssdf_example_node
  ${catkin_LIBRARIES}
)


add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

