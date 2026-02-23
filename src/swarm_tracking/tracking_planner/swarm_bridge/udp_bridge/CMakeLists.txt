cmake_minimum_required(VERSION 2.8.3)
project(udp_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        roslib
        rosbag
        swarm_msgs
        traj_opt
        )
#
#add_message_files(
#        FILES
#        QuadStatePub.msg
#        ObserveTeammate.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        geometry_msgs
#)

catkin_package(
        CATKIN_DEPENDS traj_opt
)

include_directories(
        ../../../devel/include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

add_executable(udp_unicast
src/udp_unicast.cpp
)
target_link_libraries(udp_unicast
${catkin_LIBRARIES}
)

