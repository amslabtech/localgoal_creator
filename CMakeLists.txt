cmake_minimum_required(VERSION 3.0.2)
project(localgoal_creator)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
amsl_navigation_msgs
std_msgs
geometry_msgs
nav_msgs
)

catkin_package(
INCLUDE_DIRS include
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_node src/localgoal_creator_node.cpp src/localgoal_creator.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
