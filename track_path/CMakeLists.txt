cmake_minimum_required(VERSION 3.0.2)
project(track_path)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        include/track_path/track_path.h
        src/track_path.cpp
        src/track_path_node.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
