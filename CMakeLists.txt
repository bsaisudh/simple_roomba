cmake_minimum_required(VERSION 2.8.3)
project(simple_roomba)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()



## Build simple roomba
include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(simple_roomba 
		src/simple_roomba_node.cpp 
		src/RoombaMove.cpp
		)
target_link_libraries(simple_roomba ${catkin_LIBRARIES})
