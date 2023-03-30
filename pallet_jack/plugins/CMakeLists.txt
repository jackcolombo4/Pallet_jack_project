cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
		    
link_directories(${GAZEBO_LIBRARY_DIRS})
		 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(laser_plugin SHARED LaserPlugin.cc)
add_library(lift_plugin SHARED LiftPlugin.cc)

target_link_libraries(laser_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
target_link_libraries(lift_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} ${Boost_LIBRARIES})

