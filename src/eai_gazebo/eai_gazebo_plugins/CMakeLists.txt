cmake_minimum_required(VERSION 2.8.3)
project(eai_gazebo_plugins)

find_package(gazebo REQUIRED)


find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        gazebo_plugins
                                        geometry_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES gazebo_ros_dashgo
               CATKIN_DEPENDS gazebo_ros
                              gazebo_plugins
                              geometry_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_dashgo src/gazebo_ros_dashgo.cpp 
                              src/gazebo_ros_dashgo_updates.cpp 
                              src/gazebo_ros_dashgo_loads.cpp)
add_dependencies(gazebo_ros_dashgo ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_dashgo
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_dashgo
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
