cmake_minimum_required(VERSION 2.8.3)
project(alan)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wpedantic -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_executable(pick_place_test
  src/pick_place_test.cpp
)
target_link_libraries(pick_place_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
