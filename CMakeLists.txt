cmake_minimum_required(VERSION 3.0.2)
project(mpc_path_tracking)

add_compile_options(-std=c++11 -D_GLIBCXX_USE_CXX11_ABI=1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  tf
  sensor_msgs
  std_msgs
  waypoint_maker
)

find_package(Eigen3 REQUIRED)

set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
set(IPOPT_LIBRARY_DIRS /usr/local/lib)

include_directories(
  ./include/mpc_path_tracking
  ./include/mpc
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
  /usr/local/include
  /usr/include
  /usr/include/cppad
)


link_directories(
  ${IPOPT_LIBRARY_DIRS}
  /usr/local/lib
  /usr/lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_path_tracking
  LIBRARIES mpc
  CATKIN_DEPENDS ackermann_msgs geometry_msgs message_generation message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs waypoint_maker
)

add_executable(mpc_path_tracking
  src/main.cpp
  src/mpc_path_tracking.cpp
  src/mpc.cpp
  src/ros_control.cpp
)

find_library(CPPAD_LIB cppad_lib PATHS /usr/local/lib)

target_link_libraries(mpc_path_tracking ${catkin_LIBRARIES} ${CPPAD_LIB} ipopt)
