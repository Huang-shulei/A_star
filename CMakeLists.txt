cmake_minimum_required(VERSION 3.0.2)
project(a_star)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
)

find_package(OpenCV 4.5.3 REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a_star
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(a_star_plus src/a_star_plus.cpp)
target_link_libraries(a_star_plus ${OpenCV_LIBS} ${catkin_LIBRARIES} )
add_library(a_star SHARED src/a_star.cpp)
target_link_libraries(a_star_plus ${OpenCV_LIBS} ${catkin_LIBRARIES} )
target_link_libraries(a_star_plus a_star)
