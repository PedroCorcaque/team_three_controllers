cmake_minimum_required(VERSION 3.0.2)
project(team_three_controllers)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs krssg_ssl_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
)

install(PROGRAMS
  src/listener_vision.cpp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package()

add_executable(listener_vision src/listener_vision.cpp)
target_link_libraries(listener_vision ${catkin_LIBRARIES})
