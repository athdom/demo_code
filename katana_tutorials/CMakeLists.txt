cmake_minimum_required(VERSION 2.8.3)
project(katana_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  tf
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follow_online_trajectory src/follow_online_trajectory.cpp)
target_link_libraries(follow_online_trajectory ${catkin_LIBRARIES})
add_dependencies(follow_online_trajectory ${catkin_EXPORTED_TARGETS})

add_executable(gripper_position_pub src/gripper_position_pub.cpp)
target_link_libraries(gripper_position_pub ${catkin_LIBRARIES})
add_dependencies(gripper_position_pub ${catkin_EXPORTED_TARGETS})


install(TARGETS follow_online_trajectory
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
