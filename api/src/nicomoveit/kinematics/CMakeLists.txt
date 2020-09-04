cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  message_generation
  eigen_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_message_files(
  FILES
  FloatList.msg
)

add_service_files(
  FILES
  FK_request.srv
  IK_request.srv
  collision_check.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

add_executable(ikTest src/ikTest.cpp)
target_link_libraries(ikTest ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(kinematics_server src/kinematics_server.cpp)
target_link_libraries(kinematics_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(kinematics_server ${kinematics_EXPORTED_TARGETS})

install(TARGETS ikTest DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
