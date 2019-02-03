cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_articulated_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS std_msgs gazebo_plugins actionlib tf image_transport control_msgs trajectory_msgs geometry_msgs sensor_msgs roscpp gazebo_ros robotiq_s_model_articulated_msgs
  util)

find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Contact.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # Mabel: Copied from drcsim/drcsim_gazebo_ros_plugins/CMakeLists.txt
  CATKIN_DEPENDS gazebo_plugins
#  DEPENDS gazebo_plugins gazebo_ros robotiq_s_model_articulated_msgs roscpp
)

# Mabel: Copied from drcsim/drcsim_gazebo_ros_plugins/CMakeLists.txt
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(RobotiqHandPlugin src/RobotiqHandPlugin.cpp)
set_target_properties(RobotiqHandPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(RobotiqHandPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(RobotiqHandPlugin ${catkin_LIBRARIES})
add_dependencies(RobotiqHandPlugin robotiq_s_model_articulated_msgs_gencpp)

add_library(RobotiqHandPlugin_teleport src/RobotiqHandPlugin_teleport.cpp)
set_target_properties(RobotiqHandPlugin_teleport PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(RobotiqHandPlugin_teleport PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(RobotiqHandPlugin_teleport ${catkin_LIBRARIES})
add_dependencies(RobotiqHandPlugin_teleport robotiq_s_model_articulated_msgs_gencpp)

add_library(contact_sensor_plugin src/contact_sensor_plugin.cpp)
target_link_libraries(contact_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(blank_sensor_plugin src/blank_sensor_plugin.cpp)
target_link_libraries(blank_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS RobotiqHandPlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)
install(TARGETS RobotiqHandPlugin_teleport DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)

