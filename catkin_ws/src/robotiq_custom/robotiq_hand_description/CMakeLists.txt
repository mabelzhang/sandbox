cmake_minimum_required(VERSION 2.8.3)
project(robotiq_hand_description)

# Used by launch
set(ROS_PACKAGE_NAME robotiq_hand_description)

find_package(catkin REQUIRED)

catkin_package(
)

# Mabel: I don't need this
#set(standalone_name "drcsim_gazebo_standalone_models")

# Mabel: I don't need this
#install(DIRECTORY urdf meshes robots launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# Mabel: I don't need this
#install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${standalone_name}
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/share
#)

# Mabel: I don't need this
# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
#install(FILES
#  model.config
#  dummy.urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# Mabel: I don't need this
# In addition to the usual installation, which happens above, we'll
# now do some special processing and installation to provide standalone
# versions of the models that can be used without ROS (or even Gazebo).
#include(${CMAKE_CURRENT_SOURCE_DIR}/../tools/URDF_helpers.cmake)
#make_standalone_models(robotiq_hand_description robotiq_hand_description ${standalone_name})

# Mabel: I don't need this
# Copy all the meshes and materials into standalone_models/robotiq_hand_description
#install(DIRECTORY
#  meshes
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${standalone_name}/robotiq_hand_description
#)
