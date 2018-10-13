#!/bin/bash

# Mabel Zhang
# 12 Oct 2018
#
# First run GraspIt GUI and ROS interface:
#   $ roslaunch graspit_interface_custom graspit_interface.launch 
# Then run this script.
#


# Render scenes
blensor -b -P `rospack find depth_scene_rendering`/src/scene_generation.py

# Generate cropped object images from scenes
rosrun depth_scene_rendering postprocess_scenes

# Collect GraspIt contacts
rosrun grasp_collection grasp_collection.py

# Set GEN_RAND_PTS = false, recompile, run
rosrun tactile_occlusion_heatmaps occlusion_test 

