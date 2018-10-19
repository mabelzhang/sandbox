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

# NOTE: only need to run ONCE for all objects
#   This doesn't need to be rerun all the time. It only needs to be run once,
#   then used for all rendered scenes thereafter.
# Collect GraspIt contacts
#rosrun grasp_collection grasp_collect.py

# Set GEN_RAND_PTS = false, recompile, run
rosrun tactile_occlusion_heatmaps occlusion_test 



# Visualize visible and occluded contact heat maps:
# $ rosrun tactile_occlusion_heatmaps visualize_heatmaps.py

# At the same time, visualize GraspIt saved grasps:
# $ rosrun grasp_collection grasp_replay.py


