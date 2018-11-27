#!/bin/bash

# Mabel Zhang
# 12 Oct 2018
#
# If run grasp_collect.py (only need to run ONCE) or grasp_replay.py (for
#   debugging), first run GraspIt GUI and ROS interface:
#   $ roslaunch graspit_interface_custom graspit_interface.launch 
# Then run this script.
#


MAGENTA="\e[95m"
ENDC="\e[0m"


# Render scenes
# Change n_camera_poses to change the number of scenes generated per object
cmd="blensor -b -P `rospack find depth_scene_rendering`/src/scene_generation.py"
echo -e "$MAGENTA$cmd$ENDC"
$cmd

# Generate cropped object images from scenes
cmd="rosrun depth_scene_rendering postprocess_scenes"
echo -e "$MAGENTA$cmd$ENDC"
$cmd

# NOTE: Only need to run ONCE for all objects
#   This doesn't need to be rerun all the time. It only needs to be run once,
#   then used for all rendered scenes thereafter.
# Collect GraspIt contacts
#cmd="rosrun grasp_collection grasp_collect.py"
#echo -e "$MAGENTA$cmd$ENDC"
#$cmd

# Set GEN_RAND_PTS = false, recompile, run
cmd="rosrun tactile_occlusion_heatmaps occlusion_test"
echo -e "$MAGENTA$cmd$ENDC"
$cmd
# Flags available:
# rosrun tactile_occlusion_heatmaps occlusion_test [--display] [--vis]
#   [--scale-heatmaps] [--object_i #] [--grasp_i #] [--scene_i #]



# (Optional) Visualize visible and occluded contact heat maps:
#cmd="rosrun tactile_occlusion_heatmaps visualize_heatmaps.py --display"
#cmd="rosrun tactile_occlusion_heatmaps visualize_heatmaps.py"
#echo -e "$MAGENTA$cmd$ENDC"
#$cmd

# (Optional) At the same time, visualize GraspIt saved grasps:
# $ rosrun grasp_collection grasp_replay.py


# Generate npz files
#   Flags available: --scale-heatmaps
#cmd="rosrun tactile_occlusion_heatmaps png_to_npz.py"
#echo -e "$MAGENTA$cmd$ENDC"
#$cmd

