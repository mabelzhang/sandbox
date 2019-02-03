#!/usr/bin/env python

# Mabel Zhang
# 1 Oct 2018
#
# Constants for GraspIt grasp planning.
#

# GraspIt world files in GraspIt installation path worlds/ to load
# Use this instead of YAML in depth_scene_rendering, `.` grasps don't need to
#   be re-generated all the time! Only need to generate once! Grasps are the
#   same for any camera angle.
worlds = [
  'dexnet/bar_clamp',
  'dexnet/gearbox',
  'dexnet/nozzle',
  'dexnet/part1',
  'dexnet/part3',
  'dexnet/pawn',
  'dexnet/turbine_housing',
  'dexnet/vase']

def world_to_object_base (world):
  return world.split ('/') [1]


# Quality measure for GraspIt planning.
# Default quality measure: search_energy="GUIDED_POTENTIAL_QUALITY_ENERGY"
#   Defined in $GRASPIT/src/EGPlanner/energy/searchEnergyFactory.cpp
# Key: Constant for calling GraspIt
# Value: Abbreviation for .npz label files' prefix for ML predictor
energies_abbrevs = {
  'CONTACT_ENERGY': 'cte',  # 0
  'POTENTIAL_QUALITY_ENERGY': 'pqe',  # 1
  'AUTO_GRASP_QUALITY_ENERGY': 'agqe',  # 2
  'GUIDED_POTENTIAL_QUALITY_ENERGY': 'gpqe',  # 3. Default
  'GUIDED_AUTO_GRASP_QUALITY_ENERGY': 'gagqe',  # 4
  'STRICT_AUTO_GRASP_ENERGY': 'sage',  # 5
  'COMPLIANT_ENERGY': 'cge',  # 6
  'DYNAMIC_AUTO_GRASP_ENERGY': 'dage',  # 7
}

# Select the search energy to use
SEARCH_ENERGY = 'GUIDED_POTENTIAL_QUALITY_ENERGY'
ENERGY_ABBREV = energies_abbrevs [SEARCH_ENERGY]

# Grasps with energy above this value can be skipped when writing training data
#   in tactile_occlusion_heatmaps png_to_npz.py.
# Rough observation from gpqe. Positive energy definitely bad.
THRESH_INVALID_ENERGY = 0


# Number of pose parameters for different orientation parameterizations
# Quaternions (3 translational + 4 orientational)
N_PARAMS_QUAT = 7
# Select the parameteraization to use for gripper pose
N_POSE_PARAMS = N_PARAMS_QUAT


