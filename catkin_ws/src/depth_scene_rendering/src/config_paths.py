#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Paths that are not constants
#

import sys
import os


# NOTE: Called by scene_generation.py, which runs in Blender Python.
#   Do not use libraries not in Blender Python, e.g. PyYAML.
def get_data_root ():

  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))

  # Set where you desire to place all output data
  data_root = os.path.realpath (os.path.join (this_dir, '../../../../../train/visuotactile_grasping/data'))

  if not os.path.exists (data_root):
    os.makedirs (data_root)

  return data_root


# NOTE: Called by scene_generation.py, which runs in Blender Python.
#   Do not use libraries not in Blender Python, e.g. PyYAML.
def get_intrinsics_path ():
  return os.path.join (get_data_root (), 'intrinsics.txt')

def get_depth_range_path ():
  return os.path.join (get_data_root (), 'cam_depth_range.txt')


