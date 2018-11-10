#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Paths that are not constants
#
# Script to be run in Blender. Do not uses packages like YAML that aren't in
#   Blender Python.
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


def get_render_data_path ():

  path = os.path.join (get_data_root (), 'renders')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


# NOTE: Called by scene_generation.py, which runs in Blender Python.
#   Do not use libraries not in Blender Python, e.g. PyYAML.
def get_intrinsics_path ():
  return os.path.join (get_render_data_path (), 'intrinsics.txt')

def get_depth_range_path ():
  return os.path.join (get_render_data_path (), 'cam_depth_range.txt')


# Used by render_camera_poses.py and crop_images.py to determine range of
#   extrinsics camera poses to use, so that there is a variety of poses, and
#   object is still in camera view.
def get_test_render_path ():

  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))

  # Set where you desire to place all output data
  path = os.path.realpath (os.path.join (this_dir, '../../../../../train/visuotactile_grasping/test_renders'))

  if not os.path.exists (path):
    os.makedirs (path)

  return path


