#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
#
#


import os

import rospkg


def get_heatmap_raw_fmt ():

  return ('%s_vis.png', '%s_occ.png')


def get_heatmap_blob_fmt ():

  return ('%s_vis_blob.png', '%s_occ_blob.png')


def get_root ():

  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))

  # Set where you desire to place all output data
  data_root = os.path.realpath (os.path.join (this_dir, '../../../../../../train/visuotactile_grasping'))

  if not os.path.exists (data_root):
    os.makedirs (data_root)

  return data_root


def get_data_path ():

  path = os.path.join (get_root (), 'data')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


def get_vis_path ():

  path = os.path.join (get_root (), 'vis')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


# NOTE: Called by scene_generation.py, which runs in Blender Python.
#   Do not use libraries not in Blender Python, e.g. PyYAML.
def get_cam_config_path ():

  pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
  return os.path.join (pkg_path, "config/cam_config_path.txt")

