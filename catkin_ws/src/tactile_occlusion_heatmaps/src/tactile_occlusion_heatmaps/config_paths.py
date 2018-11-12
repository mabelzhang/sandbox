#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
#
#


import os

import rospkg


def get_depth_fmt ():

  return '%scrop.png'


def get_heatmap_raw_fmt ():

  # %s is timestamp from depth_scene_rendering scene_generation.py
  # g%d is grasp number from grasp_collection grasp_collect.py
  return ('%s_g%d_vis.png', '%s_g%d_occ.png')


def get_heatmap_blob_fmt ():

  return ('%s_g%d_vis_blob.png', '%s_g%d_occ_blob.png')


def get_label_fmt ():

  return '%s_g%d_lbls.yaml'


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


# Subdir of data/
def get_renders_data_path ():

  path = os.path.join (get_data_path (), 'renders')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


# Subdir of data/
def get_heatmaps_data_path ():

  path = os.path.join (get_data_path (), 'heatmaps')

  if not os.path.exists (path):
    os.makedirs (path)

  return path



def get_vis_path ():

  path = os.path.join (get_root (), 'vis')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


def get_vis_heatmap_fmt ():

  # String is scene_path of the .pcd rendered scene, i.e.
  #   os.path.splitext (os.path.basename (scene_path)) [0]
  # Integer is grasp number from grasp_collect.py, g_i
  return ('%s_g%d.png')


def get_vis_3d_fmt ():

  # String is scene_path of the .pcd rendered scene, i.e.
  #   os.path.splitext (os.path.basename (scene_path)) [0]
  # Integer is grasp number from grasp_collect.py, g_i
  return ('%s_g%d_3d.png')


# NOTE: Called by scene_generation.py, which runs in Blender Python.
#   Do not use libraries not in Blender Python, e.g. PyYAML.
def get_cam_config_path ():

  pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
  return os.path.join (pkg_path, "config/cam_config_path.txt")

