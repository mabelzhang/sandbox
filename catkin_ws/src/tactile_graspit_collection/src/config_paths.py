#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
#
#


import os


def get_heatmap_raw_fmt ():

  return ('%s_vis.png', '%s_occ.png')


def get_heatmap_blob_fmt ():

  return ('%s_vis_blob.png', '%s_occ_blob.png')


def get_vis_path ():

  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))

  # Set where you desire to place all output data
  data_root = os.path.realpath (os.path.join (this_dir, '../../../../../train/visuotactile_grasping/vis'))

  if not os.path.exists (data_root):
    os.makedirs (data_root)

  return data_root


