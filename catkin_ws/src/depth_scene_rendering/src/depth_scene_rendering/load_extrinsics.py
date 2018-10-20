#!/usr/bin/env python

# Mabel Zhang
# 19 Oct 2018
#
#

import numpy as np

# Custom
from util.tf_transformations import euler_matrix


# Loads extrinsics matrix saved by scene_generation.py
def load_extrinsics (extrinsics_path):

  T_o_cam = np.loadtxt (extrinsics_path)
  return T_o_cam


def compensate_blender_to_graspit (extrinsics):

  # Graphics to robotics flip, pi wrt x. Flips z from out of frame to into frame
  flip_z = euler_matrix (np.pi, 0, 0, 'sxyz')

  m = np.dot (extrinsics, flip_z)


  # Rotate 180 wrt z, to negate x and y coords. occlusion_test.cpp does this
  #flip_xy = euler_matrix (0, 0, np.pi, 'sxyz')
  ##flip_xy = euler_matrix (0, np.pi, 0, 'sxyz')

  ##m = np.dot (flip_xy, m)
  #m = np.dot (m, flip_xy)



  return m

