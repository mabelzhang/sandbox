#!/usr/bin/env python

# Mabel Zhang
# 23 Oct 2018
#
# Python counterpart to ../include/tactile_occlusion_heatmaps/labels_io.h.
#   That file writes, this file reads.
#


import yaml


class LabelsIO:

  # File is written by ../include/tactile_occlusion_heatmaps/labels_io.h,
  #   called from ./occlusion_test.py
  @staticmethod
  def read_label (path):

    with open (path, 'rb') as lbl_f:

      label_yaml = yaml.load (lbl_f)

      obj_name = label_yaml ['object_name']
      energy = label_yaml ['grasp_quality']

      gpose_yaml = label_yaml ['gripper_pose']
      # Check parameterization of orientation
      if 'q_xyz' in gpose_yaml.keys ():
        # (tx ty tz qx qy qz qw)
        gpose = gpose_yaml ['t_xyz']
        gpose.extend (gpose_yaml ['q_xyz'])
        gpose.append (gpose_yaml ['qw'])

    return obj_name, energy, gpose

