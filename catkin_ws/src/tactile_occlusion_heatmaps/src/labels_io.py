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

      try:
        obj_name = label_yaml ['object_name']
      except KeyError:
        print ('ERROR: Key object_name not found in YAML file %s' % (path))
        obj_name = None

      try:
        energy = label_yaml ['grasp_quality']
      except KeyError:
        print ('ERROR: Key grasp_quality not found in YAML file %s' % (path))
        energy = None

      try:
        gpose_yaml = label_yaml ['gripper_pose']
        # Check parameterization of orientation
        if 'q_xyz' in gpose_yaml.keys ():
          # (tx ty tz qx qy qz qw)
          gpose = gpose_yaml ['t_xyz']
          gpose.extend (gpose_yaml ['q_xyz'])
          gpose.append (gpose_yaml ['qw'])

          # (u v tz)
          gpose2d = gpose_yaml ['t_uvz']
          gpose2d.append (gpose_yaml ['t_xyz'] [2])

      except KeyError:
        print ('ERROR: Key gripper_pose not found in YAML file %s' % (path))
        gpose = None

    return obj_name, energy, gpose, gpose2d

