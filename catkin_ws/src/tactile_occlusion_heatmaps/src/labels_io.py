#!/usr/bin/env python

# Mabel Zhang
# 23 Oct 2018
#
# Python counterpart to ../include/tactile_graspit_collection/labels_io.h.
#   That file writes, this file reads.
#


import yaml


class LabelsIO:

  # File is written by ../include/tactile_graspit_collection/labels_io.h,
  #   called from ./occlusion_test.py
  @staticmethod
  def read_label (path):

    with open (path, 'rb') as lbl_f:

      label_yaml = yaml.load (lbl_f)

      obj_name = label_yaml ['object_name']
      quality = label_yaml ['grasp_quality']

    return obj_name, quality

