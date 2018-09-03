#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018

import sys
import os


def get_data_root ():

  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))

  # Set where you desire to place all output data
  data_root = os.path.realpath (os.path.join (this_dir, '../../../../../train/visuotactile_grasping'))

  if not os.path.exists (data_root):
    os.makedirs (data_root)

  return data_root


