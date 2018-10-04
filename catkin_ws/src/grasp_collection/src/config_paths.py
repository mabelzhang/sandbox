#!/usr/bin/env python

# Mabel Zhang
# 4 Oct 2018
#
#
#

import os

# Custom
from tactile_occlusion_heatmaps.config_paths import get_data_root


def get_grasps_path ():

  path = os.path.join (get_data_root (), 'grasps')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


def get_contacts_path ():

  path = os.path.join (get_data_root (), 'contacts')

  if not os.path.exists (path):
    os.makedirs (path)

  return path


