#!/usr/bin/env python

# Mabel Zhang
# 22 Jan 2019
#
#
#

import os

# Custom
from tactile_occlusion_heatmaps.config_paths import get_data_path


def get_drop_tests_path ():

  path = os.path.join (get_data_path (), 'drop_tests')

  if not os.path.exists (path):
    os.makedirs (path)

  return path

