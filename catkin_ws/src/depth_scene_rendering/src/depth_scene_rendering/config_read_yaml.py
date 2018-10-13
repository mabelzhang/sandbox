#!/usr/bin/env python

# Mabel Zhang
# 12 Oct 2018
#
# Read YAML config file written manually in text format from Blender Python
#   script scene_generation.py. Do not write using Python YAML, `.` Blender
#   Python doesn't have YAML.
#


import os
import yaml

import rospkg


class ConfigReadYAML:

  @staticmethod
  def read_object_names ():

    obj_names = []

    pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
    scene_list_path = os.path.join (pkg_path, "config/scenes_noisy.yaml")

    with open (scene_list_path, 'rb') as scene_list_f:
      scene_list_yaml = yaml.load (scene_list_f)

      for o_i in range (len (scene_list_yaml ['objects'])):
     
        obj = scene_list_yaml ['objects'] [o_i]
        obj_names.append (obj ['object'])

    return obj_names


