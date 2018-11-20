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

  NAME_IDX = 0
  ID_IDX = 1
  SCENE_IDX = 2

  @staticmethod
  def read_object_names ():

    # List of strings
    obj_names = []
    # List of ints
    obj_ids = []
    # List of list of strings
    scene_paths = []

    pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
    scene_list_path = os.path.join (pkg_path, "config/scenes_noisy.yaml")

    with open (scene_list_path, 'rb') as scene_list_f:
      scene_list_yaml = yaml.load (scene_list_f)

      for o_i in range (len (scene_list_yaml ['objects'])):
     
        obj = scene_list_yaml ['objects'] [o_i]

        # String
        obj_names.append (obj ['object'])
        # Int
        obj_ids.append (obj ['id'])

        # List of strings
        scene_paths.append (obj ['scenes'])

    retval = [None] * 3
    retval [ConfigReadYAML.NAME_IDX] = obj_names
    retval [ConfigReadYAML.ID_IDX] = obj_ids
    retval [ConfigReadYAML.SCENE_IDX] = scene_paths
    return retval


