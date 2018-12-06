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

# Custom
from util.ansi_colors import ansi_colors as ansi


class ConfigReadYAML:

  NAME_IDX = 0
  ID_IDX = 1
  SCENE_IDX = 2

  pkg_path = rospkg.RosPack ().get_path ('depth_scene_rendering')
  config_path = os.path.join (pkg_path, 'config')

  @staticmethod
  def read_scene_paths (scene_base='scenes_noisy.yaml'):

    # List of strings
    obj_names = []
    # List of ints
    obj_ids = []
    # List of list of strings
    scene_paths = []

    scene_list_path = os.path.join (ConfigReadYAML.config_path, scene_base)

    print ('Reading scene paths from %s' % scene_list_path)

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


  # Parameters:
  #   3 parallel lists with corresponding indices:
  #   obj_names: List of strings, may repeat
  #   obj_ids: List of integers, may repeat
  #   scene_paths: List of list of strings
  #   outname: basename of output file
  @staticmethod
  def write_scene_paths (obj_names, obj_ids, scene_paths, outname):

    n_paths = 0

    scene_list_yaml = dict ()
    scene_list_yaml ['objects'] = []

    # For each object name in list
    for lo_i in range (len (obj_names)):

      # Check if this object already exists in the dictionary object to write to YAML
      id_exists = False
      for do_i in range (len (scene_list_yaml ['objects'])):

        # If this object is already in the list, append its scene paths
        # Assumption: 'object' field is already populated. Each object has a
        #   unique ID.
        if scene_list_yaml ['objects'] [do_i] ['id'] == obj_ids [lo_i]:
          scene_list_yaml ['objects'] [do_i] ['scenes'] += scene_paths [lo_i]
          n_paths += len (scene_paths [lo_i])
          id_exists = True
          print ('%s: %d paths' % (obj_names [lo_i], len (scene_paths [lo_i])))
          break

      # If this object is not in the list yet, create it
      if not id_exists:
        obj = dict ()
        obj ['object'] = obj_names [lo_i]
        obj ['id'] = obj_ids [lo_i]
        obj ['scenes'] = scene_paths [lo_i]
        scene_list_yaml ['objects'].append (obj)
        print ('%s: %d paths' % (obj_names [lo_i], len (scene_paths [lo_i])))

        n_paths += len (scene_paths [lo_i])

    # Use prettier block style
    #   Ref https://pyyaml.org/wiki/PyYAMLDocumentation
    yaml_str = yaml.dump (scene_list_yaml, default_flow_style=False)
    #print (yaml_str)

    scene_list_path = os.path.join (ConfigReadYAML.config_path, outname)
    with open (scene_list_path, 'w') as outf:
      outf.write (yaml_str)

    print ('%sWritten scene paths to file %s%s' % (ansi.OKCYAN,
      scene_list_path, ansi.ENDC))

    return n_paths


  # Parameters:
  #   innames: List of strings, basenames of input files
  #   outname: String, basename of output file
  @staticmethod
  def concat_scene_paths (innames, outname):

    # List of lists. Each item in the outer list is all items from a file
    all_obj_names = []
    all_obj_ids = []
    all_scene_paths = []

    for iname in innames:
      objs = ConfigReadYAML.read_scene_paths (iname)
      all_obj_names.extend (objs [ConfigReadYAML.NAME_IDX])
      all_obj_ids.extend (objs [ConfigReadYAML.ID_IDX])
      all_scene_paths.extend (objs [ConfigReadYAML.SCENE_IDX])

    n_paths = ConfigReadYAML.write_scene_paths (all_obj_names, all_obj_ids,
      all_scene_paths, outname)

    print ('%d scene paths from %d files written' % (n_paths, len (innames)))



# 120 per object. 8 objects. Total 960
