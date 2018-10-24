#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Manually configured, input object meshes for scene rendering.
#
# Defining constants in Python, because Blender does not bundle PyYaml. It
#   would need to be installed manually into Blender 
#   blender/2.74/python/lib/python3.4/.
#
# Ref: https://blender.stackexchange.com/questions/5287/using-3rd-party-python-modules
#

obj_path = '/home/master/graspingRepo/train/dexnet/mini_dexnet'
objects = [
  # Rotated in Blender manually to align to axis
  'bar_clamp_rotated_centered.obj',
  'gearbox_rotated_centered.obj',
  'nozzle_rotated_centered.obj',
  'part1_rotated_centered.obj',
  'part3_rotated_centered.obj',
  'pawn_rotated_centered.obj',
  'turbine_housing_rotated_centered.obj',
  'vase_rotated_centered.obj']
  #'climbing_hold.obj',
  #'endstop_holder.obj',
  #'mount1.obj',
  #'mount2.obj',
  #'pipe_connector.obj',

# objects [i].replace(suffix, ''), to get bare object name
obj_suffix = '_rotated_centered.obj'


# Used to output a plain-text labels file for ease of access for predictor
obj_names = [o.replace (obj_suffix, '') for o in objects]
obj_ids = range (len (objects))

