#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Entry point.
#
# Usage:
#   $ blensor -P scene_generation.py
#


import sys
# Add current directory to path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')
# Add utilities
sys.path.append ('/media/master/Data_Ubuntu/courses/research/github_repos/clean_repos/catkin_ws/src/util/util/src')


# Local
from scan_kinect import scan

scan ()


'''
def create_scene ():

  # Don't do this. It closes the console as well!
  # Reset scene
  #bpy.ops.wm.read_homefile ()

  # TODO: Create a custom scene file with a flat plane and objects on it, just
  #   always load that scene, so don't have to delete the Cube every time.


  # Don't need, already in BlenSor's default scene setup
  # Create a Kinect object
  # API https://docs.blender.org/api/blender_python_api_current/bpy.ops.object.html
  #bpy.ops.object.camera_add ()

  pass
'''
 

