#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Entry point.
#
# Usage:
#   $ blensor -P scene_generation.py
#   To run headless, add -b.
#   $ blensor -b -P scene_generation.py
#
#   To load a blender scene from file:
#   $ blender -b -P scene_generation.py
#


import sys
# Add current directory to path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')
# Add utilities
sys.path.append ('/media/master/Data_Ubuntu/courses/research/github_repos/clean_repos/catkin_ws/src/util/util/src')

# Python
import os
import time

# Blender
import bpy

# Local, from paths added above
from scan_kinect import scan
#from util.yaml_util import load_yaml
import config_objects



# Clear current scene with whatever is in it, set up our scene
def reset_scene ():

  # Default blensor scene contains a Lamp, a Camera, and a Cube with cutouts.
  # Remove the Cube, add a Plane, and move the Camera to point at the Plane.


  # Deselect all objects
  # Get a list of objects in the scene
  for obj in bpy.context.selectable_objects:
    obj.select = False
  # Select Cube in default scene
  bpy.data.objects ['Cube'].select = True
  # Delete all selected objects
  bpy.ops.object.delete ()

  # Some line here seg faults. Dont need it. Just don't delete the camera.
  '''
  # Create a camera
  # API https://docs.blender.org/api/blender_python_api_current/bpy.ops.object.html
  bpy.ops.object.camera_add ()

  # Create a plane
  bpy.ops.mesh.primitive_plane_add ()

  # Create a light
  bpy.ops.object.lamp_add (type='POINT')
  # Lamp pose copied from default scene
  bpy.data.objects ['Point'].location.x = 4.43032
  bpy.data.objects ['Point'].location.y = -4.07725
  bpy.data.objects ['Point'].location.z = 3.17647
  bpy.data.objects ['Point'].rotation_quaternion.w = 0.571
  bpy.data.objects ['Point'].rotation_quaternion.x = 0.169
  bpy.data.objects ['Point'].rotation_quaternion.y = 0.272
  bpy.data.objects ['Point'].rotation_quaternion.z = 0.756
  '''


# Load OBJ file into Blender
# Parameters:
#   obj_name: Full path to CAD file
def load_obj (obj_name):

  #bpy.ops.import_scene.obj (filepath=obj_name, axis_forward='Y', axis_up='Z')
  bpy.ops.import_scene.obj (filepath=obj_name)



# Using PyYAML, which isn't available in Blender
# Get directory of current file
#this_dir = os.path.dirname (os.path.realpath (__file__))
# Concatenate config file's relative path
#config_path = os.path.realpath (os.path.join (this_dir, '..', 'config', 'objects.yaml'))
# Load YAML config file of what object CAD files to load into scene
#config_dict = load_yaml (config_path)
#obj_path = config_dict.path




obj_path = config_objects.path

reset_scene ()

# Set stationary camera pose
# TODO: Randomize camera pose on lat/long. Use spherical coordinates to
#   calculate position. Randomize orientation as well.
cam_pos = (0.0, 0.0, 1.0)
# Blender quaternion has (w, x, y, z), w first.
cam_quat = (1, 0, 0, 0)

# Loop through each object file
#for i in range (len (config_objects.objects)):
for i in range (1):

  print ('Loading file %d out of %d' % (i+1, len (config_objects.objects)))

  obj_base = config_objects.objects [i]
  obj_name = os.path.join (obj_path, obj_base)

  load_obj (obj_name)

  # TODO: Select only the object loaded.
  #bpy.ops.object.delete ()


  scan (cam_pos, cam_quat)

# This doesn't quit Blender safely. It always prints mem err before quitting:
#   Error: Not freed memory blocks: 216, total unfreed memory 0.019569 MB
# But manually closing blender doesn't have this issue. So will just manually
#   close it.
#print ('Sleeping a few seconds to wait for memory to be freed...')
#time.sleep (5)
#bpy.ops.wm.quit_blender ()


