#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Entry point.
# To be executed in Blender.
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
# Add current directory to Blender path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')
# Add utilities
sys.path.append ('/media/master/Data_Ubuntu/courses/research/github_repos/clean_repos/catkin_ws/src/util/util/src')

# Python
import os
import time

import numpy as np

# Blender
import bpy

# Local, from paths added above
from scan_kinect import init_kinect, scan
#from util.yaml_util import load_yaml
import config_consts
from config_paths import get_intrinsics_path
from util.ansi_colors import ansi_colors



# Clear current scene with whatever is in it, set up our scene
def reset_scene ():

  print ('================')
  print ('Initializing scene...')

  # Default blensor scene contains a Lamp, a Camera, and a Cube with cutouts.
  # Remove the Cube, add a Plane, and move the Camera to point at the Plane.

  # Deselect all objects
  # Ref https://docs.blender.org/api/blender_python_api_2_59_0/bpy.ops.object.html#bpy.ops.object.select_all
  bpy.ops.object.select_all (action='DESELECT')


  # Select and delete Cube in default scene
  print ('Deleting Cube in default scene...')
  bpy.data.objects ['Cube'].select = True
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




#####
# Scene setup

reset_scene ()

# Make default scene objects (camera, light) unselectable, so that later can
#   select all and delete loaded objects, without deleting camera and light.
# Get a list of objects in the scene
print ('Making all default objects unselectable...')
for obj in bpy.context.selectable_objects:
  obj.hide_select = True
  # Ref: https://docs.blender.org/api/blender_python_api_2_77_1/bpy.ops.outliner.html#bpy.ops.outliner.object_operation
  #bpy.ops.outliner.object_operation (type='TOGSEL')

intrinsics = init_kinect ()
intrinsics_path = get_intrinsics_path ()
# Save camera intrinsics matrix to text file, formatted as YAML
#   (Cannot write YAML directly, as Blender Python does not have PyYAML)
# Don't want NumPy format, because might load in C++.
np.savetxt (intrinsics_path, intrinsics, '%g')
print ('%sCamera intrinsics matrix written to %s%s' % (ansi_colors.OKCYAN,
  intrinsics_path, ansi_colors.ENDC))

# Get directory of current file
this_dir = os.path.dirname (os.path.realpath (__file__))
# Concatenate config file's relative path
config_path = os.path.realpath (os.path.join (this_dir, '..', 'config'))

# Write config text file with path to camera intrinsics path, for
#   postprocess_scenes.py to read.
intrinsics_cfg_path = os.path.join (config_path, 'intrinsics_path.txt')
with open (intrinsics_cfg_path, 'w') as intrinsics_cfg_f:
  intrinsics_cfg_f.write (intrinsics_path)
print ('%sPath of camera intrinsics matrix written to %s%s' % (ansi_colors.OKCYAN,
  intrinsics_cfg_path, ansi_colors.ENDC))
print ('')


#####
# Define paths

# Define file with list of scene .pcd names
scene_list_path = os.path.join (config_path, 'scenes.txt')
noisy_scene_list_path = os.path.join (config_path, 'scenes_noisy.txt')
scene_list_f = open (scene_list_path, 'w')
noisy_scene_list_f = open (noisy_scene_list_path, 'w')
print ('%sPaths of output scenes .pcd files will be written to\n  %s\n  and noisy scenes to\n  %s%s' % (
  ansi_colors.OKCYAN, scene_list_path, noisy_scene_list_path, ansi_colors.ENDC))


# Load path with object .obj files

# Using PyYAML, which isn't available in Blender
# Load YAML config file of what object CAD files to load into scene
#config_dict = load_yaml (config_path)
#obj_dir = config_dict.path

obj_dir = config_consts.obj_path



# Set stationary camera pose
# TODO: Randomize camera pose on lat/long. Use spherical coordinates to
#   calculate position. Randomize orientation as well.
cam_pos = (0.0, 0.0, 1.0)
# Blender quaternion has (w, x, y, z), w first.
cam_quat = (1, 0, 0, 0)

# Loop through each object file
#for i in range (len (config_consts.objects)):
for i in range (1):

  print ('================')
  print ('%sLoading file %d out of %d%s' % (ansi_colors.OKCYAN, i+1,
    len (config_consts.objects), ansi_colors.ENDC))

  obj_base = config_consts.objects [i]
  obj_path = os.path.join (obj_dir, obj_base)

  print ('%s  %s%s' % (ansi_colors.OKCYAN, obj_base, ansi_colors.ENDC))
  load_obj (obj_path)
  scene_name, noisy_scene_name = scan ('Camera', cam_pos, cam_quat)

  # Write scene output file names in a text file, for postprocessing script
  #   to read.
  scene_list_f.write (scene_name + os.linesep)
  noisy_scene_list_f.write (noisy_scene_name + os.linesep)

  # Select all - with the above setup that made all default objs unselectable,
  #   this will select only the loaded object, without needing to know the
  #   object's name - because there is no way to know.
  print ('%sDeleting loading object%s' % (ansi_colors.OKCYAN, ansi_colors.ENDC))
  bpy.ops.object.select_all (action='SELECT')
  bpy.ops.object.delete ()


# Close text files
scene_list_f.close ()
noisy_scene_list_f.close ()


# This doesn't quit Blender safely. It always prints mem err before quitting:
#   Error: Not freed memory blocks: 216, total unfreed memory 0.019569 MB
# Using Ctrl+Q or File>Quit also produces this error.
# But manually clicking on the X button to close Blender doesn't have this
#   issue. So will just manually close it.
#print ('Sleeping a few seconds to wait for memory to be freed...')
#time.sleep (5)
#bpy.ops.wm.quit_blender ()

