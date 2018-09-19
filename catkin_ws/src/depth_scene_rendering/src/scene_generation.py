#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Entry point.
# To be executed in Blender. All imports must exist in Blender Python.
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
import shutil

import numpy as np

# Blender
import bpy

# Custom
from util.ansi_colors import ansi_colors
from util.spherical_pose_generation import get_rand_pose
from util.tf_transformations import quaternion_matrix

# Local, from paths added above
from scan_kinect import init_kinect, scan
#from util.yaml_util import load_yaml
import config_consts
from config_paths import get_intrinsics_path, get_depth_range_path



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

  # This increases rendering time significantly. 2 x 2 m plane is 25 s,
  #   0.5 x 0.5 m plane is 13 s. Too slow. Leave out the plane. On real robot,
  #   just segment the plane to produce similar input.
  # Create a plane. Default dimensions is 2 x 2 x 0 m
  #bpy.ops.mesh.primitive_plane_add ()
  #bpy.data.objects ['Plane'].dimensions = [0.5, 0.5, 0]


  # Some line here seg faults. Dont need it. Just don't delete the camera.
  '''
  # Create a camera
  # API https://docs.blender.org/api/blender_python_api_current/bpy.ops.object.html
  bpy.ops.object.camera_add ()

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


# Convert a 7-tuple pose to a 4 x 4 matrix, and save to file.
# Parameters:
#   cam_pos: 3-elt list or numpy array
#   cam_quat: Blender Quaternion convention, (w, x, y, z), w first.
#   noisy_scene_name: Full path to where the extrinsics is to be saved, can be
#     with a wrong extension. Will replace extension to .txt.
def save_extrinsics_from_pose (cam_pos, cam_quat, noisy_scene_name):

  # Convert cam_pos and cam_quat into a 4 x 4 matrix.
  # quaternion_matrix() takes quaternion (x, y, z, w), w last
  extrinsics = quaternion_matrix ((cam_quat[1], cam_quat[2], cam_quat[3],
    cam_quat[0]))

  # Correct the extrinsics matrix to computer vision convention.
  # Blender camera has y up, -z points toward object, unconventional for
  #   cameras in robotics. However, when camera quaternion is w1 0 0 0, its
  #   matrix saved is -1 -1 1 on the diagonal, not identity for some reason!
  #   Makes no sense.
  #   Account for that in this file, `.` other code files should not need to
  #   know about blender conventions.
  # Wrt world frame, identity camera pose (quaternion w1, 0, 0, 0, pointing
  #   downward in world) should be
  #   [1, 0, 0
  #    0, -1, 0
  #    0, 0, -1]
  # To get this diagonal (1, -1, -1) from the diagonal (-1, -1, 1) of identity
  #   camera pose, rotate pi wrt y.
  # Rotation matrix 180 wrt y
  R_flipZ = [[np.cos(np.pi), 0, np.sin(np.pi), 0],
             [0, 1, 0, 0],
             [-np.sin(np.pi), 0, np.cos(np.pi), 0],
             [0, 0, 0, 1]]
  extrinsics = np.dot (extrinsics, R_flipZ)

  # Set position after all the rotations are done
  extrinsics [0:3, 3] = cam_pos

  # Write the camera extrinsics used to capture the scene, to file with same
  #   prefix as scene just captured.
  extrinsics_path = os.path.splitext (noisy_scene_name) [0] + '.txt'
  np.savetxt (extrinsics_path, extrinsics, '%f')
  print ('%sCamera extrinsics matrix written to %s%s' % (ansi_colors.OKCYAN,
    extrinsics_path, ansi_colors.ENDC))



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

cam_info = init_kinect ()
intrinsics = cam_info [0]
kinect_min_dist = cam_info [1]
kinect_max_dist = cam_info [2]
# Write camera intrinsics matrix to text file, formatted as YAML
#   (Cannot write YAML directly, as Blender Python does not have PyYAML)
# Don't want NumPy format, because might load in C++.
intrinsics_path = get_intrinsics_path ()
np.savetxt (intrinsics_path, intrinsics, '%f')
print ('%sCamera intrinsics matrix written to %s%s' % (ansi_colors.OKCYAN,
  intrinsics_path, ansi_colors.ENDC))

# Write Kinect min/max depth range to text file, for postprocessing
depth_range_path = get_depth_range_path ()
with open (depth_range_path, 'w') as depth_range_f:
  depth_range_f.write (str (kinect_min_dist) + os.linesep)
  depth_range_f.write (str (kinect_max_dist))
print ('%sCamera depth range written to %s%s' % (ansi_colors.OKCYAN,
  depth_range_path, ansi_colors.ENDC))

# Get directory of current file
this_dir = os.path.dirname (os.path.realpath (__file__))
# Concatenate config file's relative path
config_path = os.path.realpath (os.path.join (this_dir, '..', 'config'))

# Write config text file with path to camera intrinsics path, for
#   postprocess_scenes.py to read.
cam_cfg_path = os.path.join (config_path, 'cam_config_path.txt')
with open (cam_cfg_path, 'w') as intrinsics_cfg_f:
  intrinsics_cfg_f.write (intrinsics_path)
print ('%sPath of camera intrinsics matrix written to %s%s' % (ansi_colors.OKCYAN,
  cam_cfg_path, ansi_colors.ENDC))
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



#n_objs = len (config_consts.objects)
n_objs = 1

n_camera_poses = 1 #2

# Loop through each object file
for o_i in range (n_objs):

  print ('================')
  print ('%sLoading file %d out of %d%s' % (ansi_colors.OKCYAN, o_i+1,
    len (config_consts.objects), ansi_colors.ENDC))

  obj_base = config_consts.objects [o_i]
  obj_path = os.path.join (obj_dir, obj_base)

  print ('%s  %s%s' % (ansi_colors.OKCYAN, obj_base, ansi_colors.ENDC))
  load_obj (obj_path)

  # Set stationary camera pose. Do first shot using this, as reference
  cam_pos = (0.0, 0.0, 1.0)
  # Blender quaternion has (w, x, y, z), w first.
  cam_quat = (1, 0, 0, 0)


  for c_i in range (n_camera_poses):

    # Scan scene
    out_name, orig_scene_name, orig_noisy_scene_name = scan ('Camera', cam_pos,
      cam_quat)
 
    # Rename files using better convention
    scene_name = out_name
    noisy_scene_name = os.path.splitext (out_name) [0] + '_n' + \
      os.path.splitext (out_name) [1]
    shutil.move (orig_scene_name, scene_name)
    shutil.move (orig_noisy_scene_name, noisy_scene_name)
 
    # Write scene output file names in a text file, for postprocessing script
    #   to read.
    scene_list_f.write (scene_name + os.linesep)
    noisy_scene_list_f.write (noisy_scene_name + os.linesep)
 
    # Write the camera extrinsics used to capture the scene, to file with same
    #   prefix as scene just captured.
    save_extrinsics_from_pose (cam_pos, cam_quat, noisy_scene_name)
 
 
    # Generate camera pose for next iteration.
    # Use spherical coordinates (long, lat) to calculate rot and pos.
 
    # TODO: Get object transformation matrix, then calculate camera transform
    #   wrt object. `.` camera matrix needs to be wrt object, not wrt world!
    #   This is needed to reproduce the view independent of Blender world frame.
 
    # Normally, latitude range (-90, 90). Truncate to (0, 90), so it is always
    #   above horizon, `.` tabletop
    # Blender quaternion has (w, x, y, z), w first.
    cam_pos, cam_quat = get_rand_pose (lat_range=(0, 0.5*np.pi), qwFirst=True)


  # Select all - with the above setup that made all default objs unselectable,
  #   this will select only the loaded object, without needing to know the
  #   object's name - because there is no way to know.
  print ('%sDeleting loaded object%s' % (ansi_colors.OKCYAN, ansi_colors.ENDC))
  bpy.ops.object.select_all (action='SELECT')
  #bpy.ops.object.delete ()




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

