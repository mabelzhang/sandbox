#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Make Kinect scans in Blender, using BlenSor.
#
# Usage:
#   $ blensor
#   In Blender console (Shift+F4):
#   >>> exec(open('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src/blender_add_py_paths.py').read())
#   >>> from scan_kinect import scan_kinect
#   >>> scan_kinect ()

#
# Ref: http://www.blensor.org/scan_python.html
#


# Python
import os

# Blender
import bpy
from bpy import data as D
from bpy import context as C
# API https://docs.blender.org/api/blender_python_api_2_70_release/mathutils.html#mathutils.Matrix
from mathutils import Matrix, Quaternion
#from math import *

# BlenSor for realistic simulated Kinect sensor
import blensor

# Custom
from util.io_util import current_timestamp_string

# Local
from config_paths import get_data_root


# Parameters:
#   quat: (w, x, y, z), Blender ordering
def scan (pos=(0, -4, 0), quat=(0.707, 0.707, 0, 0)):

  print ('Initializing camera...')

  scanner = bpy.data.objects ['Camera']
  
  # Source code https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
  scanner.scan_type = 'kinect'
  
  # TODO: Set camera transformation
  scanner.location = pos
  # Ref API https://docs.blender.org/api/blender_python_api_2_62_release/bpy.types.Object.html
  # Must set mode to quaternion first, else quaternion doesn't have effect
  scanner.rotation_mode = 'QUATERNION'
  scanner.rotation_quaternion = Quaternion ((1, 0, 0, 0))
  
  # Create a 4x4 matrix from camera pose
  # T^W_c
  # Transformation of camera wrt world
  # Ref API: https://docs.blender.org/api/blender_python_api_2_70_release/mathutils.html#mathutils.Matrix
  cam_pos = Matrix.Translation (scanner.location)
  cam_rot = Matrix.Identity (3)
  cam_rot.rotate (Quaternion (scanner.rotation_quaternion))
  cam_pose = cam_pos * cam_rot.to_4x4 ()
  
  # TODO: Set Kinect parameters, BEFORE calling scan()
  # Doesn't help speed up scanning. Still takes 13 s to scan
  # scanner.scan_frame_start = 1
  # scanner.scan_frame_end = 2
  #   kinect_max_dist  # Default 6.00
  #   kinect_min_dist  # Default 0.70
  #   kinect_noise_mu  # Default 0
  #   kinect_noise_sigma  # Default 0
  #   kinect_noise_scale  # Default 0.25
  #   kinect_noise_smooth  # Default 1.50
  #   kinect_flength

  # Don't add mesh to scene. This could save time.
  # Set to True for debugging in Blender GUI (do not specify -b to blender).
  scanner.add_noise_scan_mesh = False


  out_path = get_data_root ()
  out_name = os.path.join (out_path, current_timestamp_string () + '.pcd')
  print ('Sensor scans will be written to %s*.pcd' % os.path.splitext (out_name)[0])

  # Scan scene, save to .pcd file.
  #   e.g. if file name is given as /tmp/scan.pcd, files will be saved to
  #     /tmp/scan00000.pcd and /tmp/scan_noisy00000.pcd
  # Ref source code for BlenSor Kinect (there is no API):
  #   https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
  _, _, scan_time = blensor.kinect.scan_advanced (scanner, evd_file=out_name)
    # Not sure what world_transformation is, maybe it is the transform
    #   of points in the pcd file, in world frame instead of camera frame.
    #world_transformation=cam_pose)

