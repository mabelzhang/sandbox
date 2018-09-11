#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Make Kinect scans in Blender, using BlenSor.
# To be executed in Blender. All imports must exist in Blender Python.
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


import sys
# Add current directory to path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')

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
from util.ansi_colors import ansi_colors

# Local
from config_paths import get_data_root


class ScanKinect:

  # Static vars
  kinect_initialized = False



# Call this before calling scan()
# Parameters:
#   cam_name: Name of BlenSor camera in Blender scene.
# Returns camera intrinsics matrix
def init_kinect (cam_name='Camera'):

  print ('Initializing camera...')

  scanner = bpy.data.objects [cam_name]
  
  # Set type to Kinect. Else default is Velodyne
  scanner.scan_type = 'kinect'

  # Camera intrinsics matrix
  # Ref: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  #   Projection/camera matrix
  #       [fx'  0  cx' Tx]
  #   P = [ 0  fy' cy' Ty]
  #       [ 0   0   1   0]
  #   where (fx', fy') are focal lengths, (cx', cy') is principal point.
  # Ref BlenSor Kinect source code https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
  #   They calculate principal point like so:
  #     cx = scanner_object.kinect_xres / 2.0
  #     cy = scanner_object.kinect_yres / 2.0
  cx = scanner.kinect_xres / 2.0
  cy = scanner.kinect_yres / 2.0
  P = [[scanner.kinect_flength, 0, cx, 0],
       [0, scanner.kinect_flength, cy, 0],
       [0, 0, 1, 0]]

  print ('Camera intrinsics matrix:')
  print (P)

  ScanKinect.kinect_initialized = True

  return (P,
    scanner.kinect_min_dist,  # Default 6.00
    scanner.kinect_max_dist)  # Default 0.70


# Parameters:
#   quat: (w, x, y, z), Blender ordering
def scan (cam_name='Camera', pos=(0, -4, 0), quat=(0.707, 0.707, 0, 0)):

  if not ScanKinect.kinect_initialized:
    init_kinect ()

  scanner = bpy.data.objects [cam_name]
 
  # Set camera transform
  scanner.location = pos
  # Ref API https://docs.blender.org/api/blender_python_api_2_62_release/bpy.types.Object.html
  # Must set mode to quaternion first, else quaternion doesn't have effect
  scanner.rotation_mode = 'QUATERNION'
  scanner.rotation_quaternion = Quaternion (quat)
  
  # Camera extrinsics
  # Create a 4x4 matrix from camera pose
  # T^W_c
  # Transformation of camera wrt world
  # Ref API: https://docs.blender.org/api/blender_python_api_2_70_release/mathutils.html#mathutils.Matrix
  cam_pos = Matrix.Translation (scanner.location)
  cam_rot = Matrix.Identity (3)
  cam_rot.rotate (Quaternion (scanner.rotation_quaternion))
  cam_pose = cam_pos * cam_rot.to_4x4 ()
  
  # Set Kinect parameters, BEFORE calling scan()
  # Doesn't help speed up scanning. Still takes 13 s to scan
  # scanner.scan_frame_start = 1
  # scanner.scan_frame_end = 2
  # Ref: Source code for BlenSor Kinect https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
  #   kinect_max_dist  # Default 6.00
  #   kinect_min_dist  # Default 0.70
  #   kinect_noise_mu  # Default 0
  #   kinect_noise_sigma  # Default 0
  #   kinect_noise_scale  # Default 0.25
  #   kinect_noise_smooth  # Default 1.50

  # Don't add mesh to scene. This could save time.
  # Set to True for debugging in Blender GUI (do not specify -b to blender).
  scanner.add_noise_scan_mesh = False


  out_path = get_data_root ()
  out_ext = '.pcd' #'.pgm'
  out_name = os.path.join (out_path, current_timestamp_string () + out_ext)
  print ('%sSensor scans will be written to %s*%s%s' % (
    ansi_colors.OKCYAN, os.path.splitext (out_name)[0],
    os.path.splitext (out_name)[1], ansi_colors.ENDC))

  # Scan scene, save to .pcd file.
  #   e.g. if file name is given as /tmp/scan.pcd, files will be saved to
  #     /tmp/scan00000.pcd and /tmp/scan_noisy00000.pcd
  # Ref source code for BlenSor Kinect (there is no API):
  #   https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
  _, _, scan_time = blensor.kinect.scan_advanced (scanner, evd_file=out_name)
    # Not sure what world_transformation is, maybe it is the transform
    #   of points in the pcd file, in world frame instead of camera frame.
    #world_transformation=cam_pose)

  # BlenSor's filename format tacks on 00000 and _noisy00000
  return (os.path.splitext (out_name) [0] + '00000' + os.path.splitext (out_name) [1],
    os.path.splitext (out_name) [0] + '_noisy00000' + os.path.splitext (out_name) [1])



def main ():

  init_kinect ()

  # Take a scan in BlenSor's default scene, with the cube of different depth
  #   textures
  scan ()



