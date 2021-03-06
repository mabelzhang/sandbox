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
import math

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
from config_paths import get_render_data_path


class ScanKinect:

  # Static vars
  kinect_initialized = False

  def __init__ (self):

    self.vert_fov = 0
    self.horiz_fov = 0
    self.pixel_width = 0
    self.pixel_height = 0

    self.P = None

    self.min_dist = 0
    self.max_dist = 0

    # Focal length in world units, mm
    self.flength = 0


  # Call this before calling scan()
  # Parameters:
  #   cam_name: Name of BlenSor camera in Blender scene.
  # Returns 3 x 4 camera projection matrix (really the intrinsics matrix,
  #   with Tx = Ty = 0 added to be projection, `.` no distortion in simulation.)
  def init_kinect (self, cam_name='Camera'):

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
    #   where (fx', fy') are focal lengths in pixels, (cx', cy') is principal
    #     point.
    # Ref BlenSor Kinect source code https://github.com/mgschwan/blensor/blob/master/release/scripts/addons/blensor/kinect.py
    #   They set focal length 580 pixels (4.73 mm), from
    #     http://www.ros.org/wiki/kinect_calibration/technical
    #   They have kinect_flength = 4.73 mm, which is in world units.
    #   Focal length F in world units can be obtained from f in pixels by:
    #       F_world = f_px * (film_width / width_in_pixels)
    #     They set pixel width to 7.8 um. Film width is 0.0078 mm * 640.
    #       0.00473 = f * (0.0000078 * 640 / 640)
    #             f = 0.00473/0.0000078 = 606.4102564102564
    #   http://ksimek.github.io/2013/08/13/intrinsic/
    #   They calculate principal point like so:
    #     cx = scanner_object.kinect_xres / 2.0
    #     cy = scanner_object.kinect_yres / 2.0
 
    # Copied from kinect.py line 60
    vert_fov = 43.1845
    horiz_fov = 55.6408
 
    # Copied from kinect.py line 194. Default 0.0078 mm
    pixel_width = max (0.0001,
      (math.tan ((horiz_fov / 2.0) * math.pi / 180.0) * scanner.kinect_flength) / max (
      1.0, scanner.kinect_xres / 2.0))
    # Copied from kinect.py line 196. Default 0.0078 mm
    pixel_height = max (0.0001,
      (math.tan ((vert_fov / 2.0) * math.pi / 180.0) * scanner.kinect_flength) / max (
      1.0, scanner.kinect_yres / 2.0))
 
    # Copied from kinect.py line 199-200
    cx = scanner.kinect_xres / 2.0
    cy = scanner.kinect_yres / 2.0
 
    # fx = fy = 606.411
    P = [[scanner.kinect_flength / pixel_width, 0, cx, 0],
         [0, scanner.kinect_flength / pixel_height, cy, 0],
         [0, 0, 1, 0]]
 
    #P = [[580, 0, cx, 0],
    #     [0, 580, cy, 0],
    #     [0, 0, 1, 0]]
 
    print ('Camera intrinsics matrix:')
    print (P)
 
    # TODO: 13 Sep 2018 TEMPORARY: Trying to shrink range to see if makes
    #   scaled tactile heatmaps have more varied range and visible.
    #scanner.kinect_max_dist = 2.0
 
    self.kinect_initialized = True
    self.vert_fov = vert_fov
    self.horiz_fov = horiz_fov
    self.pixel_width = pixel_width
    self.pixel_height = pixel_height
    self.P = P
    self.min_dist = scanner.kinect_min_dist
    self.max_dist = scanner.kinect_max_dist
    self.flength = scanner.kinect_flength
 
    return (P,
      scanner.kinect_min_dist,  # Default 0.70
      scanner.kinect_max_dist)  # Default 6.00


  # Parameters:
  #   quat: (w, x, y, z), Blender ordering
  def scan (self, out_path, cam_name='Camera', pos=(0, -4, 0),
    quat=(0.707, 0.707, 0, 0)):

    if not self.kinect_initialized:
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
    return (out_name,
      os.path.splitext (out_name) [0] + '00000' + os.path.splitext (out_name) [1],
      os.path.splitext (out_name) [0] + '_noisy00000' + os.path.splitext (out_name) [1])



if __name__ == '__main__':

  init_kinect ()

  # Take a scan in BlenSor's default scene, with the cube of different depth
  #   textures
  scan ()



