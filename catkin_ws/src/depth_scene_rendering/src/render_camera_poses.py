#!/usr/bin/env python

# Mabel Zhang
# 20 Sep 2018
#
# Render images from a range of camera poses.
#


import sys
# Add current directory to Blender path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')
# Add utilities
sys.path.append ('/media/master/Data_Ubuntu/courses/research/github_repos/clean_repos/catkin_ws/src/util/util/src')

import os
import time

import numpy as np

# Blender
import bpy

# Custom
from util.ansi_colors import ansi_colors

# Local
from scene_generation import reset_scene, setup_camera, load_obj
from scan_kinect import ScanKinect
from config_paths import get_render_path
import config_consts


def setup_render_camera (kinect_obj):

  cam = bpy.data.objects ['Camera']

  cam.rotation_mode = 'XYZ'

  # Set Blender regular camera configuration, for rendering.
  # Set to same focal length and sensor width as the Kinect used to render
  #   data, so that the object appears the same size in image!
  # 4.73
  bpy.data.cameras ['Camera'].lens = cam.kinect_flength
  # 4.992
  bpy.data.cameras ['Camera'].sensor_width = kinect_obj.pixel_width * \
    cam.kinect_xres
  print ('Setting focal length to %g, sensor width to %g' % (
    bpy.data.cameras ['Camera'].lens, bpy.data.cameras ['Camera'].sensor_width))

  # Render config
  bpy.data.scenes ['Scene'].render.resolution_x = 640
  bpy.data.scenes ['Scene'].render.resolution_y = 480


# tx, ty, tz, rx, ry, rz: All have same length. t is for translation, r is
#   for rotation. rx ry rz are Euler XYZ rotation, Blender default.
def render_at_poses (tx, ty, tz, rx, ry, rz, out_path):

  # Convert to radians
  rx_rads = rx * np.pi / 180.0
  ry_rads = ry * np.pi / 180.0
  rz_rads = rz * np.pi / 180.0

  # To write paths of images outputted
  meta_path = os.path.join (out_path, 'uncropped.txt')
  meta_f = open (meta_path, 'w')

  cam = bpy.data.objects ['Camera']

  for i in range (rx.size):

    cam.location = (tx [i], ty [i], tz [i])

    cam.rotation_euler = (rx_rads [i], ry_rads [i], rz_rads [i])

    # Render an image. Save rotation in degrees for readability
    # Ref https://stackoverflow.com/questions/14982836/rendering-and-saving-images-through-blender-python
    bpy.data.scenes ['Scene'].render.filepath = os.path.join (out_path,
      'x%g_y%g_z%g_rx%g_ry%g_rz%g.png' % (tx[i], ty[i], tz[i], rx[i], ry[i], rz[i]))
    bpy.ops.render.render (write_still=True)

    # Write image path to file
    # Ref do not use os.linesep()  https://stackoverflow.com/questions/6159900/correct-way-to-write-line-to-file
    meta_f.write (bpy.data.scenes ['Scene'].render.filepath + '\n')

    print ('%sWritten render to %s%s' % (ansi_colors.OKCYAN,
      bpy.data.scenes ['Scene'].render.filepath, ansi_colors.ENDC))

  meta_f.close ()



# Render from different angles of the camera, to pick a range of camera
#   orientation within which to generate random poses, so that the object can
#   appear in every edge and corner of the image, for generalization.
def render_from_different_rots (kinect_obj, out_path):

  # Ranges for object bar_clamp to appear everywhere in 640 x 480:
  #   x range(-20, 21, 10), y (-20, 21, 10), z (-180, 181, 60)
  # Range for object bar_clamp to appear everywhere in center 100 x 100 crop:
  # Shape (m, 1, 1)
  rx = np.array (range (-5, 6, 5)) [np.newaxis] [np.newaxis].T
  # Shape (1, n, 1)
  ry = np.array (range (-5, 6, 5)) [np.newaxis].T [np.newaxis]
  # Shape (1, 1, p)
  rz = np.array (range (-180, 181, 60)) [np.newaxis] [np.newaxis]

  n_rx = rx.size
  n_ry = ry.size
  n_rz = rz.size

  # Shape (m, n, p)
  rx = np.tile (rx, (1, n_ry, n_rz))
  ry = np.tile (ry, (n_rx, 1, n_rz))
  rz = np.tile (rz, (n_rx, n_ry, 1))

  # Linearize
  # Size m*n*p
  rx = rx.flatten ()
  ry = ry.flatten ()
  rz = rz.flatten ()

  n_poses = rx.size


  # Keep position fixed
  tx = 0
  ty = 0
  tz = 1

  tx = np.tile (tx, n_poses)
  ty = np.tile (ty, n_poses)
  tz = np.tile (tz, n_poses)


  render_at_poses (tx, ty, tz, rx, ry, rz, out_path)


def render_from_different_pots (kinect_obj, out_path):

  # Objects are about 10-20 cm, so move camera up to 10 cm max, in xy-plane.
  # m x 1
  tx = np.array (np.arange (-0.08, 0.11, 0.05)) [np.newaxis].T
  # 1 x n
  ty = np.array (np.arange (-0.08, 0.11, 0.05)) [np.newaxis]

  n_tx = tx.size
  n_ty = ty.size

  # m x n
  tx = np.tile (tx, (1, n_ty))
  ty = np.tile (ty, (n_tx, 1))

  # 1D, size m*n
  tx = tx.flatten ()
  ty = ty.flatten ()

  n_poses = tx.size

  # Keep z (height from table) fixed
  tz = 1
  tz = np.tile (tz, n_poses)


  # Keep orientation fixed
  rx = 0
  ry = 0
  rz = 0

  rx = np.tile (rx, n_poses)
  ry = np.tile (ry, n_poses)
  rz = np.tile (rz, n_poses)


  render_at_poses (tx, ty, tz, rx, ry, rz, out_path)



if __name__ == '__main__':

  #####
  # Scene setup

  reset_scene ()

  kinect_obj = ScanKinect ()
  setup_camera (kinect_obj)

  setup_render_camera (kinect_obj)


  render_path = get_render_path ()
  rots_path = os.path.join (render_path, 'rotations')
  pots_path = os.path.join (render_path, 'translations')


  # Path with object .obj files
  obj_dir = config_consts.obj_path
  
  #n_objs = len (config_consts.objects)
  n_objs = 1

  start_time = time.time ()
  
  # Loop through each object file
  for o_i in range (n_objs):
  
    print ('================')
    print ('%sLoading file %d out of %d%s' % (ansi_colors.OKCYAN, o_i+1,
      len (config_consts.objects), ansi_colors.ENDC))
  
    obj_base = config_consts.objects [o_i]
    obj_path = os.path.join (obj_dir, obj_base)
  
    print ('%s  %s%s' % (ansi_colors.OKCYAN, obj_base, ansi_colors.ENDC))
    load_obj (obj_path)
 

    # Render from a range of camera poses and save renders to files
    #render_from_different_rots (kinect_obj, rots_path)
    render_from_different_pots (kinect_obj, pots_path)

  end_time = time.time ()

  print ('Elapsed time: %g seconds' % (end_time - start_time))
