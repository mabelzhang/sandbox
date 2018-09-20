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
from config_paths import get_render_path
import config_consts


# Render from different angles of the camera, to pick a range of camera
#   orientation within which to generate random poses, so that the object can
#   appear in every edge and corner of the image, for generalization.
def render_from_different_poses ():

  cam = bpy.data.objects ['Camera']

  cam.location = (0, 0, 1)

  cam.rotation_mode = 'XYZ'
  cam.rotation_euler = (0, 0, 0)

  # Shape (m, 1, 1)
  rx = np.array (range (-20, 21, 5)) [np.newaxis] [np.newaxis].T
  # Shape (1, n, 1)
  ry = np.array (range (-20, 21, 5)) [np.newaxis].T [np.newaxis]
  # Shape (1, 1, p)
  rz = np.array (range (-20, 21, 5)) [np.newaxis] [np.newaxis]

  n_rx = rx.size
  n_ry = ry.size
  n_rz = rz.size

  # Shape (m, n, p)
  rx = np.tile (rx, (1, n_ry, n_rz))
  ry = np.tile (ry, (n_rx, 1, n_rz))
  rz = np.tile (rz, (n_rx, n_ry, 1))

  # Linearize
  rx = rx.flatten ()
  ry = ry.flatten ()
  rz = rz.flatten ()

  # Convert to radians
  rx_rads = rx * np.pi / 180.0
  ry_rads = ry * np.pi / 180.0
  rz_rads = rz * np.pi / 180.0


  # TODO: Move location too
  tx = 0
  ty = 0
  tz = 1


  # Render config
  bpy.data.scenes ['Scene'].render.resolution_x = 640
  bpy.data.scenes ['Scene'].render.resolution_y = 480
  render_path = get_render_path ()


  for i in range (rx.size):

    cam.location = (tx, ty, tz)

    cam.rotation_euler = (rx_rads [i], ry_rads [i], rz_rads [i])

    # Render an image. Save rotation in degrees for readability
    # Ref https://stackoverflow.com/questions/14982836/rendering-and-saving-images-through-blender-python
    bpy.data.scenes ['Scene'].render.filepath = os.path.join (render_path,
      'x%g_y%g_z%g_rx%g_ry%g_rz%g.png' % (tx, ty, tz, rx[i], ry[i], rz[i]))
    bpy.ops.render.render (write_still=True)

    print ('%sWritten render to %s%s' % (ansi_colors.OKCYAN,
      bpy.data.scenes ['Scene'].render.filepath, ansi_colors.ENDC))


  # TODO: Crop 100 x 100 in center of image. Must be in center, so that camera
  #   intrinsics still works.



if __name__ == '__main__':

  #####
  # Scene setup

  reset_scene ()

  setup_camera ()



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
    render_from_different_poses ()

  end_time = time.time ()

  print ('Elapsed time: %g seconds' % (end_time - start_time))

