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
#from util.spherical_pose_generation import get_rand_pose
from util.euler_pose_generation import get_rand_rot
# NOTE that this has quaternions ordered (w, x, y, z), same as Blender, unlike
#   the ROS tf.transformations version, which has (x, y, z, w)!
from util.tf_transformations import quaternion_matrix, euler_matrix

# Local, from paths added above
from scan_kinect import ScanKinect
import config_consts
from config_paths import get_intrinsics_path, get_depth_range_path, \
  get_render_path


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


def make_all_unselectable ():

  # Make default scene objects (camera, light) unselectable, so that later can
  #   select all and delete loaded objects, without deleting camera and light.
  # Get a list of objects in the scene
  print ('Making all default objects unselectable...')
  for obj in bpy.context.selectable_objects:
    obj.hide_select = True
    # Ref: https://docs.blender.org/api/blender_python_api_2_77_1/bpy.ops.outliner.html#bpy.ops.outliner.object_operation
    #bpy.ops.outliner.object_operation (type='TOGSEL')


def setup_camera (kinect_obj):

  cam_info = kinect_obj.init_kinect ()
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
    depth_range_f.write (str (kinect_min_dist) + '\n')
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


# Load OBJ file into Blender
# Parameters:
#   obj_name: Full path to CAD file
def load_obj (obj_name):

  #bpy.ops.import_scene.obj (filepath=obj_name, axis_forward='Y', axis_up='Z')

  bpy.ops.import_scene.obj (filepath=obj_name)


# Create a 45-degree cone, useful for debugging camera matrix, as each quadrant
#   of image has an expected appearance.
# Correct camera image should have cone tip in upper-left quadrant, large base
#   in lower-right quadrant.
def create_cone ():

  bpy.ops.mesh.primitive_cone_add ()
  bpy.data.objects ['Cone'].location = (0, 0, 0)
  # Ref API https://docs.blender.org/api/blender_python_api_2_62_release/bpy.types.Object.html
  bpy.data.objects ['Cone'].rotation_mode = 'XYZ'
  bpy.data.objects ['Cone'].rotation_euler = (1.25*np.pi, 0.5*np.pi, 0)
  bpy.data.objects ['Cone'].scale = (0.05, 0.05, 0.05)
  bpy.data.objects ['Cone'].dimensions = (0.1, 0.05, 0.1)


# Convert a 7-tuple pose to a 4 x 4 matrix, and save to file.
# Parameters:
#   cam_pos: 3-elt list or numpy array
#   cam_quat: Blender Quaternion convention, (w, x, y, z), w first.
#   T_W_obj: Transform of object wrt world, expressed in world frame
#   noisy_scene_name: Full path to where the extrinsics is to be saved, can be
#     with a wrong extension. Will replace extension to .txt.
def save_extrinsics_from_pose (cam_pos, cam_quat, T_W_obj, noisy_scene_name):

  # Camera transformation wrt world, expressed in world frame
  # T^W_c
  # Convert cam_pos and cam_quat into a 4 x 4 matrix.
  # quaternion_matrix() takes quaternion (x, y, z, w), w last
  T_W_cam = quaternion_matrix ((cam_quat[0], cam_quat[1], cam_quat[2],
    cam_quat[3]))

  # Set position after all the rotations are done
  T_W_cam [0:3, 3] = cam_pos


  # Correct the extrinsics matrix to robotics convention.
  # Blender camera has y up, -z points toward object, unconventional for
  #   cameras in robotics. However, when camera quaternion is w1 0 0 0, its
  #   matrix saved is -1 -1 1 on the diagonal, not identity for some reason!
  #   Makes no sense. This doesn't match camera's RGB axes shown in Blender GUI.
  #   Account for that in this file, `.` other code files should not need to
  #   know about blender conventions.
  # Wrt world frame, identity camera pose (quaternion w1, 0, 0, 0, pointing
  #   downward in world) in computer vision convention should be
  #   [1, 0, 0
  #    0, -1, 0
  #    0, 0, -1]
  #   This has z pointing toward object (as opposed to Blender convention),
  #   x to the right, y downwards in image plane.
  # To get this diagonal (1, -1, -1) from the diagonal (-1, -1, 1) of identity
  #   camera pose, rotate pi wrt y.
  # Rotation matrix 180 wrt y
  R_flipY = [[np.cos(np.pi), 0, np.sin(np.pi), 0],
             [0, 1, 0, 0],
             [-np.sin(np.pi), 0, np.cos(np.pi), 0],
             [0, 0, 0, 1]]
  T_W_cam = np.dot (T_W_cam, R_flipY)


  # Camera transformation wrt object, expressed in object frame
  # T^o_c = T^o_W * T^W_c
  #       = (T^W_o)^-1 * T^W_c
  T_o_cam = np.dot (np.linalg.inv (T_W_obj), T_W_cam)

  #print ('camera pose wrt object:')
  #print (T_o_cam)

  print ('camera pose wrt world:')
  print (T_W_cam)


  # Write the camera extrinsics used to capture the scene, to file with same
  #   prefix as scene just captured.
  extrinsics_path = os.path.splitext (noisy_scene_name) [0] + '.txt'
  #np.savetxt (extrinsics_path, T_o_cam, '%f')
  np.savetxt (extrinsics_path, T_W_cam, '%f')
  print ('%sCamera extrinsics matrix wrt object written to %s%s' % (
    ansi_colors.OKCYAN, extrinsics_path, ansi_colors.ENDC))


if __name__ == '__main__':

  #####
  # Scene setup
  
  reset_scene ()
  make_all_unselectable ()
  
  kinect_obj = ScanKinect ()
  setup_camera (kinect_obj)

  # For human inspection
  setup_render_camera (kinect_obj)
  
  
  #####
  # Define paths
  
  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))
  # Concatenate config file's relative path
  config_path = os.path.realpath (os.path.join (this_dir, '..', 'config'))
  
  # Define file with list of scene .pcd names
  scene_list_path = os.path.join (config_path, 'scenes.yaml')
  scene_noisy_list_path = os.path.join (config_path, 'scenes_noisy.yaml')
  scene_list_f = open (scene_list_path, 'w')
  scene_noisy_list_f = open (scene_noisy_list_path, 'w')
  print ('%sPaths of output scenes .pcd files will be written to\n  %s\n  and noisy scenes to\n  %s%s' % (
    ansi_colors.OKCYAN, scene_list_path, scene_noisy_list_path, ansi_colors.ENDC))
  
  # Path with object .obj files
  obj_dir = config_consts.obj_path
  obj_suffix = config_consts.obj_suffix
  
  
  
  n_objs = len (config_consts.objects)
  #n_objs = 1
  
  n_camera_poses = 2

  # Not good to randomize on spherical coordinates, `.` when convert to
  #   Quaternion, only 2 degrees of freedom. Would need to combine with
  #   position to achieve 3DOF. But position is also only 2DOF, x and y, so
  #   better to randomize on Euler angles.
  # Blender rotation is set by Euler or Quaternion. Quaternion is not easy
  #   to quantify a range per parameter. So will define range using Euler
  #   XYZ.
  # Range in Euler XYZ is empirically chosen using render_camera_poses.py.
  rx_range = np.array ([-20, 21]) * np.pi / 180.0
  ry_range = np.array ([-20, 21]) * np.pi / 180.0
  rz_range = np.array ([-180, 180]) * np.pi / 180.0
  # This convention produces results that match Blender "Euler XYZ" convention,
  #   tested in Euler-to-Quaternion conversion in render_camera_poses.py
  #   render_at_euler_poses().
  euler_convention = 'sxyz'

  tx_range = np.array ([-0.08, 0.11])
  ty_range = np.array ([-0.08, 0.11])
  tz = 1

  # Don't need noise for random poses. Only need noise for fixed grid
  # Noise for camera position (meters) and orientation (radians)
  #T_NOISE_RANGE = 0.01
  #R_NOISE_RANGE = 

  start_time = time.time ()
  
  scene_list_f.write ('objects:\n')
  scene_noisy_list_f.write ('objects:\n')

  # Loop through each object file
  #for o_i in range (n_objs):
  for o_i in [1]:
  
    print ('================')
    print ('%sLoading file %d out of %d%s' % (ansi_colors.OKCYAN, o_i+1,
      len (config_consts.objects), ansi_colors.ENDC))
  
    obj_base = config_consts.objects [o_i]
    obj_path = os.path.join (obj_dir, obj_base)
  
    print ('%s  %s%s' % (ansi_colors.OKCYAN, obj_base, ansi_colors.ENDC))
    load_obj (obj_path)
    #create_cone ()

    # Write object name
    scene_list_f.write ('  - object: ' + obj_base.replace (obj_suffix, '') + '\n')
    scene_noisy_list_f.write ('  - object: ' + obj_base.replace (obj_suffix, '') + '\n')
  
    # Set stationary camera pose. Do first shot using this, as reference
    cam_pos = (0.0, 0.0, 1.0)
    # Blender quaternion has (w, x, y, z), w first.
    cam_euler = (0, 0, 0)
    cam_quat = (1, 0, 0, 0)
  

    scene_list_f.write ('    scenes:\n')
    scene_noisy_list_f.write ('    scenes:\n')
  
    for c_i in range (n_camera_poses):
  
      # Scan scene
      out_name, orig_scene_name, orig_noisy_scene_name = kinect_obj.scan (
        'Camera', cam_pos, cam_quat)
   
      # Rename files using better convention
      scene_name = out_name
      noisy_scene_name = os.path.splitext (out_name) [0] + '_n' + \
        os.path.splitext (out_name) [1]
      shutil.move (orig_scene_name, scene_name)
      shutil.move (orig_noisy_scene_name, noisy_scene_name)
   
      # Write scene output file names in a text file, for postprocessing script
      #   to read.
      scene_list_f.write ('    - ' + scene_name + '\n')
      scene_noisy_list_f.write ('    - ' + noisy_scene_name + '\n')


      # Render the RGB image for easy human inspection

      # Render an image using Blender default camera type
      # Ref https://stackoverflow.com/questions/14982836/rendering-and-saving-images-through-blender-python
      
      render_name = os.path.splitext (out_name) [0] + \
        '_rgb_x%.2f_y%.2f_z%.2f_rx%.2f_ry%.2f_rz%.2f.png' % (cam_pos[0], cam_pos[1],
        cam_pos[2], cam_euler[0]*180/np.pi, cam_euler[1]*180/np.pi,
        cam_euler[2]*180/np.pi)
      bpy.data.scenes ['Scene'].render.filepath = render_name
      bpy.ops.render.render (write_still=True)


      # Calculate camera extrinsics pose wrt object frame, for scene above

      # NOTE: Assumption: object file name ends with _rotated, the rest is the
      #   same as the mesh named loaded into Blender. If change any OBJ files,
      #   make sure this is still satisfied. Else won't be able to get object
      #   transformation.   
      obj_mesh_name = os.path.splitext (obj_base) [0].replace (
        '_centered_rotated', '')
      obj_pos = bpy.data.objects [obj_mesh_name].location
      # (qw, qx, qy, qz)
      obj_quat = bpy.data.objects [obj_mesh_name].rotation_quaternion

      # 4 x 4 matrix
      T_W_obj = quaternion_matrix ((obj_quat[0], obj_quat[1], obj_quat[2],
        obj_quat[3]))
      T_W_obj [0:3, 3] = obj_pos

      print ('DEBUG extrinsics. Object pose:')
      print (T_W_obj)

      # Write the camera extrinsics used to capture the scene, to file with same
      #   prefix as scene just captured.
      save_extrinsics_from_pose (cam_pos, cam_quat, T_W_obj, noisy_scene_name)
   
   
      # Generate camera pose for NEXT loop iteration.
      # Blender quaternion has (w, x, y, z), w first.

      # Use spherical coordinates (long, lat) to calculate rot and pos.
      # Normally, latitude range (-90, 90). Truncate to (0, 90), so it is always
      #   above horizon, `.` tabletop
      #cam_pos, cam_quat = get_rand_pose (lat_range=(0, 0.5*np.pi), qwFirst=True)

      tx = tx_range [0] + np.random.rand () * (tx_range[1] - tx_range[0])
      ty = ty_range [0] + np.random.rand () * (ty_range[1] - ty_range[0])
      cam_pos = [tx, ty, tz]

      # T^W_c
      # Use Euler XYZ, easier to find range
      # (qw, qx, qy, qz)
      cam_euler, cam_quat = get_rand_rot (rx_range, ry_range, rz_range,
        axes=euler_convention, qwFirst=True)
  
      #print (euler_matrix (cam_euler[0], cam_euler[1], cam_euler[2],
      #  euler_convention))
      #print (quaternion_matrix ((cam_quat[0], cam_quat[1], cam_quat[2],
      #  cam_quat[3])))


      # Don't need noise for random poses. Only need noise for fixed grid
      # Add uniformly random noise
      #t_noise = np.random.rand (3) * T_NOISE_RANGE
      #cam_pos += t_noise
      #r_noise = np.random.rand (3) * R_NOISE_RANGE
      #cam_euler += r_noise


    # Delete loaded object
    # Select all - with the above setup that made all default objs unselectable,
    #   this will select only the loaded object, without needing to know the
    #   object's name - because there is no way to know.
    print ('%sDeleting loaded object%s' % (ansi_colors.OKCYAN, ansi_colors.ENDC))
    bpy.ops.object.select_all (action='SELECT')
    bpy.ops.object.delete ()

  
  # Close text files
  scene_list_f.close ()
  scene_noisy_list_f.close ()
  
  
  # This doesn't quit Blender safely. It always prints mem err before quitting:
  #   Error: Not freed memory blocks: 216, total unfreed memory 0.019569 MB
  # Using Ctrl+Q or File>Quit also produces this error.
  # But manually clicking on the X button to close Blender doesn't have this
  #   issue. So will just manually close it.
  #print ('Sleeping a few seconds to wait for memory to be freed...')
  #time.sleep (5)
  #bpy.ops.wm.quit_blender ()

  print ('Elapsed time: %g seconds' % (time.time () - start_time))



