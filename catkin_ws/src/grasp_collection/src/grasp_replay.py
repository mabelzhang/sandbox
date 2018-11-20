#!/usr/bin/env python

# Mabel Zhang
# 11 Oct 2018
#
# Load grasps saved by grasp_collection.py and execute them in GraspIt, for
#   inspection and debugging. Useful to reproduce the saved grasps because
#   each time GraspIt is run, grasps generated are random.
#
# Opposite process to grasp_collection.py. That file writes. This file reads.
#
# Usage:
#   $ roslaunch graspit_interface_custom graspit_interface.launch
#   $ rosrun grasp_collection grasp_replay.py
#


# Python
import os
import csv

import numpy as np

# GraspIt
from graspit_commander_custom.graspit_commander_custom import GraspitCommander

# Custom
from util.ansi_colors import ansi_colors as ansi
from util.ros_util import matrix_from_Pose, Pose_from_matrix
from depth_scene_rendering.config_read_yaml import ConfigReadYAML
from depth_scene_rendering.load_extrinsics import load_extrinsics, \
  compensate_blender_to_graspit
from tactile_occlusion_heatmaps.config_paths import get_vis_path

# Local
from grasp_collection.config_paths import world_subdir
from grasp_collect import find_contacts
from grasp_io import GraspIO
from grasp_collection.config_consts import ENERGY_ABBREV


def main ():

  terminate = False

  # Whether to visualize in GraspIt GUI
  # Set to False to just see how many grasps are in each file
  VIS = False

  objs = ConfigReadYAML.read_object_names ()
  # List of strings
  obj_names = objs [ConfigReadYAML.NAME_IDX]
  # List of list of strings, paths to .pcd scene files
  scene_paths = objs [ConfigReadYAML.SCENE_IDX]

  # Output path of screenshots of GraspIt
  vis_path = get_vis_path ()


  for w_i in range (len (obj_names)):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    world_fname = os.path.join (world_subdir, obj_names [w_i])

    print ('Loading world from %s' % world_fname)

    # Load world
    if VIS:
      GraspitCommander.clearWorld ()
      GraspitCommander.loadWorld (world_fname)
     
      # T^W_O. Used to transform contact points to be wrt object frame later
      body = GraspitCommander.getGraspableBody(0).graspable_body
      T_W_O = matrix_from_Pose (body.pose)
      # In meters
      print ('T_W_O:')
      print (T_W_O)

    grasps = GraspIO.read_grasps (os.path.basename (world_fname))
    cmeta = GraspIO.read_contact_meta (os.path.basename (world_fname))
    # For printing for debugging only
    energies = GraspIO.read_energies (os.path.basename (world_fname),
      ENERGY_ABBREV)

    print ('%d grasps' % len (grasps))
    #print ('Contact meta:')
    #print (cmeta)
    print ('Energies min %g, max %g' % (np.min (energies), np.max (energies)))
    # Bad grasps in gpqe measure are positive numbers
    if np.max (energies) > 0:
      print (np.array (energies))

    if not VIS:
      continue


    # Each object has many scenes. Each grasp is occluded differently in
    #   each scene.
    for s_i in range (len (scene_paths [w_i])):

      scene_path = scene_paths [w_i] [s_i]

      # TODO: This doesn't account for the cropping! So camera is too far to
      #   see object. Have to zoom out. But then object is too small. Need to
      #   save crop. Maybe can just pan camera?
      # Occasionally the pose isn't correct, e.g. 2018-10-19-17-40-25 way off

      # Load camera extrinsics for current scene. Camera pose wrt object.
      extrinsics_path = os.path.splitext (scene_path) [0] + '.txt'
      print ('Loading camera extrinsics (for GraspIt visualization) from %s' % (
        extrinsics_path))
      T_O_cam = load_extrinsics (extrinsics_path)

      # Graphics camera frame convention to robotics convention flip, pi wrt x.
      #   Flips z from out of frame to into frame
      T_O_cam = compensate_blender_to_graspit (T_O_cam)


      # This works for a few objects but not all of them
      '''
      # TODO: Try to put object at center of GraspIt view
      print ('T_O_cam:')
      print (T_O_cam)
      # Set position to directly at the object, to simulate the closeup crop?
      T_cam_O = np.linalg.inv (T_O_cam)
      print ('T_cam_O:')
      print (T_cam_O)
      # Pan in x and y, in camera frame, to center object in rendered view plane
      # This doesn't work, object still not in view frame
      # When camera is in north pole pose,
      #   +x moves object to the left, contrary to expected
      #   +y moves object to the bottom, as expected
      #   +z moves object behind camera, contrary to expected. -1 is right
      T_cam_O [0, 3] = 0
      T_cam_O [1, 3] = 0
      T_cam_O [2, 3] = -1
      T_O_cam = np.linalg.inv (T_cam_O)
      print ('T_O_cam:')
      print (T_O_cam)
      '''


      # Calculate camera pose wrt world
      # T_W_cam = T_W_O * T_O_cam
      T_W_cam = np.dot (T_W_O, T_O_cam)

      # Extrinsics matrix is saved in meters. GraspIt is in mm. Convert to mm.
      T_W_cam [0:3, 3] *= 1000.0

      #print ('T_O_cam:')
      #print (T_O_cam)

      print ('T_W_cam:')
      print (T_W_cam)

      # Matrix is set from depth_scene_rendering scan_kinect.py
      #       [fx'  0  cx' Tx]
      #   P = [ 0  fy' cy' Ty]
      #       [ 0   0   1   0]
      cam_pose = Pose_from_matrix (T_W_cam)

      # I don't think this has any effect on the viewport displayed. But it
      #   affects the zoom. If set to 0, then can't zoom!
      # mm. copied from <focalDistance> of my bar_clamp.xml world file
      #focal_distance = 4.73  # Blensor Kinect focal length is actually 4.73 mm
      focal_distance = 700  # BlenSor Kinect min and max depths are 0.6 to 0.7
      GraspitCommander.setCameraPose (cam_pose, focal_distance)


      # Loop through each grasp for the object
      g_i = 0
      while g_i < len (grasps):
 
        print ('Grasp %d, %d contacts' % (g_i, cmeta [g_i]))
        GraspitCommander.setRobotPose (grasps [g_i].pose)
        # Must set dofs to loaded ones, otherwise fingers won't close half of the
        #   time, and won't reproduce the recorded contacts!!
        GraspitCommander.forceRobotDof (grasps [g_i].dofs)
        print (grasps [g_i].pose)
        print (grasps [g_i].dofs)
        GraspitCommander.autoGrasp ()
 
        contacts = find_contacts (T_W_O)
        if contacts [0] != cmeta [g_i]:
          print ('%sWARN: Number of contacts loaded in replay (%d) != number of contacts in original grasp collection (%d)!%s' % (
            ansi.WARNING, contacts[0], cmeta[g_i], ansi.ENDC))

        # Same file naming convention as tactile_occlusion_heatmaps
        #   visualize_heatmaps.py
        #img_path = os.path.join (vis_path,
        #  get_vis_3d_fmt () % (
        #    os.path.splitext (os.path.basename (scene_path)) [0], g_i))
        #GraspitCommander.saveImage (img_path)

        uinput = raw_input ('Press enter to go to next grasp, b to go back to previous, r to repeat current, or q to quit: ')
        if uinput.lower () == 'b':
          g_i -= 2
        if uinput.lower () == 'r':
          g_i -= 1
        elif uinput.lower () == 'q':
          terminate = True
          break
 
        # Open gripper for next grasp
        GraspitCommander.autoOpen ()
        g_i += 1
 
        if g_i < 0:
          g_i = 0
 
 
      if terminate:
        break

    if terminate:
      break


if __name__ == '__main__':
  main ()

