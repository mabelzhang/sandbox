#!/usr/bin/env python

# Mabel Zhang
# 18 Nov 2018
#
# Extracts gripper pose from n grasps (type graspit_interface.msgs.Grasp) saved
#   from grasp_collect.py. Transforms them from GraspIt world frame to object
#   frame, and save as n x N_POSE_PARAMS matrix to .csv files.
# Part of integration with tactile_occlusion_heatmaps for input into predictor.
#
# This file's contents are now in grasp_collect.py. But for grasp pkl files
#   generated previously that did not have poses saved to csv, run this
#   file to extract the poses to csv.
#


import os

import numpy as np

# GraspIt
from graspit_commander_custom.graspit_commander_custom import GraspitCommander

# Custom
from depth_scene_rendering.config_read_yaml import ConfigReadYAML
from util.ros_util import matrix_from_Pose, _7tuple_from_matrix

# Local
from grasp_io import GraspIO
from grasp_collection.config_paths import world_subdir
from grasp_collection.config_consts import N_POSE_PARAMS, N_PARAMS_QUAT


def main ():

  objs = ConfigReadYAML.read_object_names ()
  # List of strings
  #obj_names = objs [ConfigReadYAML.NAME_IDX]
  obj_names = ['nozzle', 'part3']
  # List of list of strings, paths to .pcd scene files
  scene_paths = objs [ConfigReadYAML.SCENE_IDX]

  SUFFIX = 'd'


  # For each object world
  for w_i in range (len (obj_names)):

    # File name of world XML under $GRASPIT/worlds/, one file per object
    world_fname = os.path.join (world_subdir, obj_names [w_i])

    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)
    
    # Get transformation of object pose wrt GraspIt world frame
    # T^W_O. Used to transform quantity expressed in world frame to be
    #   expressed in object frame.
    body = GraspitCommander.getGraspableBody(0).graspable_body
    T_W_O = matrix_from_Pose (body.pose)


    # Read grasps written by grasp_collect.py, wrt GraspIt world frame
    grasps = GraspIO.read_grasps (os.path.basename (world_fname), SUFFIX)

    # n x 7
    gposes_O = np.zeros ((len (grasps), N_POSE_PARAMS))
    for g_i in range (len (grasps)):

      g = grasps [g_i]

      # Gripper pose wrt GraspIt world frame
      # 4 x 4
      pose_W = matrix_from_Pose (g.pose)

      # 4 x 4, wrt object frame
      # Transform gripper pose to be wrt object frame
      # T^O_G = T^O_W * T^W_G
      pose_O = np.dot (np.linalg.inv (T_W_O), pose_W)

      # Convert to 7-tuple and append to big matrix
      if N_POSE_PARAMS == N_PARAMS_QUAT:
        row_O = _7tuple_from_matrix (pose_O)
      else:
        print ('%sERROR: N_POSE_PARAMS %d not implemented yet. Implement it or choose a different one! Gripper pose parameterization will default to Quaternion for orientation.%s' % (ansi.FAIL, N_POSE_PARAMS, ansi.ENDC))
        row_O = _7tuple_from_matrix (pose_O)
      gposes_O [g_i, :] = np.array (row_O)

    GraspIO.write_grasp_poses (os.path.basename (world_fname), gposes_O, SUFFIX)


if __name__ == '__main__':
  main ()

