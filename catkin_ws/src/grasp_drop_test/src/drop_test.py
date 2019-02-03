#!/usr/bin/env python

# Mabel Zhang
# 20 Jan 2019
#
# Perform drop test of grasps in Gazebo.
#

import os
import csv

import numpy as np

import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion

# Custom
from grasp_collection.config_consts import worlds, world_to_object_base
from grasp_collection.grasp_io import GraspIO
from util.ansi_colors import ansi_colors as ansi
from util.ros_util import call_trigger_rossrv, matrix_from_Pose, \
  Pose_from_matrix
from util.gazebo_util import spawn_model, delete_model, xml_replace_static, \
  gravity_on, set_model_state, get_model_state

# Local
from config_consts import dae_suffix
from config_paths import get_drop_tests_path


class DropTestIO:

  @staticmethod
  def write_drop_result (world_name, drop_res):

    drop_fname = os.path.join (get_drop_tests_path (), world_name)
    with open (drop_fname, 'wb') as drop_f:
      drop_writer = csv.writer (drop_f)
      drop_writer.writerows (drop_res)
    print ('%sWritten drop results to file %s%s' % (ansi.OKCYAN,
      drop_fname, ansi.ENDC))


  @staticmethod
  def read_drop_result (world_name):

    drop_fname = os.path.join (get_drop_tests_path (), world_name)
    print ('%sLoading drop results from file %s%s' % (ansi.OKCYAN,
      drop_fname, ansi.ENDC))

    drop_res = []
    with open (drop_fname, 'rb') as drop_f:
      drop_reader = csv.reader (drop_f)
      for row in drop_reader:
        drop_res.append ([float (s) for s in row])

    return np.array (drop_res)


# static: If true, model is ignored by physics engine. Object stays fixed.
def sdf_replace_stock (stock_sdf_path, dae_path, static=False):

  # Use random colors for objects
  color = np.random.rand (3)
  color_str = '%g %g %g' % (color[0], color[1], color[2])

  # Read the stock .sdf file
  with open (stock_sdf_path, 'r') as stock_file:
    model_xml = stock_file.read () \
      .replace ('REPLACE_STOCK_URI', dae_path) \
      .replace ('REPLACE_STOCK_RGB', color_str)

  return model_xml



def main ():

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_drop_test')
  models_path = os.path.join (pkg_path, 'models')

  # Meters. If object dropped more than this amount, count grasp as failure.
  # Assumption: Gripper is loaded at 1 meter height, object should also be
  #   at similar height. Once dropped to the ground, it would have dropped
  #   plenty more than this threshold.
  DROP_DELTA_Z_THRESH = 0.3



  objs_to_collect = range (len (worlds))

  # Activate Robotiq
  srv_success, _ = call_trigger_rossrv ('robotiq_takktile/robotiq/activate')

  # For each object
  for w_i in range (len (objs_to_collect)):

    oid = objs_to_collect [w_i]

    world_fname = worlds [oid]

    # Turn off gravity before loading object
    srv_success = gravity_on (False)
    if not srv_success:
      print ('%sERROR: Failed to turn off gravity in Gazebo. Drop test will not work!%s' % (
        ansi.FAIL, ansi.ENDC))

    # Load object mesh into Gazebo
    obj_base = world_to_object_base (world_fname)
    model_path = os.path.join (models_path, obj_base + dae_suffix)
    obj_xml = sdf_replace_stock (
      os.path.join (pkg_path, 'sdf', 'stock_dae.sdf'), model_path)
    spawn_model (obj_base, obj_xml, 'objects', Pose (Point (1, 0, 1),
      Quaternion ()))


    # Load grasp poses from file
    grasps = GraspIO.read_grasps (os.path.basename (world_fname))

    drop_res = []

    for grasp in grasps:

      # Reset scene

      # Turn off gravity before opening gripper to release object
      srv_success = gravity_on (False)
      if not srv_success:
        print ('%sERROR: Failed to turn off gravity in Gazebo. Drop test will not work!%s' % (
          ansi.FAIL, ansi.ENDC))

      # Open gripper
      print ('Opening hand')
      call_trigger_rossrv ('/robotiq_takktile/robotiq/open')


      # Move object to inverse matrix of grasp pose wrt object
      # Gripper pose wrt object
      gpose_mat = matrix_from_Pose (grasp.pose)
      # Object pose wrt gripper
      opose_mat = np.linalg.inv (gpose_mat)
      opose = Pose_from_matrix (opose_mat)
      # Set object pose wrt gripper
      srv_success = set_model_state (obj_base, opose, frame_id='robotiq')

      # Get current object height wrt world
      obj_state = get_model_state (obj_base)
      if not obj_state.success:
        print ('%sERROR: Failed to get object state from Gazebo. Skipping object')
        continue
      obj_z = obj_state.pose.position.z


      # Close gripper
      print ('Closing hand')
      call_trigger_rossrv ('/robotiq_takktile/robotiq/close')
 
      # Turn on gravity, so that object will be dropped if grasp is bad
      srv_success = gravity_on (True)
      if not srv_success:
        print ('%sERROR: Failed to turn off gravity in Gazebo. Drop test will not work!%s' % (
          ansi.FAIL, ansi.ENDC))
      # Wait for object to fall
      rospy.sleep (1)


      # Get current object height
      obj_state = get_model_state (obj_base)
      if not obj_state.success:
        print ('%sERROR: Failed to get object state from Gazebo. Skipping object')
        continue
      new_z = obj_state.pose.position.z

      # If dropped, new_z < obj_z. Subtract in direction to get positive number
      if obj_z - new_z > DROP_DELTA_Z_THRESH:
        grasp_success = False
        print ('%sDetected grasp failure (pre-grasp z %g, post-grasp z %g)%s' % (
          ansi.OKCYAN, obj_z, new_z, ansi.ENDC))
      else:
        grasp_success = True
        print ('%sDetected grasp success (pre-grasp z %g, post-grasp z %g)%s' % (
          ansi.OKCYAN, obj_z, new_z, ansi.ENDC))
      # Record drop test result
      drop_res.append (grasp_success)
 
 
    # Open gripper
    print ('Opening hand')
    call_trigger_rossrv ('/robotiq_takktile/robotiq/open')

    # Remove object
    delete_model (obj_base)

    DropTestIO.write_drop_result (os.path.basename (world_fname), drop_res)


  # Print stats
  n_grasp_success = np.count_nonzero (drop_res)
  print ('%d grasps succeeded, %d failed' % (n_grasp_success,
    len (drop_res) - n_grasp_success))

  # Deactivate Robotiq
  srv_success, _ = call_trigger_rossrv ('robotiq_takktile/robotiq/deactivate')



if __name__ == '__main__':
  main ()
