#!/usr/bin/env python

# Mabel Zhang
# 20 Jan 2019
#
# Perform drop test of grasps in Gazebo.
#

import os
import csv

import numpy as np

import rospkg
from geometry_msgs.msg import Pose

# Custom
from grasp_collection.config_consts import worlds, world_to_object_base
from grasp_collection.grasp_io import GraspIO
from util.ansi_colors import ansi_colors as ansi
from util.ros_util import call_trigger_rossrv, matrix_from_Pose
from util.gazebo_util import spawn_model, delete_model, xml_replace_static

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



  objs_to_collect = range (len (worlds))

  # Activate Robotiq
  success, _ = call_trigger_rossrv ('robotiq_takktile/robotiq/activate')

  # For each object
  for w_i in range (len (objs_to_collect)):

    oid = objs_to_collect [w_i]

    world_fname = worlds [oid]


    # Load object mesh into Gazebo, set static
    obj_base = world_to_object_base (world_fname)
    model_path = os.path.join (models_path, obj_base + dae_suffix)
    obj_xml = sdf_replace_stock (
      os.path.join (pkg_path, 'sdf', 'stock_dae.sdf'), model_path)
    spawn_model (obj_base, obj_xml, 'objects', Pose ())
    # TODO call gazebo srv to set object as static


    # Load grasp poses from file
    grasps = GraspIO.read_grasps (os.path.basename (world_fname))

    drop_res = []


    for grasp in grasps:

      # Move object to inverse matrix of grasp pose wrt object
      # Gripper pose wrt object
      gpose_mat = matrix_from_Pose (grasp.pose)
      # Object pose wrt gripper
      opose_mat = np.linalg.inv (gpose_mat)
      # TODO call gazebo srv to set object pose

 
      # Close gripper
      print ('Closing hand')
      call_trigger_rossrv ('/robotiq_takktile/robotiq/close')
 
 
      # Set object to non-static, to be dropped if grasp is bad
      # TODO call gazebo srv to set static to false
      #   Look at gym_gazebo grasp_collection_pipeline.py
 

      ''' 
      # TODO: instead of this, just check object height, better validation
      grasp_success, err_msg = call_trigger_rossrv (
        '/robotiq_takktile/robotiq/check_grasp')
 
 
      # Record drop test result
      drop_res.append (grasp_success)
      '''
 
      # Open gripper
      print ('Opening hand')
      call_trigger_rossrv ('/robotiq_takktile/robotiq/open')


    # Remove object
    delete_model (obj_base)

    DropTestIO.write_drop_result (os.path.basename (world_fname), drop_res)


  # Deactivate Robotiq
  success, _ = call_trigger_rossrv ('robotiq_takktile/robotiq/deactivate')



if __name__ == '__main__':
  main ()
