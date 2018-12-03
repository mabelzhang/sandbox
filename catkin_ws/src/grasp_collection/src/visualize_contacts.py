#!/usr/bin/env python

# Mabel Zhang
# 23 Oct 2018
#
# Load contact .csv files saved from grasp_collect.py, and visualize contact
#   points in 3D in RViz, on top of original object point cloud.
#


import os

import numpy as np

# ROS
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Custom
from util.create_marker import create_marker
from depth_scene_rendering.config_read_yaml import ConfigReadYAML
from depth_scene_rendering.config_consts import objects
from util.matplotlib_util import mpl_color

# Local
from grasp_io import GraspIO
from grasp_collection.config_paths import world_subdir
from grasp_collection.config_consts import ENERGY_ABBREV


def main ():

  SUFFIX = 'temp'

  rospy.init_node ('visualize_contacts', anonymous=True)

  vis_pub = rospy.Publisher ('visualization_marker', Marker, queue_size=5)
  vis_arr_pub = rospy.Publisher ('visualization_marker_array', MarkerArray,
    queue_size=5)


  objs = ConfigReadYAML.read_object_names ()
  # List of strings
  obj_names = objs [ConfigReadYAML.NAME_IDX]

  # For marker color based on grasp quality
  ENERGY_MIN = 0.7 #0.4
  ENERGY_MAX = 1.6 #0.6
  # Number of segments to calculate colormap
  N_COLOR_SEG = 10
  #SEG_WIDTH = (ENERGY_MAX - ENERGY_MIN) / N_COLOR_SEG

  mesh_dir = 'package://grasp_collection/graspit_input/models/objects/dexnet/'

  # Marker size in meters
  CONTACT_SIZE = 0.003
  # Length of normal vector
  NORM_LEN = 0.01


  terminate = False

  # Delete all markers so next round starts clean
  del_mkr = Marker ()
  create_marker (Marker.SPHERE_LIST, 'contacts', '/world', 0,
    0, 0, 0, 0, 0, 0, 0.5, CONTACT_SIZE, CONTACT_SIZE, CONTACT_SIZE,
    del_mkr, duration=0)
  del_mkr.action = Marker.DELETEALL
  vis_pub.publish (del_mkr)

  # Loop through each object
  for o_i in range (len (obj_names)):

    # File name of world XML, one file per object
    world_fname = os.path.join (world_subdir, obj_names [o_i])

    # A set of contacts per object
    mesh_mkr = Marker ()
    create_marker (Marker.MESH_RESOURCE, 'mesh', '/world', 0,
      0, 0, 0, 1, 1, 1, 0.5, 1, 1, 1,
      mesh_mkr, duration=0)
    # OBJ file path
    mesh_mkr.mesh_resource = mesh_dir + objects [o_i]


    # nGrasps x 3
    contacts_m, normals_m = GraspIO.read_contacts (os.path.basename (
      world_fname), SUFFIX)
    # Contacts meta tells how many contacts per grasp
    cmeta = GraspIO.read_contact_meta (os.path.basename (world_fname), SUFFIX)

    # List of nGrasps elts
    energies = GraspIO.read_energies (os.path.basename (world_fname),
      ENERGY_ABBREV, SUFFIX)

    # A set of contacts per object
    o_contacts_mkr = Marker ()
    create_marker (Marker.SPHERE_LIST, 'contacts', '/world', 0,
      0, 0, 0, 0, 0, 0, 0.5, CONTACT_SIZE, CONTACT_SIZE, CONTACT_SIZE,
      o_contacts_mkr, duration=0)

    o_normals_arr = MarkerArray ()

    # Contact index for current grasp
    ct_start_i = 0

    # Loop through each grasp
    for g_i in range (len (energies)):

      # A set of contacts per object
      contacts_mkr = Marker ()
      create_marker (Marker.SPHERE_LIST, 'contacts', '/world', 0,
        0, 0, 0, 0, 0, 0, 0.5, CONTACT_SIZE, CONTACT_SIZE, CONTACT_SIZE,
        contacts_mkr, duration=0)

      normals_arr = MarkerArray ()

      # Energies are negative. Use absolute value
      fraction = (abs (energies [g_i]) - ENERGY_MIN) / (ENERGY_MAX - ENERGY_MIN)
      color_tuple = mpl_color (fraction, N_COLOR_SEG, colormap_name='jet')
      print ('energy: %g' % (abs (energies [g_i])))
      #print ('fraction: %g' % fraction)

      color = ColorRGBA ()
      color.r = color_tuple [0]
      color.g = color_tuple [1]
      color.b = color_tuple [2]
      color.a = 0.8

      # Loop through each contact point x y z in this grasp
      for p_i in range (ct_start_i, ct_start_i + cmeta [g_i]):
     
        # Contact point
        pt = Point ()
        pt.x = contacts_m [p_i, 0]
        pt.y = contacts_m [p_i, 1]
        pt.z = contacts_m [p_i, 2]
        contacts_mkr.points.append (pt)
        contacts_mkr.colors.append (color)
        o_contacts_mkr.points.append (pt)
        o_contacts_mkr.colors.append (color)

        # Contact normal vector

        norm_vec = normals_m [p_i, :] - contacts_m [p_i, :]
        norm_vec /= np.linalg.norm (norm_vec)
        norm_vec *= NORM_LEN

        normals_mkr = Marker ()
        create_marker (Marker.ARROW, 'normals', '/world', p_i,
          # sx: shaft diameter, sy: arrowhead diameter
          0, 0, 0, color.r, color.g, color.b, color.a, 0.001, 0.002, 0,
          normals_mkr, duration=0)
        # Start at contact point
        norm_s = Point ()
        norm_s.x = contacts_m [p_i, 0]
        norm_s.y = contacts_m [p_i, 1]
        norm_s.z = contacts_m [p_i, 2]
        normals_mkr.points.append (norm_s)
        # End at endpoint of vector
        norm_t = Point ()
        norm_t.x = contacts_m [p_i, 0] + norm_vec [0]
        norm_t.y = contacts_m [p_i, 1] + norm_vec [1]
        norm_t.z = contacts_m [p_i, 2] + norm_vec [2]
        normals_mkr.points.append (norm_t)
        normals_arr.markers.append (normals_mkr)
        o_normals_arr.markers.append (normals_mkr)

        print ('Normal length: %g' % (np.linalg.norm (normals_m [p_i, :] - contacts_m [p_i, :])))

      # Update for next grasp
      ct_start_i += cmeta [g_i]

      for i in range (5):
        vis_pub.publish (mesh_mkr)
        vis_pub.publish (contacts_mkr)
        vis_arr_pub.publish (normals_arr)
        rospy.sleep (0.5)

      uinput = raw_input ('Press enter to go to next grasp, s to skip to next object, q to quit: ')
      if uinput.lower () == 's':
        break
      elif uinput.lower () == 'q':
        terminate = True
        break

      # Delete all markers so next round starts clean
      del_mkr = Marker ()
      create_marker (Marker.SPHERE_LIST, 'contacts', '/world', 0,
        0, 0, 0, 0, 0, 0, 0.5, CONTACT_SIZE, CONTACT_SIZE, CONTACT_SIZE,
        del_mkr, duration=0)
      del_mkr.action = Marker.DELETEALL
      vis_pub.publish (del_mkr)


    #print (o_contacts_mkr.points)
    #print (o_contacts_mkr.colors)

    if terminate:
      break

    for i in range (5):
      vis_pub.publish (mesh_mkr)
      vis_pub.publish (o_contacts_mkr)
      vis_arr_pub.publish (o_normals_arr)
      rospy.sleep (0.5)

    uinput = raw_input ('Press enter to go to next object, q to quit: ')
    if uinput.lower () == 'q':
      break


if __name__ == '__main__':
  main ()

