#!/usr/bin/env python

# Mabel Zhang
# 23 Oct 2018
#
# Load contact .csv files saved from grasp_collect.py, and visualize contact
#   points in 3D in RViz, on top of original object point cloud.
#


import os

# ROS
import rospy
from visualization_msgs.msg import Marker
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


def main ():

  rospy.init_node ('visualize_contacts', anonymous=True)

  vis_pub = rospy.Publisher ('visualization_marker', Marker, queue_size=5)


  objs = ConfigReadYAML.read_object_names ()
  # List of strings
  obj_names = objs [0]

  # For marker color based on grasp quality
  ENERGY_MIN = 0.4
  ENERGY_MAX = 0.6
  # Number of segments to calculate colormap
  N_COLOR_SEG = 10
  #SEG_WIDTH = (ENERGY_MAX - ENERGY_MIN) / N_COLOR_SEG

  mesh_dir = 'package://grasp_collection/graspit_input/models/objects/dexnet/'

  # Marker size in meters
  CONTACT_SIZE = 0.003


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
    contacts_m = GraspIO.read_contacts (os.path.basename (world_fname))
    # Contacts meta tells how many contacts per grasp
    cmeta = GraspIO.read_contact_meta (os.path.basename (world_fname))

    # List of nGrasps elts
    quals = GraspIO.read_energies (os.path.basename (world_fname))

    # A set of contacts per object
    contacts_mkr = Marker ()
    create_marker (Marker.SPHERE_LIST, 'contacts', '/world', 0,
      0, 0, 0, 0, 0, 0, 0.5, CONTACT_SIZE, CONTACT_SIZE, CONTACT_SIZE,
      contacts_mkr, duration=0)

    # Contact index for current grasp
    ct_start_i = 0

    # Loop through each grasp
    for g_i in range (len (quals)):

      # Energies are negative. Use absolute value
      fraction = (abs (quals [g_i]) - ENERGY_MIN) / (ENERGY_MAX - ENERGY_MIN)
      color_tuple = mpl_color (fraction, N_COLOR_SEG, colormap_name='jet')
      print ('quality: %g' % (abs (quals [g_i])))
      #print ('fraction: %g' % fraction)

      color = ColorRGBA ()
      color.r = color_tuple [0]
      color.g = color_tuple [1]
      color.b = color_tuple [2]
      color.a = 0.8

      # Loop through each contact point x y z in this grasp
      for p_i in range (ct_start_i, ct_start_i + cmeta [g_i]):
     
        pt = Point ()
        pt.x = contacts_m [p_i, 0]
        pt.y = contacts_m [p_i, 1]
        pt.z = contacts_m [p_i, 2]
        contacts_mkr.points.append (pt)
     
        contacts_mkr.colors.append (color)

      # Update for next grasp
      ct_start_i += cmeta [g_i]


    #print (contacts_mkr.points)
    #print (contacts_mkr.colors)

    for i in range (5):
      vis_pub.publish (mesh_mkr)
      vis_pub.publish (contacts_mkr)
      rospy.sleep (0.5)

    uinput = raw_input ('Press enter to go to next object, q to quit: ')
    if uinput.lower () == 'q':
      break


if __name__ == '__main__':
  main ()

