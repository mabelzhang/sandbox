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


import os

# GraspIt
from graspit_commander_custom.graspit_commander_custom import GraspitCommander

# Custom
from util.ansi_colors import ansi_colors
from depth_scene_rendering.config_read_yaml import ConfigReadYAML

# Local
from grasp_collection.config_paths import world_subdir
from grasp_io import GraspIO


def main ():

  terminate = False

  obj_names = ConfigReadYAML.read_object_names ()


  for w_i in range (len (obj_names)):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    world_fname = os.path.join (world_subdir, obj_names [w_i])

    print ('Loading world from %s' % world_fname)

    # Load world
    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)

    grasps = GraspIO.read_grasps (os.path.basename (world_fname))
    cmeta = GraspIO.read_contact_meta (os.path.basename (world_fname))


    # Loop through each result grasp
    g_i = 0
    while g_i < len (grasps):

      print ('Grasp %d, %d contacts' % (g_i, cmeta [g_i]))
      GraspitCommander.setRobotPose (grasps [g_i].pose)
      GraspitCommander.autoGrasp ()

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


if __name__ == '__main__':
  main ()

