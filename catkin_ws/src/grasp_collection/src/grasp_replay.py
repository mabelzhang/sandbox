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

# Local
from config_consts import worlds
from grasp_io import GraspIO


def main ():

  terminate = False


  for w_i in range (len (worlds)):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    world_fname = worlds [w_i]

    print ('Loading world from %s' % world_fname)

    # Load world
    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)

    grasps = GraspIO.read_grasps (os.path.basename (world_fname))


    # Loop through each result grasp
    for g_i in range (len (grasps)):

      print ('Grasp %d' % (g_i+1))
      GraspitCommander.setRobotPose (grasps [g_i].pose)
      GraspitCommander.autoGrasp ()

      uinput = raw_input ('Press enter to go to next grasp, or q to quit: ')
      if uinput.lower () == 'q':
        terminate = True
        break

      # Open gripper for next grasp
      GraspitCommander.autoOpen ()


    if terminate:
      break

if __name__ == '__main__':
  main ()

