#!/usr/bin/env python

# Mabel Zhang
# 8 Nov 2018
#
# Calls GraspIt to execute manually calculated antipodal grasps, and saves them
#   to disk.
#
# Parallel to grasp_collect.py. This file is an antipodal grasp baseline to
#   compare with grasps planned by GraspIt.
#


# Python
import os
import cPickle as pickle
import csv
import yaml
import argparse
import time

import numpy as np

# ROS
import rospy
import rospkg
from geometry_msgs.msg import Pose

# GraspIt
from graspit_commander_custom.graspit_commander_custom import GraspitCommander
from graspit_interface.srv import LoadWorld
from graspit_interface.msg import SearchContact, Grasp

# Custom
from util.ros_util import matrix_from_Pose
from util.ansi_colors import ansi_colors as ansi

# Local
from grasp_collection.config_consts import worlds, SEARCH_ENERGY, ENERGY_ABBREV
from grasp_collection.config_paths import world_subdir
from grasp_io import GraspIO
from grasp_collect import find_contacts


def main ():

  rospy.init_node ('grasp_antipodal_collect')

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_collection')


  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('--debug', type=str,
    help='Specify for debugging one by one. Waits for user input to move onto displaying next grasp. Note that if you skip grasps, the contacts will NOT be saved!!')

  args = arg_parser.parse_args ()


  UINPUT = args.debug

  # Different gripper widths
  # TODO: This is a dof in GraspitCommander........... which one? What value? Wil need to try in GraspIt GUI to find out.
  antipodal_widths = [0, ]


  n_contacts_ttl = 0

  start_time = time.time ()

  for w_i in range (len (worlds)):

    world_fname = worlds [w_i]

    print ('Loading world %d out of %d from %s' % (w_i+1, len (worlds),
      world_fname))

    # Load world
    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)

    # T^W_O. Used to transform contact points to be wrt object frame later
    # Object pose wrt GraspIt world
    body = GraspitCommander.getGraspableBody(0).graspable_body
    T_W_O = matrix_from_Pose (body.pose)


    energies = []

    # 3 x (nContacts * nGrasps)
    contacts_m = np.zeros ((3, 0))
    cmeta = [0] * len (grasps)

    # Calculate gripper pose for antipodal grasp manually, based on object
    #   dimensions and pose

    # TODO: Get object dimensions. Make a script that reads OBJ file and
    #   dump dimensions into a YAML file? That way can quickly update if change
    #   mesh file, instead of manually writing the file.
    g_i = 0
    while g_i < len (antipodal_widths):

      print ('\nGrasp %d' % (g_i))
      grasp = Grasp ()

      # Body id
      grasp.graspable_body_id = 0

      # Gripper pose wrt object
      pose = Pose ()

      # TODO: Use T_W_O object pose, to calculate gripper pose wrt it

      # Float array
      # TODO: Calculate 3 antipodal poses, `.` 3-finger gripper. Do one with thumb and forefinger pinch, then one with regular width, then one with wide width?
      dofs = []


      GraspitCommander.setRobotPose (pose)
      # TODO: How many dofs are there, which one is the forefinger? will autoGrasp() close beyond the forced dofs, if I just specify 0 for everything else, and the desired opening for the forefinger width?
      #   Could try other functions if forceRobotDof() doesn't work:
      #   findInitialContact(), approachToContact(), setRobotDesiredDOF(),
      #   moveDOFToContacts()
      GraspitCommander.forceRobotDof (dofs)
 
      # This gives contacts!
      GraspitCommander.autoGrasp ()

      # Populate rest of Grasp object
      qres = GraspitCommander.computeQuality ()
      grasp.volume_quality = qres.volume
      grasp.epsilon_quality = qres.epsilon

      # The approach direction to take before picking an object
      # geometry_msgs/Vector3Stamped approach_direction
      grasp.approach_direction = # TODO what to set this to?

      grasps.append (grasp)


      energies.append (GraspitCommander.computeEnergy (SEARCH_ENERGY))
 
      n_contacts, contacts_O = find_contacts (T_W_O)
 
 
      # Let user replay current grasp, before accumulating contact count and
      #   matrices, so that if user replays current grasp many times, contacts
      #   from this iteration don't get accumulated more than once!
      if UINPUT:
        uinput = raw_input ('Press enter to view next grasp, r to replay current grasp, q to stop viewing (note skipped contacts will NOT be saved!): ')
        if uinput.lower () == 'r':
          GraspitCommander.autoOpen ()
          continue
        elif uinput.lower () == 'q':
          break
 
 
      cmeta [g_i] = n_contacts
      n_contacts_ttl += n_contacts
 
      # One list item per grasp. List item is a matrix 4 x n of n contacts
      # Take top 3 rows, skip bottom homogeneous coordinate row, all 1s
      contacts_m = np.hstack ((contacts_m, contacts_O [0:3, :]))
 
 
      # Open gripper for next grasp
      GraspitCommander.autoOpen ()
      g_i += 1

    print ('Total %d contacts in %d grasps' % (n_contacts_ttl, n_best_grasps))

     
    # Save grasps and contact locations to disk. Try to do this just once per
    #   world file, `.` file I/O is expensive. But don't wait till end of
    #   program, `.` may leave it running for hours, don't want to lose all the
    #   work!
    if SAVE_GRASPS:

      # One file per world (object) for now. It contains all grasps obtained
      #   in this world, possibly hundreds of grasps.
      # Some grasps have 0 contacts with object, usually because finger hits
      #   floor, and gripper stops closing. Will save them, as bad grasps.
      #   TODO: Hopefully their energies are low. If not, might overwrite as
      #   0 energy?
      GraspIO.write_grasps (os.path.basename (world_fname), grasps,
        suffix='antipd')
 
      GraspIO.write_contacts (os.path.basename (world_fname), contacts_m, cmeta,
        suffix='antipd')
 
      # Write grasp energies to a separate csv file, for easy loading and
      #   inspection.
      GraspIO.write_energies (os.path.basename (world_fname), energies,
        ENERGY_ABBREV, suffix='antipd')


  end_time = time.time ()
  print ('Elapsed time: %g seconds' % (end_time - start_time))


if __name__ == '__main__':

  main ()

