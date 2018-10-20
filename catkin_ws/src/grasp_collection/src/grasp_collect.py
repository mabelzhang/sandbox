#!/usr/bin/env python

# Mabel Zhang
# 26 Sep 2018
#
# Calls GraspIt to plan eigengrasps and save them to disk.
#
# Grasps are saved as original graspit_interface/msg/Grasp.msg format to pickle,
#   because never needs to be loaded in C++. They only need to be loaded to be
#   reproduced in GraspIt, so Python suffices.
# Contacts need to be loaded in C++ by tactile_occlusion_heatmaps
#   occlusion_test.cpp to do PCL raytracing, so they are saved as .csv files.
#
# Usage:
#   $ roslaunch graspit_interface_custom graspit_interface.launch
#   $ rosrun grasp_collection grasp_collection.py
#

import os
import re  # Regular expressions
import cPickle as pickle
import csv
import yaml
import argparse

import numpy as np

import rospy
import rospkg
from geometry_msgs.msg import Pose
import tf

from graspit_commander_custom.graspit_commander_custom import GraspitCommander

from graspit_interface.srv import LoadWorld
from graspit_interface.msg import SearchContact

# Custom
from util.ros_util import matrix_from_Pose
from util.ansi_colors import ansi_colors
from depth_scene_rendering.config_read_yaml import ConfigReadYAML

# Local
from grasp_collection.config_consts import worlds
from grasp_collection.config_paths import world_subdir
from grasp_io import GraspIO


def find_contacts (T_W_O):

  # rres, r for robot
  rres = GraspitCommander.getRobot ()
  #print ('Robot:')
  #print (rres.robot)
  #print ('')
 
  # List is empty, I don't know which robot has tactile sensors. ReFlex?
  #print ('Tactile:')
  #print (rres.robot.tactile)
  #print ('')
 
  print ('Contacts:')
  #print (rres.robot.contacts)

  n_contacts = 0
  contacts_W = np.zeros ((4, len (rres.robot.contacts)))
  for contact in rres.robot.contacts:
    # Skip self-contacts and contacts with floor
    if contact.body1 == 'Base' or contact.body2 == 'Base' or \
       contact.body1 == 'simpleFloor' or contact.body2 == 'simpleFloor':
      continue
    else:
      # \d is a digit in [0-9]
      m = re.search ('_chain\d_link\d', contact.body1)
      n = re.search ('_chain\d_link\d', contact.body2)

      # If both contact bodies are robot link names, this is a self contact
      if m != None and n != None:
        continue

    #print contact
    print ('Contact between %s and %s' % (contact.body1, contact.body2))

    # Append to a matrix, so can multiply all at once at end of loop
    # 4 x n, wrt GraspIt world frame
    contacts_W [:, n_contacts] = np.dot (matrix_from_Pose (contact.ps.pose),
      [0, 0, 0, 1])

    # Increment AFTER index has been used in matrix
    n_contacts += 1

  # Delete unused columns
  contacts_W = contacts_W [:, 0:n_contacts]

  # 4 x n, wrt object frame
  # Transform contact points to be wrt object frame
  # T^O_C = T^O_W * T^W_C
  contacts_O = np.dot (np.linalg.inv (T_W_O), contacts_W)
  print ('contacts_O:')
  print (contacts_O)

  print ('%d contacts' % n_contacts)

  return n_contacts, contacts_O



def main ():

  rospy.init_node ('graspit_commander')

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_collection')


  arg_parser = argparse.ArgumentParser ()

  # Variable number of args http://stackoverflow.com/questions/13219910/argparse-get-undefined-number-of-arguments
  arg_parser.add_argument ('--debug', type=str,
    help='Specify for debugging one by one. Waits for user input to move onto displaying next grasp. Note that if you skip grasps, the contacts will NOT be saved!!')

  args = arg_parser.parse_args ()


  UINPUT = args.debug

  # Set to False if debugging and don't want to overwrite previously saved data!
  SAVE_GRASPS = True #False
  print ('%sSAVE_GRASPS is set to %s, make sure this is what you want!%s' % (
    ansi_colors.OKCYAN, str(SAVE_GRASPS), ansi_colors.ENDC))


  # TODO: Change this to a bigger number to get poor quality grasps as well
  # Temporary using small number for testing
  #n_best_grasps = 20
  # Using just 1 grasp, for many camera views, to check if there are more
  #   occluded contacts in different camera views.
  n_best_grasps = 1
  #n_best_grasps = 10

  # Default 70000, takes 30 seconds. Set to small number for testing. Usually
  #   there are grasps ranking in top 20 from 30000s. Time is not linear wrt
  #   steps, so setting to 30000 will take a lot shorter time than 70000.
  #max_steps = 70000
  max_steps = 40000

  n_contacts_ttl = 0

  # Replaced this with config_consts, because grasps don't need to be
  #   regenerated all the time! It's always about the same for the same object.
  #   Just generate a bunch, and then you never need to generate them again!
  # Read object from YAML file
  #objs = ConfigReadYAML.read_object_names ()
  #obj_names = objs [0]


  for w_i in range (len (worlds)):
  #for w_i in range (len (obj_names)):
  # TODO TEMPORARY debugging mismatch of contacts btw grasp_collection and grasp_replay
  #for w_i in [0]:
  #for w_i in range (3, len (worlds)):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    # These are the actual files in repo, $GRASPIT installation path should be
    #   symlinks to these files.
    #world_fname = os.path.join (pkg_path, 'graspit_input/worlds/dexnet/',
    #  worlds [w_i])
    #world_fname = os.path.join (world_subdir, obj_names [w_i])
    world_fname = worlds [w_i]

    print ('Loading world %d out of %d from %s' % (w_i+1, len (worlds),
      world_fname))

    # Load world
    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)

    # T^W_O. Used to transform contact points to be wrt object frame later
    body = GraspitCommander.getGraspableBody(0).graspable_body
    T_W_O = matrix_from_Pose (body.pose)
    #print ('Graspable body [0]:')
    #print (body.header)


    # Plan Eigengrasps

    # TODO Try different quality measures.

    # Returns graspit_interface_custom/action/PlanGrasps.action result.
    # Request for more than the default top 20 grasps, to get low-quality ones
    #   as well.
    # gres, g for grasp
    gres = GraspitCommander.planGrasps (n_best_grasps=n_best_grasps,
      max_steps=max_steps,
      search_contact=SearchContact(SearchContact.CONTACT_LIVE))
    print (type (gres))
    print ('Returned %d grasps. First one:' % (len (gres.grasps)))
    print (gres.grasps [0])
    print (gres.energies)
    print (gres.search_energy)

    # Parallel list to gres.grasps. Make sure these are the same length!
    #contacts_l = []
    # 3 x (nContacts * nGrasps)
    contacts_m = np.zeros ((3, 0))
    cmeta = [0] * len (gres.grasps)


    # Loop through each result grasp
    g_i = 0
    while g_i < len (gres.grasps):

      print ('\nGrasp %d: energy %g' % (g_i, gres.energies [g_i]))
      GraspitCommander.setRobotPose (gres.grasps [g_i].pose)


      # Get contact locations wrt object frame
      # /graspit/findInitialContact? No, rossrv show doesn't look like it's useful
      # There is /graspit/moveDOFToContacts, but where are the contacts? They
      #   seem to be at the blue lines in the GUI, but are they exposed?
      # /graspit/toggleAllCollisions, what does this do?
     
      # /graspit/approachToContact
      #GraspitCommander.approachToContact ()
     
      #GraspitCommander.findInitialContact ()
     
      # This gives contacts!
      GraspitCommander.autoGrasp ()

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
      #contacts_l.append (contacts_O [0:3, :])
      contacts_m = np.hstack ((contacts_m, contacts_O [0:3, :]))


      # Open gripper for next grasp
      GraspitCommander.autoOpen ()
      g_i += 1

    print ('Total %d contacts in %d grasps' % (n_contacts_ttl, n_best_grasps))

     
    # Save grasps and contact locations to disk. Try to do this just once per
    #   world file, `.` file I/O is expensive. But don't wait till end of
    #   program, `.` may leave it running for hours, don't want to lose all the
    #   work!

    '''
    # Sanity check
    if len (gres.grasps) != len (contacts_l):
      print ('%sWARN: Length of grasps (%d) != length of contacts (%d), in parallel lists! You will not know which one goes with which when you load the file.%s' % (
        ansi_colors.WARNING, len (gres.grasps), len (contacts_l),
        ansi_colors.ENDC))
    '''


    if SAVE_GRASPS:

      # One file per world for now. It contains all grasps obtained in this
      #   world, possibly hundreds of grasps.
      # Some grasps have 0 contacts with object, usually because finger hits
      #   floor, and gripper stops closing. Will save them, as bad grasps.
      #   TODO: Hopefully their energies are low. If not, might overwrite as
      #   0 energy?
      GraspIO.write_grasps (os.path.basename (world_fname), gres.grasps)
 
      GraspIO.write_contacts (os.path.basename (world_fname), contacts_m, cmeta)
 
      # Write grasp qualities to a separate csv file, for easy loading and
      #   inspection.
      GraspIO.write_energies (os.path.basename (world_fname), gres.energies)


if __name__ == '__main__':

  main ()

