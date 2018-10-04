#!/usr/bin/env python

# Mabel Zhang
# 26 Sep 2018
#
# Calls GraspIt to plan eigengrasps and save them to disk.
#

import os
import re  # Regular expressions
import cPickle as pickle
import csv

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
#from tactile_occlusion_heatmaps import get_data_root

# Local
from config_consts import worlds
from config_paths import get_grasps_path, get_contacts_path
from util.ansi_colors import ansi_colors


def main ():

  rospy.init_node ('graspit_commander')

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_collection')

  # TODO: Change this to a bigger number to get poor quality grasps as well
  # Temporary using small number for testing
  n_best_grasps = 10

  # Default 70000, takes 30 seconds. Set to small number for testing. Usually
  #   there are grasps ranking in top 20 from 30000s. Time is not linear wrt
  #   steps, so setting to 30000 will take a lot shorter time than 70000.
  max_steps = 40000

  n_contacts_ttl = 0

  # TODO: Use this
  #save_every_n_grasps = 50


  #for w_i in range (len (worlds)):
  for w_i in range (1, 2):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    #world_fname = os.path.join (pkg_path, 'graspit_input/worlds/dexnet/',
    #  worlds [w_i])
    world_fname = worlds [w_i]

    print ('Loading world from %s' % world_fname)

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
    for g_i in range (len (gres.grasps)):

      print ('Grasp %d: energy %g' % (g_i+1, gres.energies [g_i]))
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
      # Transform contact points to be wrt object frame. TODO: Test this
      # T^O_C = T^O_W * T^W_C
      contacts_O = np.dot (np.linalg.inv (T_W_O), contacts_W)
      print (contacts_O)

      # One list item per grasp. List item is a matrix 4 x n of n contacts
      # Take top 3 rows, skip bottom homogeneous coordinate, all 1s
      #contacts_l.append (contacts_O [0:3, :])
      contacts_m = np.hstack ((contacts_m, contacts_O [0:3, :]))

      print ('%d contacts' % n_contacts)

      cmeta [g_i] = n_contacts
      n_contacts_ttl += n_contacts


      # Open gripper for next grasp
      GraspitCommander.autoOpen ()

      #uinput = raw_input ('Press enter to view next grasp, q to stop viewing')
      #if uinput.lower () == 'q':
      #  break
 
    print ('Total %d contacts in %d grasps' % (n_contacts_ttl, n_best_grasps))

     
    # Save grasps and contact locations to disk. Try to do this just once per
    #   world file, `.` file I/O is expensive. But don't wait till end of
    #   program, `.` may leave it running for hours, don't want to lose all the
    #   work!

    # One file per world for now. It contains all grasps obtained in this world,
    #   possibly hundreds of grasps.
    # TODO: Why do some grasps have 0 contacts with object? Should I just not
    #   save them? Or save them anyway because they're just bad grasps? What if
    #   they also have high energy, and are just wrong because it's simulation?
    #   Just save all of them for now, might see things that indicate they're
    #   useful.
    grasps_fname = os.path.join (get_grasps_path (),
      os.path.basename (world_fname) + '.pkl')
    with open (grasps_fname, 'wb') as grasps_f:
      pickle.dump (gres.grasps, grasps_f, pickle.HIGHEST_PROTOCOL)
    print ('%sWritten grasps to file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))


    '''
    # Sanity check
    if len (gres.grasps) != len (contacts_l):
      print ('%sWARN: Length of grasps (%d) != length of contacts (%d), in parallel lists! You will not know which one goes with which when you load the file.%s' % (
        ansi_colors.WARNING, len (gres.grasps), len (contacts_l),
        ansi_colors.ENDC))

    # pickle file, of a list of matrices, one list item per grasp
    contacts_fname = os.path.join (get_contacts_path (),
      os.path.basename (world_fname) + '.pkl')
    with open (contacts_fname, 'wb') as contacts_f:
      pickle.dump (contacts_l, contacts_f, pickle.HIGHEST_PROTOCOL)
    '''


    # csv file, of a large matrix of nContactsPerGrasp * nGrasps.
    contacts_fname = os.path.join (get_contacts_path (),
      os.path.basename (world_fname) + '.csv')
    with open (contacts_fname, 'wb') as contacts_f:
      contacts_writer = csv.writer (contacts_f)
      contacts_writer.writerows (contacts_m)
    print ('%sWritten contacts to file %s%s' % (ansi_colors.OKCYAN,
      contacts_fname, ansi_colors.ENDC))

    # Meta csv file, records how many contacts there are in each grasp. Used
    #   for indexing the big contacts matrix by grasp.
    cmeta_fname = os.path.join (get_contacts_path (),
      os.path.basename (world_fname) + '_meta.csv')
    with open (cmeta_fname, 'wb') as cmeta_f:
      cmeta_writer = csv.writer (cmeta_f)
      cmeta_writer.writerow (cmeta)
    print ('%sWritten contacts meta to file %s%s' % (ansi_colors.OKCYAN,
      cmeta_fname, ansi_colors.ENDC))






    # TODO: This needs to be in C++, in tactile_occlusion_heatmaps... how is
    #   C++ supposed to load pickle?? Maybe should use a data format standard
    #   across languages. Does ROS have its own format?
    # Grasps never need to be loaded in C++. They only need to be loaded when
    #   need to reproduce the grasp in GraspIt, so only need to be in Python.
    # Contacts however need to be loaded in C++, to do PCL raytracing.
    # Contacts are just matrices... any matrix format works. Although, they are
    #   lists of matrices...
    # One easy way is to save all contacts to a big matrix, and then save an
    #   indexing matrix to a separate file, saying how many contacts there are
    #   per grasp. Then it is pretty easy to index the big matrix.
    # Can either save as csv (large file size, possibly lower decimal precision)
    # Or save as pickle, and use https://stackoverflow.com/questions/1296162/how-can-i-read-a-python-pickle-database-file-from-c
    # Just use csv first, easy to read/write in both C++ and Python, reliable.
    #   Read it into an Eigen::MatrixXf.

    # Load grasps saved
    #with open (path, 'rb') as f:
    #  data = pickle.load (path)


if __name__ == '__main__':

  main ()

