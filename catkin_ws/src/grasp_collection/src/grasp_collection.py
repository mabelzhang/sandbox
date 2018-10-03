#!/usr/bin/env python

# Mabel Zhang
# 26 Sep 2018
#
# Calls GraspIt to plan eigengrasps and save them to disk.
#

import os
import re  # Regular expressions

import rospy
import rospkg
from geometry_msgs.msg import Pose

from graspit_commander_custom.graspit_commander_custom import GraspitCommander

from graspit_interface.srv import LoadWorld
from graspit_interface.msg import SearchContact

# Local
from config_consts import worlds


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

  skip_contact_bodies = []


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


    # Pros: This returns result directly to me. GraspitCommands returns nothing
    '''
    rospy.wait_for_service ('/graspit/loadWorld')
    try:
      load_world_srv = rospy.ServiceProxy ('/graspit/loadWorld', LoadWorld)
      resp = load_world_srv (world_fname)
      if result.result != LoadWorld._response_class.RESULT_SUCCESS:
        print ('ERROR: Failed to load world file %s' % world_fname)

    except rospy.ServiceException, e:
      print ('ERROR: Exception while calling /graspit/loadWorld service.')
    '''


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


    # Loop through each result grasp
    for g_i in range (len (gres.grasps)):

      print ('Grasp %d: energy %g' % (g_i, gres.energies [g_i]))
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
        n_contacts += 1
      print ('%d contacts' % n_contacts)

      print ('')

      n_contacts_ttl += n_contacts
     

      GraspitCommander.autoOpen ()
     

      #uinput = raw_input ('Press enter to view next grasp, q to stop viewing')
      #if uinput.lower () == 'q':
      #  break
 
    print ('Total %d contacts in %d grasps' % (n_contacts_ttl, n_best_grasps))

     
    # Save grasps and contact locations to disk. Try to do this just ONCE,
    #   `.` file I/O is expensive




if __name__ == '__main__':

  main ()

