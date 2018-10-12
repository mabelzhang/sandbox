#!/usr/bin/env python

# Mabel Zhang
# 4 Oct 2018
#
# Writing and reading of grasp data
#


import os
import cPickle as pickle

# Custom
from util.ansi_colors import ansi_colors

# Local
from grasp_collection.config_paths import get_grasps_path, get_contacts_path


# One file per object. A file may contain many grasps.
class GraspIO:

  # Used by grasp_collection.py
  # Parameters:
  #   obj_name: os.path.basename (world_fname), where world_fname is
  #     config_consts.worlds [i], e.g. 'dexnet/bar_clamp', file name of
  #     a GraspIt world XML file in GraspIt installation directory $GRASPIT.
  #   grasps: graspit_interface.msg.Grasp[], returned from graspit_interface
  #     action/PlanGrasps.action.
  @staticmethod
  def write_grasps (world_name, grasps):

    # Save raw Grasp type, so can easily feed back into GraspIt if need be.
    #   Otherwise need custom translation code between saved format and
    #   Grasp.msg. The less code required, the better.

    grasps_fname = os.path.join (get_grasps_path (), world_name + '.pkl')
    with open (grasps_fname, 'wb') as grasps_f:
      pickle.dump (grasps, grasps_f, pickle.HIGHEST_PROTOCOL)
    print ('%sWritten grasps to file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))


  @staticmethod
  def read_grasps (world_name):

    grasps_fname = os.path.join (get_grasps_path (), world_name + '.pkl')
    with open (grasps_fname, 'rb') as grasps_f:
      grasps = pickle.load (grasps_f)
    print ('%sLoaded grasps from file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))

    return grasps


  # Used by grasp_collection.py
  # Parameters:
  #   obj_name: os.path.basename (world_fname), where world_fname is
  #     config_consts.worlds [i], e.g. 'dexnet/bar_clamp', file name of
  #     a GraspIt world XML file in GraspIt installation directory $GRASPIT.
  #   contacts_m: 3 x (nContacts * nGrasps) numpy array, all contacts from all
  #     grasps of one EigenGrasp search session of GraspIt.
  #   cmeta: List of integers. Each integer describes number of contacts in
  #     each grasp. A grasp is a graspit_interface.msg.Grasp.
  @staticmethod
  def write_contacts (world_name, contacts_m, cmeta):

    '''
    # pickle file, of a list of matrices, one list item per grasp
    contacts_fname = os.path.join (get_contacts_path (), world_name + '.pkl')
    with open (contacts_fname, 'wb') as contacts_f:
      # contacts_l contains one 3 x nContacts matrix per element, from
      #   grasp_collection.py. Each element is one Grasp.
      pickle.dump (contacts_l, contacts_f, pickle.HIGHEST_PROTOCOL)
    '''


    # csv file, of a large matrix of nContactsPerGrasp * nGrasps.
    contacts_fname = os.path.join (get_contacts_path (), world_name + '.csv')
    with open (contacts_fname, 'wb') as contacts_f:
      contacts_writer = csv.writer (contacts_f)
      # Write n x 3, for easier human reading
      contacts_writer.writerows (contacts_m.T)
    print ('%sWritten contacts to file %s%s' % (ansi_colors.OKCYAN,
      contacts_fname, ansi_colors.ENDC))

    # Meta csv file, records how many contacts there are in each grasp. Used
    #   for indexing the big contacts matrix by grasp.
    cmeta_fname = os.path.join (get_contacts_path (), world_name + '_meta.csv')
    with open (cmeta_fname, 'wb') as cmeta_f:
      cmeta_writer = csv.writer (cmeta_f)
      cmeta_writer.writerow (cmeta)
    print ('%sWritten contacts meta to file %s%s' % (ansi_colors.OKCYAN,
      cmeta_fname, ansi_colors.ENDC))



