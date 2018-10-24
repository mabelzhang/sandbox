#!/usr/bin/env python

# Mabel Zhang
# 4 Oct 2018
#
# Writing and reading of grasp data
#


import os
import cPickle as pickle
import csv
import numpy as np

# Custom
from util.ansi_colors import ansi_colors

# Local
from grasp_collection.config_paths import get_grasps_path, get_contacts_path, \
  get_quals_path


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
    print ('%sLoading grasps from file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))

    with open (grasps_fname, 'rb') as grasps_f:
      grasps = pickle.load (grasps_f)

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


  @staticmethod
  def read_contacts (world_name):

    # csv file, of a large matrix of nContactsPerGrasp * nGrasps.
    contacts_fname = os.path.join (get_contacts_path (), world_name + '.csv')
    print ('%sLoading contacts from file %s%s' % (ansi_colors.OKCYAN,
      contacts_fname, ansi_colors.ENDC))

    contacts_m = []

    with open (contacts_fname, 'rb') as contacts_f:
      contacts_reader = csv.reader (contacts_f)
      # n x 3
      for row in contacts_reader:
        # 1 x 3. Convert strings to floats
        contacts_m.append ([float (s) for s in row])

    # n x 3
    return np.array (contacts_m)


  @staticmethod
  def read_contact_meta (world_name):

    cmeta_fname = os.path.join (get_contacts_path (), world_name + '_meta.csv')
    print ('%sLoading contacts meta from file %s%s' % (ansi_colors.OKCYAN,
      cmeta_fname, ansi_colors.ENDC))

    with open (cmeta_fname, 'rb') as cmeta_f:
      cmeta_reader = csv.reader (cmeta_f)
      # Only one row in file
      for row in cmeta_reader:
        # Convert strings to ints
        cmeta = [int (s) for s in row]

    return cmeta


  # Write grasp energies, or qualities, to csv file
  #   energies: List of floats
  @staticmethod
  def write_energies (world_name, energies):

    # Write nGrasps x 1, for easier human reading
    energies_np = np.array (energies)
    energies_np = np.reshape (energies_np, (energies_np.size, 1))

    # csv file, of a row of nGrasps elements
    quals_fname = os.path.join (get_quals_path (), world_name + '.csv')
    with open (quals_fname, 'wb') as quals_f:
      quals_writer = csv.writer (quals_f)
      quals_writer.writerow (energies_np)
    print ('%sWritten quals to file %s%s' % (ansi_colors.OKCYAN,
      quals_fname, ansi_colors.ENDC))


  # Returns list of nGrasps floats
  @staticmethod
  def read_energies (world_name):

    # csv file, of a row of nGrasps elements
    quals_fname = os.path.join (get_quals_path (), world_name + '.csv')
    print ('%sLoading quals from file %s%s' % (ansi_colors.OKCYAN,
      quals_fname, ansi_colors.ENDC))

    energies = []

    with open (quals_fname, 'rb') as quals_f:
      quals_reader = csv.reader (quals_f)
      # nGrasps x 1
      for row in quals_reader:
        # There's only 1 value per row. Convert string to float
        energies.extend ([float (s) for s in row])
      
    # List of nGrasps floats
    return energies

