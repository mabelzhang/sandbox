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
  get_energies_path


# One file per object. A file may contain many grasps.
class GraspIO:

  # Used by grasp_collection.py
  # Parameters:
  #   obj_name: os.path.basename (world_fname), where world_fname is
  #     config_consts.worlds [i], e.g. 'dexnet/bar_clamp', file name of
  #     a GraspIt world XML file in GraspIt installation directory $GRASPIT.
  #   grasps: graspit_interface.msg.Grasp[], returned from graspit_interface
  #     action/PlanGrasps.action.
  #   suffix: e.g. antipd for antipodal, etc
  @staticmethod
  def write_grasps (world_name, grasps, suffix=''):

    # Save raw Grasp type, so can easily feed back into GraspIt if need be.
    #   Otherwise need custom translation code between saved format and
    #   Grasp.msg. The less code required, the better.

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    grasps_fname = os.path.join (get_grasps_path (), world_name + suffix + '.pkl')
    with open (grasps_fname, 'wb') as grasps_f:
      pickle.dump (grasps, grasps_f, pickle.HIGHEST_PROTOCOL)
    print ('%sWritten grasps to file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))


  @staticmethod
  def read_grasps (world_name, suffix=''):

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    grasps_fname = os.path.join (get_grasps_path (), world_name + suffix + '.pkl')
    print ('%sLoading grasps from file %s%s' % (ansi_colors.OKCYAN,
      grasps_fname, ansi_colors.ENDC))

    with open (grasps_fname, 'rb') as grasps_f:
      grasps = pickle.load (grasps_f)

    return grasps


  # Concatenate multiple files into a new file
  # Parameters:
  #   innames: List of full base name of files, excluding file extension.
  @staticmethod
  def concat_grasps (innames, outname):

    all_grasps = []
    for iname in innames:
      grasps = GraspIO.read_grasps (iname)
      print ('%d grasps' % len (grasps))
      all_grasps.extend (grasps)

    GraspIO.write_grasps (outname, all_grasps)
    print ('%d grasps total' % len (all_grasps))


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
  def write_contacts (world_name, contacts_m, cmeta, suffix=''):

    '''
    # pickle file, of a list of matrices, one list item per grasp
    contacts_fname = os.path.join (get_contacts_path (), world_name + '.pkl')
    with open (contacts_fname, 'wb') as contacts_f:
      # contacts_l contains one 3 x nContacts matrix per element, from
      #   grasp_collection.py. Each element is one Grasp.
      pickle.dump (contacts_l, contacts_f, pickle.HIGHEST_PROTOCOL)
    '''


    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    # csv file, of a large matrix of nContactsPerGrasp * nGrasps.
    contacts_fname = os.path.join (get_contacts_path (), world_name + suffix + '.csv')
    with open (contacts_fname, 'wb') as contacts_f:
      contacts_writer = csv.writer (contacts_f)
      # Write n x 3, for easier human reading
      contacts_writer.writerows (contacts_m.T)
    print ('%sWritten contacts to file %s%s' % (ansi_colors.OKCYAN,
      contacts_fname, ansi_colors.ENDC))

    # Meta csv file, records how many contacts there are in each grasp. Used
    #   for indexing the big contacts matrix by grasp.
    cmeta_fname = os.path.join (get_contacts_path (), world_name + suffix + '_meta.csv')
    with open (cmeta_fname, 'wb') as cmeta_f:
      cmeta_writer = csv.writer (cmeta_f)
      cmeta_writer.writerow (cmeta)
    print ('%sWritten contacts meta to file %s%s' % (ansi_colors.OKCYAN,
      cmeta_fname, ansi_colors.ENDC))


  @staticmethod
  def read_contacts (world_name, suffix=''):

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    # csv file, of a large matrix of nContactsPerGrasp * nGrasps.
    contacts_fname = os.path.join (get_contacts_path (), world_name + suffix + '.csv')
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
  def read_contact_meta (world_name, suffix=''):

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    cmeta_fname = os.path.join (get_contacts_path (), world_name + suffix + '_meta.csv')
    print ('%sLoading contacts meta from file %s%s' % (ansi_colors.OKCYAN,
      cmeta_fname, ansi_colors.ENDC))

    with open (cmeta_fname, 'rb') as cmeta_f:
      cmeta_reader = csv.reader (cmeta_f)
      # Only one row in file
      for row in cmeta_reader:
        # Convert strings to ints
        cmeta = [int (s) for s in row]

    return cmeta


  # Parameters:
  #   innames: List of full base name of files, excluding file extension.
  #     NOTE that meta files must be named with _meta at end of file name, as
  #     read_contact_meta() assumes this.
  @staticmethod
  def concat_contacts (innames, outname):

    all_contacts_m = np.zeros ((0, 3))
    all_cmeta = []
    for iname in innames:
      # n x 3
      contacts_m = GraspIO.read_contacts (iname)
      print ('%d contacts' % (contacts_m.shape [0]))
      all_contacts_m = np.vstack ((all_contacts_m, contacts_m))

      # List of ints
      cmeta = GraspIO.read_contact_meta (iname)
      print ('%d contact metas' % (len (cmeta)))
      all_cmeta.extend (cmeta)

    GraspIO.write_contacts (outname, all_contacts_m, all_cmeta)
    print ('%d contacts total' % (all_contacts_m.shape [0]))
    print ('%d contact metas total' % (len (all_cmeta)))


  # Write grasp energies to csv file
  #   energies: List of floats
  @staticmethod
  def write_energies (world_name, energies, energy_abbrev, suffix=''):

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    # Write nGrasps x 1, for easier human reading
    energies_np = np.array (energies)
    energies_np = np.reshape (energies_np, (energies_np.size, 1))

    # csv file, of a row of nGrasps elements
    ens_fname = os.path.join (get_energies_path (),
      world_name + suffix + '_' + energy_abbrev + '.csv')
    with open (ens_fname, 'wb') as ens_f:
      ens_writer = csv.writer (ens_f)
      ens_writer.writerows (energies_np)
    print ('%sWritten energies to file %s%s' % (ansi_colors.OKCYAN,
      ens_fname, ansi_colors.ENDC))


  # Returns list of nGrasps floats
  @staticmethod
  def read_energies (world_name, energy_abbrev, suffix=''):

    if len (suffix) > 0 and not suffix.startswith ('_'):
      suffix = '_' + suffix

    # csv file, of a row of nGrasps elements
    ens_fname = os.path.join (get_energies_path (),
      world_name + suffix + '_' + energy_abbrev + '.csv')
    print ('%sLoading energies from file %s%s' % (ansi_colors.OKCYAN,
      ens_fname, ansi_colors.ENDC))

    energies = []

    with open (ens_fname, 'rb') as ens_f:
      ens_reader = csv.reader (ens_f)
      # nGrasps x 1
      for row in ens_reader:
        # There's only 1 value per row. Convert string to float
        energies.extend ([float (s) for s in row])
      
    # List of nGrasps floats
    return energies


  # Parameters:
  #   innames: List of full base name of files, excluding file extension.
  #     NOTE that files must be named with _<energy_abbrev> at end of file
  #     name, as read_energies() assumes this.
  @staticmethod
  def concat_energies (innames, outname, energy_abbrev):

    all_energies = []
    for iname in innames:
      # List of floats
      # energy_abbrev already in full file name in innames
      energies = GraspIO.read_energies (iname, energy_abbrev)
      print ('%d energies' % (len (energies)))
      all_energies.extend (energies)

    GraspIO.write_energies (outname, all_energies, energy_abbrev)
    print ('%d energies total' % (len (all_energies)))

