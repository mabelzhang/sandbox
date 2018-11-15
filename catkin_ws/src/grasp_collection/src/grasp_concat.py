#!/usr/bin/env python

# Mabel Zhang
# 15 Nov 2018
#
# Concatenate files from multiple runs of grasp_collect.py
#


# Local
from grasp_io import GraspIO


def main ():

  # NOTE: Manually name grasp, contact, contact meta, and energy file names to
  #   have the same prefix, excluding the _meta part and _<energy_abbrev> part.

  '''
  innames = ['nozzle_2018-11-14-20-57_52graspsWithContacts',
    'nozzle_2018-11-14-21-18_57graspsWithContacts',
    'nozzle']
  outname = 'nozzle_concat'
  '''

  #innames = ['part3_a', 'part3_b']
  #outname = 'part3'

  #innames = ['bar_clamp_a', 'bar_clamp_b']
  #outname = 'bar_clamp'

  #innames = ['gearbox_a', 'gearbox_b']
  #outname = 'gearbox'

  #innames = ['turbine_housing_a', 'turbine_housing_b']
  #outname = 'turbine_housing'

  innames = ['vase_a', 'vase_b']
  outname = 'vase'

  GraspIO.concat_grasps (innames, outname)
  print ('')
  GraspIO.concat_contacts (innames, outname)
  print ('')
  GraspIO.concat_energies (innames, outname, 'gpqe')
  print ('')


if __name__ == '__main__':
  main ()

