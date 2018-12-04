#!/usr/bin/env python

# Mabel Zhang
# 22 Oct 2018
#
# Output a YAML file of object names and IDs for predictor.
# Input from config_consts.py.
#

import os
import yaml

# Custom
from util.ansi_colors import ansi_colors as ansi

# Local
from depth_scene_rendering.config_consts import obj_names, obj_ids


def main ():

  # Sanity check:
  if len (obj_names) != len (obj_ids):
    print ('%sERROR: len (obj_names) != len (obj_ids). Check config_consts.py and fix it. Terminating.%s' % (ansi.OKCYAN, ansi.ENDC))
    return


  # Get directory of current file
  this_dir = os.path.dirname (os.path.realpath (__file__))
  # Concatenate config file's relative path
  config_path = os.path.realpath (os.path.join (this_dir, '..', 'config'))

  lbls_path = os.path.join (config_path, 'object_lbls.yaml')
  lbls_f = open (lbls_path, 'w')


  data = dict ()
  data ['objects'] = []

  for i in range (len (obj_names)):

    obj = dict ()

    obj ['name'] = obj_names [i]
    obj ['id'] = obj_ids [i]

    data ['objects'].append (obj)

  # Use prettier block style
  #   Ref https://pyyaml.org/wiki/PyYAMLDocumentation
  yaml_str = yaml.dump (data, default_flow_style=False)
  print (yaml_str)

  lbls_f.write (yaml_str)
  lbls_f.close ()
  print ('%sWritten object labels YAML to %s%s' % (ansi.OKCYAN, lbls_path,
    ansi.ENDC))


if __name__ == '__main__':
  main ()

