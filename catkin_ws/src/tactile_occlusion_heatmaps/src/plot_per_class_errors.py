#!/usr/bin/env python

# Mabel Zhang
# 22 Nov 2018
#
# Plot per-object-class prediction error as bar plot
#

import os
import csv

import numpy as np

import matplotlib.pyplot as plt

from util.matplotlib_util import truetype, black_background, black_legend, \
  custom_colormap_neon, mpl_color, plot_line
from util.ansi_colors import ansi_colors as ansi


def main ():


  val_err_path = '/home/master/graspingRepo/train/visuotactile_grasping/analyses/model_gukpulhstc/val_err_by_class.csv'

  # Load csv file of per-class validation error, outputted by predictor
  obj_ids_tmp = []
  obj_errs_tmp = []
  with open (val_err_path, 'rb') as val_err_f:
    val_err_reader = csv.DictReader (val_err_f)
    for row in val_err_reader:
      for k in row:
        obj_ids_tmp.append (int (float (k)))
        obj_errs_tmp.append (float (row [k]))

  # Sort object IDs so that they are in order
  sort_idx = np.argsort (obj_ids_tmp)
  obj_ids = np.array (obj_ids_tmp) [sort_idx]
  obj_errs = np.array (obj_errs_tmp) [sort_idx]

  print (obj_ids)
  print (obj_errs)


  ## Plot

  truetype ()

  cm_name = custom_colormap_neon ()

  # 2 (light sky blue) and 7 (noen orange) look good
  color = mpl_color (2, 8, colormap_name=cm_name)

  _, title_hdl = plot_line (obj_ids, obj_errs,
    'Per-Object-Class Errors', 'Object', 'Error (%)',
    out_name='',
    color=color, lbl='', style='bar', grid=True, do_save=False, do_show=False,
    return_title_hdl=True)

  black_background (title_hdl=title_hdl)

  # Limit ticks to object IDs, not the extra columns on left and right
  plt.gca ().set_xticks (obj_ids)


  ## Plot a horizontal line across, at average per-class error

  # API https://matplotlib.org/api/_as_gen/matplotlib.axes.Axes.axhline.html
  plt.gca ().axhline (y=np.mean (obj_errs), linewidth=1, color='w',
    linestyle='--', label='Mean per-class')

  legend_hdl = plt.legend ()
  black_legend (legend_hdl)


  ## Plot

  img_name = os.path.splitext (val_err_path) [0] + '.eps'
  plt.savefig (img_name, bbox_inches='tight',
    facecolor=plt.gcf ().get_facecolor (), edgecolor='none', transparent=True)
  print ('%sWritten plot to %s%s' % (ansi.OKCYAN, img_name, ansi.ENDC))

  plt.show ()


if __name__ == '__main__':
  main ()

