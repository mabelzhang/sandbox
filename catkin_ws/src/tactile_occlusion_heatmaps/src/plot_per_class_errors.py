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
  custom_colormap_neon, mpl_color, plot_line, mpl_diagonal_xticks
from util.ansi_colors import ansi_colors as ansi

from depth_scene_rendering.config_consts import obj_names


def main ():

  # Heatmaps in raw [0, 1] range
  # v+gp7 on 2018-11-18_3
  #model_name = 'model_zezjkjwvfe'  # 10 epochs
  #model_name = 'model_ktsqysrluy'  # 25 epochs
  # v+t+gp7 on 2018-11-18_3
  #model_name = 'model_ulvedexxtp'
  # Changed good grasp threshold to -0.53
  model_name = 'model_gwxzbgbqlk'  # v+gp7, 10 epochs
  #model_name = 'model_tssxpuieol'  # v+t+gp7, 10 epochs

  # Before changing heatmaps to raw [0, 1] range
  # 2018-11-07_09_11
  #model_name = 'model_hjigsbwqqf'
  # 2018-11-18_3
  #model_name = 'model_gukpulhstc'
  #model_name = 'model_ltjyjulfjc'  # v+gp7

  val_err_path = '/home/master/graspingRepo/train/visuotactile_grasping/analyses/' + \
    model_name + '/val_err_by_class.csv'

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

  # Replace _ with newline, so longer object names show up pretty on xtick lbls
  obj_names_ordered = [obj_names [i].replace ('_', '\n') for i in obj_ids]


  # Percentage of labels
  # Assumption: Labels are binary classification, there are only 2 labels, 0/1
  lbls_pc_path = '/home/master/graspingRepo/train/visuotactile_grasping/analyses/' + \
    model_name + '/lbls_by_class.csv'
  lbls_by_class = []
  with open (lbls_pc_path, 'rb') as lbls_pc_f:
    lbls_pc_reader = csv.DictReader (lbls_pc_f)
    for row in lbls_pc_reader:
      if len (row.keys ()) > 2:
        print ('ERROR: Labels not binary. Will not plot labels portion horizontal line in plot.')
        break

      for k in row:
        lbls_by_class.append (int (row [k]))


  ## Plot

  truetype ()

  cm_name = custom_colormap_neon ()

  # 2 (light sky blue) and 7 (noen orange) look good
  color = mpl_color (2, 8, colormap_name=cm_name)

  _, title_hdl = plot_line (obj_ids, obj_errs,
    'Per-Object-Class Errors', 'Object', 'Error',
    out_name='',
    color=color, lbl='', style='bar', dots=False, grid=True,
    do_save=False, do_show=False, return_title_hdl=True)

  black_background (title_hdl=title_hdl)

  # Limit ticks to object IDs, not the extra columns on left and right
  mpl_diagonal_xticks (plt.gca (), obj_ids, obj_names_ordered, rot_degs=0)


  ## Plot a horizontal line across, at average per-class error

  # Dashed line
  # API https://matplotlib.org/api/_as_gen/matplotlib.axes.Axes.axhline.html
  plt.gca ().axhline (y=np.mean (obj_errs), linewidth=1, color='w',
    linestyle='--', label='Mean per-class')

  if len (lbls_by_class) > 0:
    plt.gca ().axhline (y=np.min (lbls_by_class) / float (np.sum(lbls_by_class)),
      linewidth=1, color='r', linestyle='--', label='Labels portion')

  legend_hdl = plt.legend ()
  black_legend (legend_hdl)

  # y-axis is percentage
  plt.gca ().set_ylim ([0.0, 1.0])


  ## Plot

  img_name = os.path.splitext (val_err_path) [0] + '.eps'
  plt.savefig (img_name, bbox_inches='tight',
    facecolor=plt.gcf ().get_facecolor (), edgecolor='none', transparent=True)
  print ('%sWritten plot to %s%s' % (ansi.OKCYAN, img_name, ansi.ENDC))

  plt.show ()


if __name__ == '__main__':
  main ()

