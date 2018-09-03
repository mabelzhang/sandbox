#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# To be called from within Blender console, to use custom scripts not built-in
#   to Blender.
#
# Usage, in Blender:
#   >>> exec(open('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src/blender_add_py_paths.py').read())
#
# Ref: http://stackoverflow.com/questions/11161901/how-to-install-python-modules-in-blender

import sys
import os

# Add current directory to path
sys.path.append ('/media/master/Data_Ubuntu/courses/research/graspingRepo/visuotactile_grasping/catkin_ws/src/depth_scene_rendering/src')
# Add utilities
sys.path.append ('/media/master/Data_Ubuntu/courses/research/github_repos/clean_repos/catkin_ws/src/util/util/src')

