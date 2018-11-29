#!/bin/bash

# Mabel Zhang
# 29 Nov 2018
#
# Run grasp_collect.py many times, each time with a different file suffix, to
#   auto-collect many grasps.
# The alternative is to run with larger max_steps, but with that way, planning
#   takes a lot longer, and you do not get access to the grasps until planning
#   ends.
#
# Ctrl+Z to kill all iterations at once.
#
# Usage:
#   $ rosrun grasp_collection batch_grasp_collect.sh 
#

for i in `seq 1 50`;
do
  echo "Run $i"

  rosrun grasp_collection grasp_collect.py --suffix $i
done

