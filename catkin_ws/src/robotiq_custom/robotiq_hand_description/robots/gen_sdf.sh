#!/bin/bash

if [[ "$#" -ne 1 ]]; then
  echo "Argument missing."
  echo "Usage: gen_sdf.sh <filename_without_extension>"
  return
fi

xacro --inorder $1.urdf.xacro > $1.urdf
gz sdf --print $1.urdf > $1.sdf
# Ref:  https://stackoverflow.com/questions/525592/find-and-replace-inside-a-text-file-from-a-bash-command
sed -i -e 's/model:\/\/robotiq_hand/model:\/\/robotiq\/robotiq_hand/g' $1.sdf
