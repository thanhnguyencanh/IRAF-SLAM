#!/bin/bash

#------------------------------------
# Monocular Examples
echo "------------------------------------"
echo "Evaluation of MH01 trajectory with Monocular sensor"

evo_traj tum --ref=results/gt/euroc/MH04.txt results/iraf/euroc/MH04_f.txt results/iraf/euroc/orb3_MH04.txt -p --plot_mode=xy --align  --correct_scale -v

evo_traj tum --ref=results/gt/euroc/MH05.txt results/iraf/euroc/MH05_f.txt results/iraf/euroc/orb3_MH05.txt -p --plot_mode=xy --align  --correct_scale -v