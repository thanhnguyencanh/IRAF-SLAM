#!/bin/bash

#------------------------------------
# Monocular Examples
echo "------------------------------------"
echo "Evaluation of MH01 trajectory with Monocular sensor"

python3 evaluation/py3_evaluate_ate_scale.py evaluation/Ground_truth/EuRoC_left_cam/MH05_GT.txt results/iraf/euroc/MH05_f.txt --plot MH05_mono.pdf
