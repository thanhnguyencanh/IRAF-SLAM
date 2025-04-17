#!/bin/bash

#------------------------------------
# Monocular Examples
echo "------------------------------------"
echo "Evaluation of MH01 trajectory with Monocular sensor"

python3 evaluation/py3_evaluate_ate_scale.py evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt results/f_dataset-MH01-mono.txt --plot MH01_mono.pdf
