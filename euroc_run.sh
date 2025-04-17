#!/bin/bash

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_01_easy ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01-mono


echo "------------------------------------"
echo "Evaluation of MH01 trajectory with Monocular sensor"

python3 evaluation/evaluate_ate_scale_py3.py evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt results/f_dataset-MH01-mono.txt --plot MH01_mono.pdf
