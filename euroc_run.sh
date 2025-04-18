#!/bin/bash

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_01_easy ./Test/Monocular/EuRoC_TimeStamps/MH01.txt iraf/euroc/MH01