#!/bin/bash

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
# for i in {1..10}; do
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_01_easy ./Test/Monocular/EuRoC_TimeStamps/MH01.txt MH01_$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_02_easy ./Test/Monocular/EuRoC_TimeStamps/MH02.txt MH02$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_03_medium ./Test/Monocular/EuRoC_TimeStamps/MH03.txt MH03$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_04_difficult ./Test/Monocular/EuRoC_TimeStamps/MH04.txt MH04$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/machine_hall/MH_05_difficult ./Test/Monocular/EuRoC_TimeStamps/MH05.txt MH05$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/vincon_room1/V1_01_easy ./Test/Monocular/EuRoC_TimeStamps/V101.txt V101$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/vincon_room1/V1_02_mediume ./Test/Monocular/EuRoC_TimeStamps/V102.txt V102$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/vincon_room2/V2_01_easy ./Test/Monocular/EuRoC_TimeStamps/V201.txt V201$i
#     # ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/vincon_room2/V2_02_medium ./Test/Monocular/EuRoC_TimeStamps/V202.txt V202$i
#     ./Test/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Test/Monocular/EuRoC.yaml ../euroc_dataset/vincon_room2/V2_03_difficult ./Test/Monocular/EuRoC_TimeStamps/V203.txt V203$i
# done

echo "Launching Kitti dataset with Monocular camera"

./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM_VI.yaml ../TUMVI_dataset/dataset-outdoors1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-outdoors1_512.txt dataset-outdoor1_512_mono
