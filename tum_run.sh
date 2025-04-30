#!/bin/bash

#------------------------------------
# Monocular Test
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
for i in {1..10}; do
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-corridor1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-corridor1_512.txt corridor1$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-corridor2_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-corridor2_512.txt corridor2$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-corridor3_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-corridor3_512.txt corridor3$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-corridor4_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-corridor4_512.txt corridor4$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-corridor5_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-corridor5_512.txt corridor5$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room1_512.txt room1$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room2_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room2_512.txt room2$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room3_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room3_512.txt room3$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room4_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room4_512.txt room4$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room5_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room5_512.txt room5$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-room6_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-room6_512.txt room6$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-slides1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-slides1_512.txt slides1$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-slides2_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-slides2_512.txt slides2$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-slides3_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-slides3_512.txt slides3$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-outdoors1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-outdoors1_512.txt outdoor1$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-outdoors2_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-outdoors2_512.txt outdoor2$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-outdoors8_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-outdoors8_512.txt outdoor8$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-magistrale1_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-magistrale1_512.txt magistrale1$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-magistrale2_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-magistrale2_512.txt magistrale2$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-magistrale3_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-magistrale3_512.txt magistrale3$i
    ./Test/Monocular/mono_tum_vi ./Vocabulary/ORBvoc.txt ./Test/Monocular/TUM-VI.yaml ../TUMVI_dataset/dataset-magistrale6_512_16/mav0/cam0/data ./Test/Monocular/TUM_TimeStamps/dataset-magistrale6_512.txt magistrale6$i
done