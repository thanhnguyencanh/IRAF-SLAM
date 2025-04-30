#!/bin/bash

#------------------------------------
# Monocular Examples
echo "------------------------------------"
echo "Evaluation of MH05 trajectory with Monocular sensor"

# evo_ape tum results/iraf/euroc/MH05/Ground-Truth.txt results/iraf/euroc/MH05/Our-Proposed.txt --align --correct_scale -v -va --plot --plot_mode xz --save_results results/Our-Proposed.zip
# evo_ape tum results/iraf/euroc/MH05/Ground-Truth.txt results/iraf/euroc/MH05/ORB-SLAM3.txt --align --correct_scale -v  -va --plot --plot_mode xz --save_results results/ORB-SLAM3.zip
# evo_rpe tum results/iraf/euroc/MH05/Ground-Truth.txt results/iraf/euroc/MH05/Our-Proposed.txt --pose_relation angle_deg --delta 1 --delta_unit m --plot --plot_mode xz --align --correct_scale -v --save_results results/rpeOur-Proposed.zip
# evo_rpe tum results/iraf/euroc/MH05/Ground-Truth.txt results/iraf/euroc/MH05/ORB-SLAM3.txt --pose_relation angle_deg --delta 1 --delta_unit m --plot --plot_mode xz --align --correct_scale -v --save_results results/rpeORB-SLAM3.zip


echo "room1 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf1.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf1.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf2.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf2.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf3.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf3.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf4.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf4.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf5.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf5.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf6.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf6.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/iraf7.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/iraf7.zip

# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb1.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb1.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb2.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb2.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb3.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb3.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb4.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb4.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb5.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb5.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb6.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb6.zip
# evo_ape tum results/iraf/tum/room1/gt.txt results/iraf/tum/room1/orb7.txt --align --correct_scale -v -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room1/orb7.zip


echo "room2 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf1.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf2.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf3.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf4.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf5.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf6.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/iraf7.zip

# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb1.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb2.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb3.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb4.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb5.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb6.zip
# evo_ape tum results/iraf/tum/room2/gt.txt results/iraf/tum/room2/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room2/orb7.zip

echo "room3 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf1.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf2.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf3.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf4.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf5.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf6.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/iraf7.zip

# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb1.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb2.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb3.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb4.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb5.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb6.zip
# evo_ape tum results/iraf/tum/room3/gt.txt results/iraf/tum/room3/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room3/orb7.zip

echo "room4 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf1.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf2.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf3.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf4.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf5.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf6.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/iraf7.zip

# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb1.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb2.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb3.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb4.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb5.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb6.zip
# evo_ape tum results/iraf/tum/room4/gt.txt results/iraf/tum/room4/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room4/orb7.zip

echo "room5 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf1.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf2.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf3.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf4.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf5.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf6.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/iraf7.zip

# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb1.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb2.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb3.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb4.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb5.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb6.zip
# evo_ape tum results/iraf/tum/room5/gt.txt results/iraf/tum/room5/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room5/orb7.zip

echo "room6 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf1.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf2.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf3.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf4.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf5.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf6.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/iraf7.zip

# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb1.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb2.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb3.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb4.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb5.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb6.zip
# evo_ape tum results/iraf/tum/room6/gt.txt results/iraf/tum/room6/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/room6/orb7.zip

echo "corridor1 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf1.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf2.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf3.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf4.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf5.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf6.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/iraf7.zip

# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb1.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb2.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb3.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb4.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb5.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb6.zip
# evo_ape tum results/iraf/tum/corridor1/gt.txt results/iraf/tum/corridor1/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor1/orb7.zip

echo "corridor2 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf1.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf2.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf3.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf4.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf5.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf6.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/iraf7.zip

# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb1.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb2.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb3.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb4.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb5.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb6.zip
# evo_ape tum results/iraf/tum/corridor2/gt.txt results/iraf/tum/corridor2/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor2/orb7.zip

echo "corridor3 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf1.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf2.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf3.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf4.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf5.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf6.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/iraf7.zip

# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb1.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb2.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb3.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb4.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb5.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb6.zip
# evo_ape tum results/iraf/tum/corridor3/gt.txt results/iraf/tum/corridor3/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor3/orb7.zip

echo "corridor4 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf1.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf2.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf3.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf4.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf5.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf6.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/iraf7.zip

# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb1.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb2.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb3.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb4.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb5.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb6.zip
# evo_ape tum results/iraf/tum/corridor4/gt.txt results/iraf/tum/corridor4/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor4/orb7.zip

echo "corridor5 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf1.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf2.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf3.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf4.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf5.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf6.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/iraf7.zip

# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb1.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb2.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb3.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb4.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb5.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb6.zip
# evo_ape tum results/iraf/tum/corridor5/gt.txt results/iraf/tum/corridor5/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/corridor5/orb7.zip


echo "outdoor1 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf1.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf2.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf3.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf4.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf5.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf6.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/iraf7.zip

# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb1.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb2.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb3.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb4.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb5.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb6.zip
# evo_ape tum results/iraf/tum/outdoor1/gt.txt results/iraf/tum/outdoor1/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor1/orb7.zip

cho "outdoor2 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf1.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf2.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf3.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf4.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf5.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf6.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/iraf7.zip

# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb1.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb2.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb3.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb4.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb5.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb6.zip
# evo_ape tum results/iraf/tum/outdoor2/gt.txt results/iraf/tum/outdoor2/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor2/orb7.zip

cho "outdoor8 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf1.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf2.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf3.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf4.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf5.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf6.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/iraf7.zip

# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb1.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb2.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb3.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb4.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb5.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb6.zip
# evo_ape tum results/iraf/tum/outdoor8/gt.txt results/iraf/tum/outdoor8/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/outdoor8/orb7.zip

echo "magistrale1 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf1.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf2.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf3.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf4.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf5.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf6.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/iraf7.zip

# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb1.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb2.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb3.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb4.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb5.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb6.zip
# evo_ape tum results/iraf/tum/magistrale1/gt.txt results/iraf/tum/magistrale1/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale1/orb7.zip

echo "magistrale2 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf1.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf2.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf3.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf4.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf5.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf6.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/iraf7.zip

# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb1.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb2.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb3.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb4.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb5.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb6.zip
# evo_ape tum results/iraf/tum/magistrale2/gt.txt results/iraf/tum/magistrale2/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale2/orb7.zip

echo "magistrale3 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf1.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf2.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf3.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf4.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf5.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf6.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/iraf7.zip

# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb1.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb2.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb3.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb4.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb5.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb6.zip
# evo_ape tum results/iraf/tum/magistrale3/gt.txt results/iraf/tum/magistrale3/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale3/orb7.zip

echo "magistrale6 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf1.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf2.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf3.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf4.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf5.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf6.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/iraf7.zip

# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb1.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb2.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb3.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb4.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb5.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb6.zip
# evo_ape tum results/iraf/tum/magistrale6/gt.txt results/iraf/tum/magistrale6/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/magistrale6/orb7.zip

echo "slides1 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf1.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf2.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf3.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf4.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf5.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf6.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/iraf7.zip

# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb1.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb2.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb3.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb4.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb5.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb6.zip
# evo_ape tum results/iraf/tum/slides1/gt.txt results/iraf/tum/slides1/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides1/orb7.zip

echo "slides2 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf1.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf2.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf3.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf4.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf5.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf6.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/iraf7.zip

# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb1.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb2.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb3.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb4.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb5.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb6.zip
# evo_ape tum results/iraf/tum/slides2/gt.txt results/iraf/tum/slides2/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides2/orb7.zip

echo "slides3 trajectory with Monocular sensor"

# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf1.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf2.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf3.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf4.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf5.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf6.zip
# evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/iraf7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/iraf7.zip

evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb1.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb1.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb2.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb2.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb3.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb3.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb4.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb4.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb5.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb5.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb6.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb6.zip
evo_ape tum results/iraf/tum/slides3/gt.txt results/iraf/tum/slides3/orb7.txt --align --correct_scale -v -va --t_offset 60000000 --t_max_diff 6000000 --save_results results/iraf/tum/slides3/orb7.zip




# evo_ape tum results/iraf/tum/room6/Ground-Truth.txt results/iraf/tum/room6/ORB-SLAM3.txt --align --correct_scale -v  -va --plot --plot_mode xz --t_offset 60000000 --t_max_diff 6000000 --save_results results/ORB-SLAM3.zip
# evo_rpe tum results/iraf/tum/room6/Ground-Truth.txt results/iraf/tum/room6/Our-Proposed.txt --pose_relation angle_deg --delta 1 --delta_unit m --plot --plot_mode xz --align --correct_scale -v --t_offset 60000000 --t_max_diff 6000000 --save_results results/rpeOur-Proposed.zip
# evo_rpe tum results/iraf/tum/room6/Ground-Truth.txt results/iraf/tum/room6/ORB-SLAM3.txt --pose_relation angle_deg --delta 1 --delta_unit m --plot --plot_mode xz --align --correct_scale -v --t_offset 60000000 --t_max_diff 6000000 --save_results results/rpeORB-SLAM3.zip
# evo_res results/APE/*.zip -p --save_table results/APE/table.csv
# evo_res results/RPE/*.zip -p --save_table results/RPE/table.csv