# IRAF-SLAM

### V1.0, April 16th, 2025
**Authors:** [Thanh Nguyen Canh](https://thanhnguyencanh.github.io/), Bao Nguyen Quoc, HaoLan Zhang, [Xiem HoangVan](https://sites.google.com/site/xiemhoang/), [Nak Young Chong](https://www.jaist.ac.jp/robot/).


IRAF-SLAM is ...



# 1. License


If you use IRAF-SLAM in an academic work, please cite:
  
    @article{thanh_IRAF-SLAM,
      title={IRAF-SLAM: An Illumination-Robust and Adaptive Feature-Culling Front-End for Visual SLAM in Challenging Environments},
      author={Thanh Nguyen Canh, Bao Nguyen Quoc, HaoLan Zhang, Xiem HoangVan, and Nak Young Chong},
      journal={}, 
      volume={},
      number={},
      pages={},
      year={2025}
     }

# 2. Prerequisites
We have tested the library in **20.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. 

## OpenCV 4.4.0
We use [OpenCV](http://opencv.org) to manipulate images and features. 

## Eigen3
[Eigen3](http://eigen.tuxfamily.org) 

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.
<!-- 
## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04. -->

# 3. Building IRAF-SLAM library and examples

Clone the repository:
```
git clone https://github.com/thanhnguyencanh/IRAF-SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd IRAF-SLAM
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.


# 4. Testing
First, we test with [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was recorded with two pinhole cameras and an inertial sensor. So please download and modify the directory in the script "euroc_run.sh"


```
./euroc_run.sh
```

Evaluation
```
./euroc_evaluation.sh
```

<!-- # 6. TUM-VI Examples
[TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Open the script "tum_vi_examples.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./tum_vi_examples
```

## Evaluation
In TUM-VI ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 

Execute the following script to process sequences and compute the RMS ATE:
```
./tum_vi_eval_examples
``` -->