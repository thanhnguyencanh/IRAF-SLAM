cmake_minimum_required(VERSION 2.8)
project(iraf_slam_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  message_filters
  roscpp
  rospy
  tf
  tf2
  message_generation
)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/iraf_slam
  ${PROJECT_SOURCE_DIR}/iraf_slam/include
  ${PROJECT_SOURCE_DIR}/iraf_slam/include/CameraModels
  ${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty
  ${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# include(${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty/DBoW2/CMakeLists.txt)
# include(${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/iraf_slam/lib)

add_library(${PROJECT_NAME} SHARED
  iraf_slam/src/System.cc
  iraf_slam/src/Tracking.cc
  iraf_slam/src/LocalMapping.cc
  iraf_slam/src/LoopClosing.cc
  iraf_slam/src/ORBextractor.cc
  iraf_slam/src/ORBmatcher.cc
  iraf_slam/src/FrameDrawer.cc
  iraf_slam/src/Converter.cc
  iraf_slam/src/MapPoint.cc
  iraf_slam/src/KeyFrame.cc
  iraf_slam/src/Atlas.cc
  iraf_slam/src/Map.cc
  iraf_slam/src/MapDrawer.cc
  iraf_slam/src/Optimizer.cc
  iraf_slam/src/Frame.cc
  iraf_slam/src/KeyFrameDatabase.cc
  iraf_slam/src/Sim3Solver.cc
  iraf_slam/src/Viewer.cc
  iraf_slam/src/ImuTypes.cc
  iraf_slam/src/G2oTypes.cc
  iraf_slam/src/CameraModels/Pinhole.cpp
  iraf_slam/src/CameraModels/KannalaBrandt8.cpp
  iraf_slam/src/OptimizableTypes.cpp
  iraf_slam/src/MLPnPsolver.cpp
  iraf_slam/src/GeometricTools.cc
  iraf_slam/src/TwoViewReconstruction.cc
  iraf_slam/src/Config.cc
  iraf_slam/src/Settings.cc
  iraf_slam/include/System.h
  iraf_slam/include/Tracking.h
  iraf_slam/include/LocalMapping.h
  iraf_slam/include/LoopClosing.h
  iraf_slam/include/ORBextractor.h
  iraf_slam/include/ORBmatcher.h
  iraf_slam/include/FrameDrawer.h
  iraf_slam/include/Converter.h
  iraf_slam/include/MapPoint.h
  iraf_slam/include/KeyFrame.h
  iraf_slam/include/Atlas.h
  iraf_slam/include/Map.h
  iraf_slam/include/MapDrawer.h
  iraf_slam/include/Optimizer.h
  iraf_slam/include/Frame.h
  iraf_slam/include/KeyFrameDatabase.h
  iraf_slam/include/Sim3Solver.h
  iraf_slam/include/Viewer.h
  iraf_slam/include/ImuTypes.h
  iraf_slam/include/G2oTypes.h
  iraf_slam/include/CameraModels/GeometricCamera.h
  iraf_slam/include/CameraModels/Pinhole.h
  iraf_slam/include/CameraModels/KannalaBrandt8.h
  iraf_slam/include/OptimizableTypes.h
  iraf_slam/include/MLPnPsolver.h
  iraf_slam/include/GeometricTools.h
  iraf_slam/include/TwoViewReconstruction.h
  iraf_slam/include/SerializationUtils.h
  iraf_slam/include/Config.h
  iraf_slam/include/Settings.h
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/iraf_slam/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

## ROS node
add_executable(ros_mono
  src/ros_mono.cc
  src/common.cc
)
target_link_libraries(ros_mono
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_mono_inertial 
  src/ros_mono_inertial.cc
  src/common.cc
)
target_link_libraries(ros_mono_inertial 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_stereo 
  src/ros_stereo.cc
  src/common.cc
)
target_link_libraries(ros_stereo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_stereo_inertial
  src/ros_stereo_inertial.cc
  src/common.cc
)
target_link_libraries(ros_stereo_inertial 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_rgbd
  src/ros_rgbd.cc
  src/common.cc
)
target_link_libraries(ros_rgbd
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_rgbd_inertial
  src/ros_rgbd_inertial.cc
  src/common.cc
)
target_link_libraries(ros_rgbd_inertial
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)