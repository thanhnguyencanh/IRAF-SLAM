/**
* This file is part of IRAF-SLAM, which is built on top of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2024-2025 Thanh Nguyen Canh, Japan Advanced Institute of Science and Technology (JAIST).
*
* IRAF-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* IRAF-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with IRAF-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;

// thanhnc: for publishing tf
private:
    ros::Time lastStamp;
    int frameCounter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    // for subscribing rgb and depth image
    ros::NodeHandle nh;

    // synchronize topic, subcribe rgb and depth image
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    // thanhnc: register callback function for synchronized topics
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

// 
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        // convert rgb image to cv::Mat with shared data
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // convert depth image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Pass the image to the SLAM system in track RGBD function
    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    frameCounter++;

    // thanhnc: print frame rate every x second
    if((cv_ptrRGB->header.stamp - lastStamp).toSec() >= 5.0)
    {
        float fps = frameCounter / (cv_ptrRGB->header.stamp - lastStamp).toSec();
        lastStamp = cv_ptrRGB->header.stamp;
        ROS_INFO("Frames per second: %f", fps);
        frameCounter = 0;
    }

    Sophus::SE3f Twc = Tcw.inverse();

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    Eigen::Matrix3f R_mat = Twc.rotationMatrix();
    Eigen::Vector3f t_vec = Twc.translation(); 

    // thanhnc: rotation matrix in ROS coordinate
    // tf::Matrix3x3 R_tf(
    //     R_mat(2, 0), R_mat(2, 1), R_mat(2, 2),
    //     R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
    //     R_mat(1, 0), R_mat(1, 1), R_mat(1, 2)
    // );

    // thanhnc: translation vector in ROS coordinate
    tf::Vector3 t_tf(
        t_vec(2),
        -t_vec(0),
        -t_vec(1)
    );

    // publish tf

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = msg_time;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "camera_rgb_optical_frame_s3m";
    transformStamped.transform.translation.x = t_tf[0];
    transformStamped.transform.translation.y = t_tf[1];
    transformStamped.transform.translation.z = t_tf[2];

    cv::Mat R_tf = ORB_SLAM3::Converter::toCvMat(R_mat);
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(R_tf);
    transformStamped.transform.rotation.x = q[2];
    transformStamped.transform.rotation.y = -q[0];
    transformStamped.transform.rotation.z = -q[1];
    transformStamped.transform.rotation.w = q[3];  

    br.sendTransform(transformStamped);

    // cout << "t: " << t_tf[0] << " " << t_tf[1] << " " << t_tf[2] << endl;
    // double roll, pitch, yaw;
    // R_tf.getRPY(roll, pitch, yaw);
    // cout << "rpy: " << roll*180.0/M_PI << " " << pitch*180.0/M_PI << " " << yaw*180.0/M_PI << endl;

    // tf::Transform tf_transform = tf::Transform(R_tf, t_tf);
    // static tf::TransformBroadcaster tf_broadcaster;
    // tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, msg_time, "
}


