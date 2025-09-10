/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
 *
 */

#include "common.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber() {};

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void initialize_offset();

    tf::Transform cam_to_world_offset;
    bool offset_initialized;

private:
    ros::Time lastStamp;
    int frameCounter = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ROS_INFO_STREAM("Node name: " << node_name);

    ros::NodeHandle node_handler;

    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/cam_to_world", cam_to_world, "camera_link");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    igb.initialize_offset();

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_img(node_handler, "/camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    // ROS_INFO_STREAM("Tracking Time: " << (ros::Time::now() - msg_time).toSec() << " seconds");

    publish_topics(msg_time, cam_to_world_offset);

    frameCounter++;

    // Print frame rate every x second
    // if ((cv_ptrRGB->header.stamp - lastStamp).toSec() >= 5.0)
    // {
    //     float fps = frameCounter / (cv_ptrRGB->header.stamp - lastStamp).toSec();
    //     lastStamp = cv_ptrRGB->header.stamp;
    //     ROS_INFO("Frames per second: %f", fps);
    //     frameCounter = 0;
    // }

    // Sophus::SE3f Twc = Tcw.inverse();

    // Suppose Tcw is the pose from ORB-SLAM3 (Sophus::SE3f)
    // Convert Sophus::SE3f to tf::Transform
    // Eigen::Matrix3f R_mat = Twc.rotationMatrix();
    // Eigen::Vector3f t_vec = Twc.translation();

    // tf::Matrix3x3 R_cv(
    //     R_mat(2, 0), R_mat(2, 1), R_mat(2, 2),
    //     R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
    //     R_mat(1, 0), R_mat(1, 1), R_mat(1, 2));

    // tf::Vector3 t_tf(
    //     -t_vec(2),
    //     t_vec(0),
    //     -t_vec(1));

    // ROS_WARN_STREAM_THROTTLE(5.0, "Tcw = \n"
    //                                   << Tcw.matrix() << "\n ");
    

    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;
    // transformStamped.header.stamp = msg_time;
    // transformStamped.header.frame_id = "world";
    // transformStamped.child_frame_id = "test";
    // transformStamped.transform.translation.x = t_tf[0];
    // transformStamped.transform.translation.y = t_tf[1];
    // transformStamped.transform.translation.z = t_tf[2];

    // cv::Mat R_tf = ORB_SLAM3::Converter::toCvMat(R_mat);
    // vector<float> q = ORB_SLAM3::Converter::toQuaternion(R_tf);
    // transformStamped.transform.rotation.x = q[2];
    // transformStamped.transform.rotation.y = q[0];
    // transformStamped.transform.rotation.z = -q[1];
    // transformStamped.transform.rotation.w = q[3];

    // br.sendTransform(transformStamped);

    // tf::Transform iraf_to_cam_transform = tf::Transform(R_cv, t_tf);
    // tf::Transform cam_to_world_transform = iraf_to_cam_transform;
    // cam_to_world_transform = cam_to_world_offset;

    // static tf::TransformBroadcaster br2;
}

void ImageGrabber::initialize_offset()
{
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        // Wait for up to 5 seconds for the transform to be available
        listener.waitForTransform("/world", cam_to_world, ros::Time(0), ros::Duration(20.0));
        listener.lookupTransform("/world", cam_to_world, ros::Time(0), transform);
        cam_to_world_offset = tf::Transform(transform.getRotation(), transform.getOrigin());
        offset_initialized = true;
        ROS_WARN_STREAM_ONCE("Offset from " << cam_to_world << " to /world_frame initialized:\n"
                             << "Translation: " << cam_to_world_offset.getOrigin().x() << ", "
                             << cam_to_world_offset.getOrigin().y() << ", "
                             << cam_to_world_offset.getOrigin().z());
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Could not get initial offset from /kinect_frame to /world_frame: %s", ex.what());
        offset_initialized = false;
    }
}