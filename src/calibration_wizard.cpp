#include "find_chessboard.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_chessboard");

    image_transport::Publisher image_pub_cam1_;
    image_transport::Publisher image_pub_cam2_;
    ChessboardFinder cf;

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, "/rgbd_cam_1/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_1(nh, "/rgbd_cam_1/rgb/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, "/rgbd_cam_2/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_2(nh, "/rgbd_cam_2/rgb/camera_info", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                            sensor_msgs::Image, sensor_msgs::CameraInfo>
                                                            syncPolicy;
    message_filters::Synchronizer<syncPolicy>
            sync(syncPolicy(100), image_sub_1, info_sub_1, image_sub_2, info_sub_2);

    sync.registerCallback(boost::bind(&ChessboardFinder::callback, &cf, _1, _2, _3, _4));
    ros::Rate r(30); // 10 hz
    cv::Mat rvec, tvec;

    //while (true) {
    //while (!cf.bothVisible()) {
    while (cf.get_nimages() < 4) {
        ros::spinOnce();
        r.sleep();
        if (cv::waitKey(0) != -1) {
            ROS_INFO_STREAM("Got image " << cf.get_image());
            ros::Duration(0.5).sleep();
        }
    }

    ROS_INFO("Found both!");

    double rms = cf.calibrate(rvec, tvec);
    ROS_INFO_STREAM("Calibrated! RMS error: " << rms);

    Eigen::Quaterniond rquat = cf.rvec2quat(rvec);
    ROS_INFO_STREAM("TF transform: " << std::endl
                                     << tvec.at<double>(0,2) << " "
                                     << tvec.at<double>(0,1) << " "
                                     << tvec.at<double>(0,0) << " "
                                     << rquat.x() << " "
                                     << rquat.y() << " "
                                     << rquat.z() << " "
                                     << rquat.w() << " ");

    // TODO: This transform is borked.
    // Possible reasons why it is borked:
    // - intrinsics are borked
    // - rvec->quat is borked
    // - scale is borked
    // - param order is borked
    // - obj_ is borked

    return 0;
}