#include "find_chessboard.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/LinearMath/Transform.h>

void print_tf_transform(tf2::Transform tf);
tf::Transform tf22tf(tf2::Transform tf2);
tf2::Transform tf2tf2(tf::Transform tf);

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_wizard");
    ros::NodeHandle nh;

    image_transport::Publisher image_pub_cam1_;
    image_transport::Publisher image_pub_cam2_;

    // Params
    std::string image_topic_1, info_topic_1, image_topic_2, info_topic_2, cloud_topic_1, cloud_topic_2;
    nh.param<std::string>("image_topic_1", image_topic_1, "/rgbd_cam_1/rgb/image_raw");
    nh.param<std::string>("info_topic_1", info_topic_1, "/rgbd_cam_1/rgb/camera_info");
    nh.param<std::string>("image_topic_2", image_topic_2, "/rgbd_cam_2/rgb/image_raw");
    nh.param<std::string>("image_topic_2", info_topic_2, "/rgbd_cam_2/rgb/camera_info");

    nh.param<std::string>("cloud_topic_1", cloud_topic_1, "/rgbd_cam_1/depth_registered/points");
    nh.param<std::string>("cloud_topic_2", cloud_topic_2, "/rgbd_cam_2/depth_registered/points");

    int cb_size_x, cb_size_y;
    double cb_square_size;
    nh.param("chessboard_x", cb_size_x, 6);
    nh.param("chessboard_y", cb_size_y, 7);
    nh.param("chessboard_square_size", cb_square_size, .06);

    ChessboardFinder cf(cb_size_x, cb_size_y, cb_square_size);

    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, image_topic_1, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_1(nh, info_topic_1, 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, image_topic_2, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_2(nh, info_topic_2, 1);


    int nimages;
    nh.param("number_of_images", nimages, 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                            sensor_msgs::Image, sensor_msgs::CameraInfo>
                                                            imageSyncPolicy;
    message_filters::Synchronizer<imageSyncPolicy>
            image_sync(imageSyncPolicy(100), image_sub_2, info_sub_2, image_sub_1, info_sub_1);
    image_sync.registerCallback(boost::bind(&ChessboardFinder::callback, &cf, _1, _2, _3, _4));
    ros::Rate r(30); // 10 hz
    cv::Mat rmat, tvec;

    while (cf.get_nimages() < nimages) {
        ros::spinOnce();
        r.sleep();
        if (cv::waitKey(1) != -1) {
            ROS_INFO_STREAM("Got image " << cf.get_image());
            ros::Duration(0.5).sleep();
        }
    }

    double rms = cf.calibrate(rmat, tvec);
    tf2::Transform tf = ChessboardFinder::cv2tf(rmat, tvec);

    ROS_INFO_STREAM("Calibrated! RMS error: " << rms);
    ROS_INFO_STREAM("Estimate from chessboard: ");
    print_tf_transform(tf);
    return 0;
}

tf::Transform tf22tf(tf2::Transform tf2) {
    tf::Transform tf;
    tf.setRotation(tf::Quaternion(tf2.getRotation().getX(),
                                  tf2.getRotation().getY(),
                                  tf2.getRotation().getZ(),
                                  tf2.getRotation().getW()));
    tf.setOrigin(tf::Vector3(tf2.getOrigin().getX(), tf2.getOrigin().getY(), tf2.getOrigin().getZ()));
    return tf;
}

tf2::Transform tf2tf2(tf::Transform tf) {
    tf2::Transform tf2;
    tf2.setRotation(tf2::Quaternion(tf.getRotation().getX(),
                                    tf.getRotation().getY(),
                                    tf.getRotation().getZ(),
                                    tf.getRotation().getW()));
    tf2.setOrigin(tf2::Vector3(tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ()));
    return tf2;
}

void print_tf_transform(tf2::Transform tf) {
    ROS_INFO_STREAM(   tf.getOrigin().getX() << " "
                    << tf.getOrigin().getY() << " "
                    << tf.getOrigin().getZ() << " "
                    << tf.getRotation().getX() << " "
                    << tf.getRotation().getY() << " "
                    << tf.getRotation().getZ() << " "
                    << tf.getRotation().getW());
}
