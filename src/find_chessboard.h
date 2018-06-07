#ifndef FIND_CHESSBOARD_H
#define FIND_CHESSBOARD_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>

class ChessboardFinder {
    // ROS
    ros::NodeHandle nh_;
    image_geometry::PinholeCameraModel cam_model_[2];

    // Chessboard Model
    cv::Size patternSize_;
    std::vector < cv::Point3f > obj_;
    std::vector < std::vector< cv::Point3f> > objectPoints_;
    float squareSize_;

    // Cams
    bool found_[2];
    std::vector< std::vector< cv::Point2f> > imagePoints_[2];
    std::vector<cv::Point2f> corners_[2];
    cv::Size imageSize;

public:
    ChessboardFinder();
    void callback(const sensor_msgs::ImageConstPtr& msg_cam1,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_cam1,
                                    const sensor_msgs::ImageConstPtr& msg_cam2,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_cam2);
    bool bothVisible();
    int get_image();
    int get_nimages();
    void print_debug_shit();
    double calibrate(cv::Mat& rotationMatrix, cv::Mat& transformVector);
    static tf2::Quaternion rvec2tfquat(cv::Mat &rmat);
    static Eigen::Quaterniond rvec2quat(cv::Mat &rvec);
    ~ChessboardFinder();
};

#endif // FIND_CHESSBOARD_H
