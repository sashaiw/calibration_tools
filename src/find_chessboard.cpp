#include "find_chessboard.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const std::string OPENCV_WINDOW_1 = "Camera 1";
static const std::string OPENCV_WINDOW_2 = "Camera 2";

ChessboardFinder::ChessboardFinder() {
    found_[0] = false;
    found_[1] = false;

    // Create model of chessboard
    patternSize_ = cv::Size(6, 7);
    squareSize_ = .06;
    for (int j = 0; j < patternSize_.height; j++) {
        for (int k = 0; k < patternSize_.width; k++) {
            obj_.push_back(cv::Point3f((float)k * squareSize_, (float)j * squareSize_, 0));
        }
    }

    cv::namedWindow(OPENCV_WINDOW_1);
    cv::namedWindow(OPENCV_WINDOW_2);
}

void ChessboardFinder::callback(const sensor_msgs::ImageConstPtr& msg_cam1,
                                const sensor_msgs::CameraInfoConstPtr& info_msg_cam1,
                                const sensor_msgs::ImageConstPtr& msg_cam2,
                                const sensor_msgs::CameraInfoConstPtr& info_msg_cam2) {
    cv_bridge::CvImagePtr cv_ptr[2];

    try {
        cv_ptr[0] = cv_bridge::toCvCopy(msg_cam1, sensor_msgs::image_encodings::BGR8);
        cv_ptr[1] = cv_bridge::toCvCopy(msg_cam2, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptr[0]->image.cols != cv_ptr[1]->image.cols || cv_ptr[0]->image.rows != cv_ptr[1]->image.rows) {
        ROS_ERROR("Images are not same size.");
        return;
    }
    imageSize = cv::Size(cv_ptr[0]->image.cols, cv_ptr[0]->image.rows);

    cam_model_[0].fromCameraInfo(info_msg_cam2);
    cam_model_[1].fromCameraInfo(info_msg_cam2);

    found_[0] = cv::findChessboardCorners(cv_ptr[0]->image, patternSize_, corners_[0],
                                        cv::CALIB_CB_ADAPTIVE_THRESH +
                                        cv::CALIB_CB_NORMALIZE_IMAGE +
                                        cv::CALIB_CB_FAST_CHECK);
    found_[1] = cv::findChessboardCorners(cv_ptr[1]->image, patternSize_, corners_[1],
                                          cv::CALIB_CB_ADAPTIVE_THRESH +
                                          cv::CALIB_CB_NORMALIZE_IMAGE +
                                          cv::CALIB_CB_FAST_CHECK);

    if (found_[0]) {
        cv::drawChessboardCorners(cv_ptr[0]->image, patternSize_, corners_[0], found_[0]);
    }
    if (found_[1]) {
        cv::drawChessboardCorners(cv_ptr[1]->image, patternSize_, corners_[1], found_[1]);
    }

    cv::putText(cv_ptr[0]->image, "cam 1", cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255), 1, 8, false);
    cv::putText(cv_ptr[1]->image, "cam 2", cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255), 1, 8, false);
    cv::imshow(OPENCV_WINDOW_1, cv_ptr[0]->image);
    cv::imshow(OPENCV_WINDOW_2, cv_ptr[1]->image);
}

bool ChessboardFinder::bothVisible() {
    return found_[0] && found_[1];
}

int ChessboardFinder::get_image() {
    if (!bothVisible()) { return -1; }
    imagePoints_[0].push_back(corners_[0]);
    imagePoints_[1].push_back(corners_[1]);
    objectPoints_.push_back(obj_);

    return (int)imagePoints_[0].size();
}

int ChessboardFinder::get_nimages() {
    return (int)imagePoints_[0].size();
}

double ChessboardFinder::calibrate(cv::Mat& rotationMatrix, cv::Mat& transformVector) {
    if (imagePoints_[0].empty() || imagePoints_[1].empty()) { return -1; }
    cv::Mat essentialMatrix, fundamentalMatrix;
    double rms = cv::stereoCalibrate(objectPoints_,
                                     imagePoints_[0], imagePoints_[1],
                                     cam_model_[0].intrinsicMatrix(), cam_model_[0].distortionCoeffs(),
                                     cam_model_[1].intrinsicMatrix(), cam_model_[1].distortionCoeffs(),
                                     imageSize,
                                     rotationMatrix, transformVector, essentialMatrix, fundamentalMatrix,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6),
                                     CV_CALIB_FIX_INTRINSIC);
    return rms;
}

tf2::Quaternion ChessboardFinder::rvec2tfquat(cv::Mat &rmat) {
    // Let's just pretend there's no singularity, okay?
    double r = atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));
    double p = atan2(-rmat.at<double>(2, 0), sqrt(pow(rmat.at<double>(2, 1), 2) + pow(rmat.at<double>(2, 2), 2)));
    double y = atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
    tf2::Quaternion pose_quat;
    pose_quat.setRPY(r, p, y);
    ROS_INFO_STREAM("YPR: " << y << " " << p << " " << r << std::endl);
    return pose_quat;
}

Eigen::Quaterniond ChessboardFinder::rvec2quat(cv::Mat &rvec) {
    Eigen::Matrix3d rvec_eigen;
    cv::cv2eigen(rvec, rvec_eigen);
    Eigen::Quaterniond quat(rvec_eigen);
    return quat;
}

ChessboardFinder::~ChessboardFinder() {
    cv::destroyWindow(OPENCV_WINDOW_1);
    cv::destroyWindow(OPENCV_WINDOW_2);
}