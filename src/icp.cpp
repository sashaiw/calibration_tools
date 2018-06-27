#include "icp.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

ICP::ICP() {

}

ICP::~ICP() {

}

tf::Transform ICP::run_icp(tf::Transform best_guess) {
    ros::Rate r(10);
    while (output_1_pcl_.width == 0 || output_2_pcl_.width == 0) {
        ros::spinOnce();
        r.sleep();
        ROS_INFO_STREAM("Cam 1: " << output_1_pcl_.width << ", Cam 2: " << output_1_pcl_.width);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_in = output_1_pcl_;
    *cloud_out = output_2_pcl_;

    //pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
    //viewer1.showCloud(cloud_out, "2");
    //viewer1.showCloud(cloud_in, "3");
    //while (!viewer1.wasStopped()) { }

    // Segfaults if NaN values not removed.
    std::vector<int> indices1;
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices1);
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices2);

    pcl_ros::transformPointCloud(*cloud_in, *cloud_in, best_guess);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
    icp.align(*Final);
    ROS_INFO_STREAM("has converged: " << icp.hasConverged() <<
                    " score: " << icp.getFitnessScore());

    pcl::visualization::CloudViewer viewer("Cloud");
    viewer.showCloud(Final, "1");
    viewer.showCloud(cloud_out, "2");
    //viewer.showCloud(cloud_in, "3");
    while (!viewer.wasStopped()) { }

    return matrix4f_to_tf(icp.getFinalTransformation());
}

void ICP::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_1, const sensor_msgs::PointCloud2ConstPtr& cloud_2) {
    pcl::fromROSMsg(*cloud_1, output_1_pcl_);
    pcl::fromROSMsg(*cloud_2, output_2_pcl_);
}

tf::Transform ICP::matrix4f_to_tf(Eigen::Matrix4f em) {
    tf::Vector3 origin;
    origin.setValue(static_cast<double>(em(0,3)), static_cast<double>(em(1,3)), static_cast<double>(em(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(em(0,0)), static_cast<double>(em(0,1)), static_cast<double>(em(0,2)),
                  static_cast<double>(em(1,0)), static_cast<double>(em(1,1)), static_cast<double>(em(1,2)),
                  static_cast<double>(em(2,0)), static_cast<double>(em(2,1)), static_cast<double>(em(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);

    return transform;
}