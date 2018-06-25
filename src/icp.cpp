#include "icp.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>

void ICP::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_1, const sensor_msgs::PointCloud2ConstPtr& cloud_2) {
    pcl::fromROSMsg(*cloud_1, output_1_pcl_);
    pcl::fromROSMsg(*cloud_2, output_2_pcl_);
}