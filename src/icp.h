#ifndef ICP_H
#define ICP_H

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ICP {
    // ROS
    ros::NodeHandle nh_;
    pcl::PointCloud<pcl::PointXYZRGB> output_1_pcl_, output_2_pcl_;

public:
    ICP();
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_1, const sensor_msgs::PointCloud2ConstPtr& cloud_2);
    tf::Transform run_icp();

    ~ICP();
};

#endif //ICP_H
