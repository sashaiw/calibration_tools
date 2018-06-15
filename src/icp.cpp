// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
// ONLY NEEDED FOR DEBUG
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// TF
#include <tf/transform_broadcaster.h>

ros::Publisher pub;
sensor_msgs::PointCloud2 output_merged, output1, output2;
pcl::PointCloud<pcl::PointXYZRGB> output_merged_pcl, output1_pcl, output2_pcl;

// DEBUG
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


void cloud_1_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::fromROSMsg(*input, output1_pcl);
}

void cloud_2_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::fromROSMsg(*input, output2_pcl);
}


Eigen::Matrix4f run_icp(pcl::PointCloud<pcl::PointXYZRGB> cloud_1, pcl::PointCloud<pcl::PointXYZRGB> cloud_2) {
//Eigen::Matrix4f run_icp() {
	ROS_INFO("Attempting ICP.");
	
	// M E M O R Y  E F F I C I E N T
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

	// I detest the fact that I had to copy the clouds but it's the
	// only way I could find to do it so far.
	*cloud_in = output1_pcl;
	*cloud_out = output2_pcl;

	// Segfaults if NaN values not removed.
	std::vector<int> indices1;
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices1);
	pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices2);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	ROS_INFO_STREAM("has converged: " << icp.hasConverged() << 
		      " score: " << icp.getFitnessScore());
	
	// DEBUG VISUALIZATION
	//pcl::PointCloud<PointNT>::Ptr scene (new PointCloudT);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB>);
	//PointCloudT::Ptr scene (new PointCloudT);
	//pcl::visualization::PCLVisualizer visu("Alignment");
	//visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
	//visu.addPointCloud (Final, ColorHandlerT (Final, 0.0, 0.0, 255.0), "object_aligned");
	//visu.spin ();
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(Final, "1");
	viewer.showCloud(cloud_out, "2");
	viewer.showCloud(cloud_in, "3");
	while (!viewer.wasStopped()) { }
	
	return icp.getFinalTransformation();
}

tf::Transform matrix4f_to_tf(Eigen::Matrix4f em) {
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

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "get_transforms");
	ros::NodeHandle nh;

	ros::Subscriber cam1 = nh.subscribe("/rgbd_cam_1/depth_registered/points", 1, cloud_1_cb);
	ros::Subscriber cam2 = nh.subscribe("/rgbd_cam_2/depth_registered/points", 1, cloud_2_cb);

	ros::Rate r(10); // 10 hz
	ROS_INFO("Initializing camera data");
	while (output1_pcl.width == 0 || output2_pcl.width == 0) {
		ros::spinOnce();
		r.sleep();
		ROS_INFO_STREAM("Cam 1: " << output1_pcl.width << ", Cam 2: " << output2_pcl.width);
	}

	ROS_INFO("Camera data initialized.");
	Eigen::Matrix4f icp_transform = run_icp(output1_pcl, output2_pcl);
	ROS_INFO_STREAM("Transform found:\n" << icp_transform);
	
	// TODO: test Matrix4f -> tf
	tf::Transform final_transform = matrix4f_to_tf(icp_transform);
	ROS_INFO_STREAM("TF transform (x, y, z, qx, qy, qz, qw):\n"
			<< final_transform.getOrigin().getX() << " "
			<< final_transform.getOrigin().getY() << " "
			<< final_transform.getOrigin().getZ() << " "
			<< final_transform.getRotation().getX() << " "
			<< final_transform.getRotation().getY() << " "
			<< final_transform.getRotation().getZ() << " "
			<< final_transform.getRotation().getW() << " "
			<< std::endl);

	// TODO: Figure out where transform should be published
	//static tf::TransformBroadcaster br;
	//tf::StampedTransform stamped_transform = tf::StampedTransform(final_transform, ros::Time::now(), "l_cam_center", "r_cam_center");
	//br.sendTransform(stamped_transform);

	return 0;
}
