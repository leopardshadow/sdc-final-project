

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


ros::Publisher pub_processed_cloud;

void publishCloud(ros::Publisher in_publisher, pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);



void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
    ROS_INFO("%d points from LiDAR", current_sensor_cloud_ptr->points.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    for(int i=0 ; i<current_sensor_cloud_ptr->points.size() ; i++) {

    	if(current_sensor_cloud_ptr->points[i].z > 1.5 && current_sensor_cloud_ptr->points[i].intensity > 80) {
    		processed_cloud->points.push_back(current_sensor_cloud_ptr->points[i]);
    	}
    }
        
    publishCloud(pub_processed_cloud, processed_cloud);
}


int main(int argc, char** argv) {


	// Initialize ROS
	ros::init (argc, argv, "proj_node");

	ros::NodeHandle h;
	ros::NodeHandle private_nh("~");


	pub_processed_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_processed",10);

	ros::Subscriber sub = h.subscribe ("/points_raw", 1, velodyne_callback);

	// Spin
    ros::spin ();
}



void publishCloud(ros::Publisher in_publisher, pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = "velodyne";
    in_publisher.publish(cloud_msg);
}