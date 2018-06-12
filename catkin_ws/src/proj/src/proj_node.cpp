

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <cstdio>
#include <cstdlib>


#include <deque>
#include <vector>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>



using namespace std;

deque <pcl::PointCloud<pcl::PointXYZI>::Ptr> pc_record;
deque <Eigen::Matrix4f> tf_record;
int SIZE = 10;

ros::Publisher pub_processed_cloud;

void publishCloud(ros::Publisher in_publisher, pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);
void print_matrix(Eigen::Matrix4f m);
Eigen::Matrix4f getTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr pcSrc, pcl::PointCloud<pcl::PointXYZI>::Ptr pcTrg);

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
    ROS_INFO("%d points from LiDAR", current_sensor_cloud_ptr->points.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);

    /*for(int i=0 ; i<current_sensor_cloud_ptr->points.size() ; i++) {

    	if( current_sensor_cloud_ptr->points[i].z > 2.0 ) {
    		processed_cloud->points.push_back(current_sensor_cloud_ptr->points[i]);
    	}
    }*/


	//
	bool enough_frames = true;
	Eigen::Matrix4f transf;
	transf.setIdentity();

	if(pc_record.size() >= SIZE) {

		pc_record.pop_front();
		tf_record.pop_front();
	}
	else {
		enough_frames = false;
	}

	pc_record.push_back(current_sensor_cloud_ptr);
	tf_record.push_back(transf);


	if(enough_frames) {

		processed_cloud->points.clear();

		for(int i=0 ; i<SIZE-1 ; i++) {

			tf_record[i] = getTransformation(   pc_record[i], pc_record[ SIZE-1 ]   );
			//tf_record[i] = transf;

			pcl::transformPointCloud(*pc_record[i], *tmp, tf_record[i]);

			*processed_cloud += *tmp;
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
	ros::Subscriber sub = h.subscribe ("/points_raw", 10, velodyne_callback);


	// init
	/*
	for(int i=0 ; i<SIZE ; i++) {
	
		Eigen::Matrix4f I;
		I << 1,0,0,0,
		     0,1,0,0,
			 0,0,1,0,
			 0,0,0,1;
		pcl::PointCloud<pcl::PointXYZI>::Ptr p;

		pc_record.push_back(p);
		tf_record.push_back(I);
	}
	*/
	ROS_INFO("init done");



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

// src --> trg
Eigen::Matrix4f getTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr pcSrc, pcl::PointCloud<pcl::PointXYZI>::Ptr pcTrg) {

	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr trg(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*pcSrc, *src);
	pcl::copyPointCloud(*pcTrg, *trg);

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> crs;
	boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
	Eigen::Matrix4f transf;

	corr_est.setInputSource (src);
	corr_est.setInputTarget (trg);
	corr_est.determineCorrespondences (*correspondences);

	crs.setInputSource(src);
	crs.setInputTarget(trg);

	crs.setInlierThreshold(5.0);
	crs.setMaximumIterations(200);
	crs.setInputCorrespondences(correspondences);

	crs.setRefineModel(true);
	crs.setSaveInliers(true);

	boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
	crs.getCorrespondences(*correspondences_result_rej_sac);
	transf = crs.getBestTransformation();

	std::vector<int> idxInliers;
	crs.getInliersIndices(idxInliers);
	cout << "inlier: " << idxInliers.size() << "  total: "  << pcTrg->points.size() << endl << endl;

	return transf;
	*/
}

void print_matrix(Eigen::Matrix4f m) {

	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", m(0,0), m(0,1), m(0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", m(1,0), m(1,1), m(1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", m(2,0), m(2,1), m(2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", m(0,3), m(1,3), m(2,3));
}

