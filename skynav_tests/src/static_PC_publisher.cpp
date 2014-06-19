#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/package.h>

ros::Publisher pub,pubCloud;

pcl::PCLPointCloud2 read()
{	
	pcl::PCDReader reader;	
	pcl::PointCloud<pcl::PointXYZ> data;
	pcl::PCLPointCloud2 cloud;

	reader.read(ros::package::getPath("skynav_tests")+"/include/EXPORT8.pcd", data);
	
	pcl::toPCLPointCloud2(data, cloud);

	cloud.header.frame_id = "/map";
	return cloud;
}


void publish(pcl::PCLPointCloud2& cloud)
{	
	cloud.header.frame_id = "/map";
	
	pubCloud.publish(cloud);
}


void loop()
{
	pcl::PCLPointCloud2 cloud;

	cloud = read();
	
	publish(cloud);
}


int main (int argc, char** argv)
{
	ROS_INFO("started static publisher");	

	ros::init (argc, argv, "pcl_publisher");
	
	ros::NodeHandle nh_control = ros::NodeHandle("/control");
	
	pubCloud = nh_control.advertise<pcl::PCLPointCloud2> ("cloud", 1);

	ros::Rate loop_rate(2);	
	while(ros::ok)
	{		
		loop();	
		loop_rate.sleep();		
	}
	return 0;	
}
