#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h> 
#include <ros/package.h>

ros::Publisher pub,pubCloud;
tf::TransformListener* mTransformListener;


pcl::PointCloud<pcl::PointXYZI> read()
{	
	pcl::PCDReader reader;	
	pcl::PointCloud<pcl::PointXYZI> cloud;

	reader.read(ros::package::getPath("skynav_tests")+"/include/EXPORT8.pcd", cloud);

	cloud.header.frame_id = "/laser_link";
	return cloud;
}


pcl::PCLPointCloud2 transform(const pcl::PointCloud<pcl::PointXYZI>& input)
{	
	tf::StampedTransform transform;	
	pcl::PointCloud<pcl::PointXYZI> cloudTransformed;
	pcl::PCLPointCloud2 output;

	try
	{
		mTransformListener->lookupTransform("/map", "/laser_link", ros::Time(0), transform);
		pcl_ros::transformPointCloud(input, cloudTransformed, transform);
	}
	catch (tf::TransformException& e)
	{
		ROS_ERROR("%s", e.what());
			
		cloudTransformed = input;
	}	
	
	try{
		pcl::toPCLPointCloud2(cloudTransformed, output);
	}
	catch(std::exception& e)
	{
		ROS_ERROR("%s",e.what());
	}
	
	return output;
}


void publish(pcl::PCLPointCloud2& cloud)
{	
	cloud.header.frame_id = "/map";
	
	pubCloud.publish(cloud);
	
	//ROS_INFO("Original pointCloud dimensions: %d" , cloud.width * cloud.height );
}


void loop()
{
	pcl::PointCloud<pcl::PointXYZI> pclpointcloud;
	pcl::PCLPointCloud2 pclpointcloud2;

	pclpointcloud = read();
	pclpointcloud2 = transform(pclpointcloud);
	
	publish(pclpointcloud2);
}


int main (int argc, char** argv)
{
	ROS_INFO("started publisher");	

	ros::init (argc, argv, "pcl_publisher");
	
	ros::NodeHandle nh_control = ros::NodeHandle("/control");
	
    mTransformListener = new tf::TransformListener();

	pubCloud = nh_control.advertise<pcl::PCLPointCloud2> ("cloud", 1);

	ros::Duration(1).sleep();

	ros::Rate loop_rate(2);	
	while(ros::ok)
	{		
		loop();	
		loop_rate.sleep();		
	}
	return 0;	
}
