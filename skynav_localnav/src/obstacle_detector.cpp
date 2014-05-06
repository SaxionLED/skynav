#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <skynav_msgs/RangeDefinedArray.h>
#include <skynav_msgs/RangeDefined.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/Object.h>
#include <set>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <skynav_msgs/Objects.h>

#define SIMULATOR 						false 	// set this to true when using the simulator, false indicates the actual robot with lower specs is used
#define ROBOTRADIUS 					0.5 	//the radius of the robot in meters. TODO get this from somewhere robot dependent
#define MAX_SENSORDIST 					4		//the outer range of the sensors in meters

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

struct compPoint {

    bool operator() (const Point32& first, const Point32& second) const {
        if (first.x < second.x)
            return true;
        else if (first.x > second.x)
            return false;
        else // if x the same
            if (first.y < second.y)
            return true;
        else
            return false; // if x the same and y the same or second y greater
    }
};

int mObjectsFound = 0;

ros::Publisher pubObjects, pubObstacles, pubSensorData;
ros::ServiceClient servClientCurrentPose;

laser_geometry::LaserProjection mLaserProjector;
tf::TransformListener* mTransformListener;

set<Point32, compPoint> mSensorData;
vector<PointCloud> mObjects;	//objects in memory

Pose getCurrentPose() {

    Pose currentPose;

    skynav_msgs::current_pose poseService;

    if (servClientCurrentPose.call(poseService)) {

        currentPose = poseService.response.pose;

        //        ROS_INFO("Current pose (x %f) (y %f) (theta %f)", poseService.response.pose.x, poseService.response.pose.y, poseService.response.pose.theta);
    } else {
        ROS_ERROR("Failed to call current_pose service from obstacle detector");
    }

    return currentPose;
}

//determine if a point already exist in a list of points.
static bool pointExists(const Point32* a, vector<Point32>* vec) {
	const double xyRange = 0.02; //within 2 cm, it is concidered the same coordinate
    vector<Point32>::iterator it;

    for (it = vec->begin(); it != vec->end(); ++it) {
		
		
		if(((*it).x == a->x && (*it).y == a->y)){	//point coordinates are exactly the same
            return true;
		}
		
		double xyDistance = sqrt(pow((*it).x - a->x,2) + (pow((*it).y - a->y,2)));
		if (xyDistance <= xyRange){					//point exist by close proximity ( taken accuracy of sensor in consideration)
			return true;							
		}
    }	

    return false;									//point does not already exist
}

//determine if two points are within a certain distance to eachother
static bool pointInRange(const Point32* a, const Point32* b, const double searchDistance) {

		double xyDistance = sqrt(pow((a->x -b->x),2) + pow((a->y - b->y),2));	//use pythagoras to determine if coordinates of a are within certain range of b
		if(xyDistance <= searchDistance){														
			return true;
		}
    
    return false;
}

//truncate values (in meters) to certain precision
float truncateValue(const float value){
	//return floorf(value*1000)/1000; //mm
	return floorf(value*100)/100; //cm 
}

void subSensorCallback(const skynav_msgs::RangeDefinedArray::ConstPtr& msg) {	// for x80 sonar/IR sensors

    Pose currentPose = getCurrentPose();

    PointCloud pointCloud;

    for (uint i = 0; i < msg->ranges.size(); i++) {

        skynav_msgs::RangeDefined rangeMsg = msg->ranges.at(i);

        if (rangeMsg.range.range > rangeMsg.range.min_range && rangeMsg.range.range < rangeMsg.range.max_range) { // skip if not within limits

            double alpha = rangeMsg.angleFromCenter + currentPose.orientation.z; // sensor angle + current pose angle
            double distance = rangeMsg.distanceFromCenter + rangeMsg.range.range; // sensor range + offset from center

            double objectX = (cos(alpha) * distance) + currentPose.position.x; // add current pose x
            double objectY = (sin(alpha) * distance) + currentPose.position.y + rangeMsg.yOffsetFromCenter; // y

            Point32 p;
            p.x = truncateValue(objectX); // truncate values to mm (because sonar and IR return very specific values that fall outside their precision)
            p.y = truncateValue(objectY);

            pointCloud.points.push_back(p);
        }
    }

    if (pointCloud.points.size() > 0) {

        pointCloud.header.stamp = ros::Time::now();
        pointCloud.header.frame_id = "/base_link";

        pubSensorData.publish(pointCloud);
    }
}
//receive laserscan data and convert to pointcloud
//move this function to data_verifier.cpp !?
void subLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)	{
	
	 if(!mTransformListener->waitForTransform(scan_in->header.frame_id, "/map", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0)))	{
		return;
	}	
	
	PointCloud pointCloud;
	try
	{
		mLaserProjector.transformLaserScanToPointCloud("/map", *scan_in, pointCloud, *mTransformListener);
	}
	catch (tf::TransformException& e)
	{
		ROS_ERROR("obstacle_detector - %s", e.what());
	}

	if (pointCloud.points.size() > 0) {
		
		//truncate pointcloud coordinates to mm precision.
		for(vector<Point32>::iterator it = pointCloud.points.begin(); it != pointCloud.points.end(); it++){
			(*it).x = truncateValue((*it).x );
			(*it).y = truncateValue((*it).y);
			(*it).z = truncateValue((*it).z);
		}			

        pointCloud.header.stamp = ros::Time::now();
        pointCloud.header.frame_id = "/map";

        pubSensorData.publish(pointCloud);
    }
	
}

//detect objects within sensor range, compare to known objects and add/merge in object list
void subObjectDetectionCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    // some vars
    const double xySearchDistance = 0.1; // 10cm        //TODO should be settable somewhere 
	
    // now we can check our newest points with the ones already in memory, find other points that are nearby and points near the found points
    for (uint i = 0; i < msg->points.size(); ++i) {
    
        PointCloud foundPoints;
		set<Point32>::iterator sensor_it;

        // check if objects were found nearby
        vector<PointCloud>::iterator firstObjectIt;

        for (firstObjectIt = mObjects.begin(); firstObjectIt != mObjects.end(); ++firstObjectIt) {

            for (uint firstPoints = 0; firstPoints < (*firstObjectIt).points.size(); ++firstPoints) {  
                
                if( pointInRange(&((*firstObjectIt).points.at(firstPoints)), &(msg->points.at(i)), xySearchDistance))   { // if point within range of another point
                
                    // if true, the coordinate is within xySearchDistance of any of the coordinates contained in the object and we should add it to that object
                    bool addOriginalPoint = true;

                    if (pointExists(&(msg->points.at(i)), &((*firstObjectIt).points))) { // do not add duplicates
                        addOriginalPoint = false; // point is already in the object, prevent adding it twice 
                    }

                    // point is added to object, now check if the point is also in range of another object
                    vector<PointCloud>::iterator secondObjectIt;

                    for (secondObjectIt = firstObjectIt + 1; secondObjectIt != mObjects.end();) {
                        bool erasedObject = false;

                        for (uint secondPoints = 0; secondPoints < (*secondObjectIt).points.size(); ++secondPoints) {
                            if( pointInRange(&((*secondObjectIt).points.at(secondPoints)), &(msg->points.at(i)), xySearchDistance))   {
                            
                                // another object in range was found

                                // to prevent the original point being added twice
                                if (pointExists( &(msg->points.at(i)), &((*secondObjectIt).points))) { // do not add duplicates
                                    addOriginalPoint = false;
                                }

                                // merge the two objects
                                (*firstObjectIt).points.insert((*firstObjectIt).points.end(), (*secondObjectIt).points.begin(), (*secondObjectIt).points.end()); // add all points from 2nd vector to the first
                                (*secondObjectIt).points.clear(); // empty vector

                                erasedObject = true;
                                secondObjectIt = mObjects.erase(secondObjectIt); // remove locally, can re-use the index for it
                                //ROS_INFO("merged objects");

                                break; // if point is in range, this needs not be executed again for that object

                                // repeat until all objects have been checked
                            }
                        }

                        if (!erasedObject) {
                            ++secondObjectIt;
                        }
                    }

                    if (addOriginalPoint) {
                        (*firstObjectIt).points.push_back(msg->points.at(i));
                    }

                    goto nextNewPoint;	//point has been processed. jump to next point in pointcloud
                }
            }
        }

        // no objects nearby were found, check if there are any single points nearby to create a new object out of

        for (sensor_it = mSensorData.begin(); sensor_it != mSensorData.end(); ++sensor_it) {
			
			if( pointInRange( &(*sensor_it), &(msg->points.at(i)), xySearchDistance))   {       //if point near other point
                foundPoints.points.push_back((*sensor_it));
                mSensorData.erase(sensor_it--); // is this safe?
                }
        }

        if (foundPoints.points.size() > 0) { // if points were found nearby

            foundPoints.points.push_back(msg->points.at(i)); // add the reference point, since it was not added anywhere yet
            
            foundPoints.header.stamp = ros::Time(0);
            foundPoints.header.frame_id = "/map";
            
            mObjects.push_back(foundPoints);

        } else { // no nearby points found
            // if no object found either, save the point for future searching
            mSensorData.insert(msg->points.at(i)); // note that a set is used because this prevents duplicates
        }
        
        // this skips everything and restarts the first for loop. should only be called when a new point (from msg) has been added to an object.
        nextNewPoint:
        asm("NOP");
        
    }    
}

//discard objects outside of certain range, or when a new scan has been done
void forgetObjects(){
	if(!SIMULATOR){	//if runs on actual robot, use only transient data from each laser scan. dont keep objects in memory
		mObjects.clear();
	}else{
		try{
		Pose currentPose = getCurrentPose();
		
		for(vector<PointCloud>::iterator objectIt = mObjects.begin(); objectIt!= mObjects.end(); ++objectIt){
			for(vector<Point32>::iterator pointIt = (*objectIt).points.begin(); pointIt != (*objectIt).points.end(); ++pointIt){
				double xyDistance = sqrt(pow(currentPose.position.x - (*pointIt).x,2) + (pow(currentPose.position.y - (*pointIt).y,2)));
					if (xyDistance > MAX_SENSORDIST){					
						(*objectIt).points.erase( pointIt--);	//carefull here	!					
					}
				}
				if((*objectIt).points.empty()){
					mObjects.erase(objectIt--); //careful here!
				}
					
			}
		}catch(exception& e){
			ROS_ERROR("ERROR forgetting objects: %s",e.what());
		}
	}
}

//publish all known objects
void publishObjects()	{
		
	//ROS_INFO("%d objects", mObjects.size());
	vector<PointCloud> obstacles;
	vector<PointCloud>::iterator it;
		
	for(it = mObjects.begin(); it != mObjects.end(); ++it)	{
		
		//(*it).header.stamp = ros::Time::now();
		//ROS_INFO("contains %d", (*it).points.size());
		pubObjects.publish( (*it) );
		
		obstacles.push_back((*it));
	}
	skynav_msgs::Objects msg;
	msg.objects = obstacles;
	pubObstacles.publish(msg);
	
	forgetObjects();	

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");
    ros::NodeHandle n_SLAM("/slam");

    //pubs
    pubSensorData = n.advertise<sensor_msgs::PointCloud>("sensor_data", 1024);
    pubObjects = n.advertise<PointCloud>("objects", 1024);
    pubObstacles = n.advertise<skynav_msgs::Objects>("obstacles", 1024);

    //subs
    ros::Subscriber subSensors = n_control.subscribe("sensors", 10, subSensorCallback); // raw unprocessed sensor values
    ros::Subscriber subSensorData = n.subscribe("sensor_data", 10, subObjectDetectionCallback);
    ros::Subscriber subLaser = n_control.subscribe("laser_scan", 10, subLaserScanCallback);

    //services
    servClientCurrentPose = n_SLAM.serviceClient<skynav_msgs::current_pose>("current_pose");
    
    mTransformListener = new tf::TransformListener();

    ros::Rate loop_rate(1);
	
	while (ros::ok()) {
            
		ros::spinOnce();
		
		publishObjects();
		
		loop_rate.sleep();
		
	}
            
    delete mTransformListener;


    return 0;
}
