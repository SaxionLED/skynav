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

set<Point32, compPoint> mSensorData;
vector<skynav_msgs::Object> mObjects;

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

static bool pointExists(const Point32* a, vector<Point32>* vec) {

    vector<Point32>::iterator it;

    for (it = vec->begin(); it != vec->end(); ++it) {

        if ((*it).x == a->x && (*it).y == a->y)
            return true;
    }

    return false;
}

static bool pointInRange(const Point32* a, const Point32* b, const double searchDistance) {

    if (
            a->x <= b->x + searchDistance &&    //TODO could be replaced by magnitude (circular search distance)
            a->x >= b->x - searchDistance &&    // currently it searches within a square
            a->y <= b->y + searchDistance &&
            a->y >= b->y - searchDistance) {

        return true;
    }

    return false;
}

void subSensorCallback(const skynav_msgs::RangeDefinedArray::ConstPtr& msg) {

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
            p.x = double(int(objectX * 1000)) / 1000; // truncate values to mm
            p.y = double(int(objectY * 1000)) / 1000;

            pointCloud.points.push_back(p);
        }
    }

    if (pointCloud.points.size() > 0) {

        pointCloud.header.stamp = ros::Time::now();
        pointCloud.header.frame_id = "/base_link";

        pubSensorData.publish(pointCloud);
    }
}

void subObjectDetectionCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    // some vars
    const double xySearchDistance = 0.1; // 10cm        //TODO should be settable somewhere 

    skynav_msgs::Object objectHandle;

    // now we can check our newest points with the ones already in memory, find other points that are nearby and points near the found points
    for (uint i = 0; i < msg->points.size(); ++i) {
        
        vector<Point32> foundPoints;

        // check if objects were found nearby
        vector<skynav_msgs::Object>::iterator firstObjectIt;

        for (firstObjectIt = mObjects.begin(); firstObjectIt != mObjects.end(); ++firstObjectIt) {

            for (uint firstPoints = 0; firstPoints < (*firstObjectIt).points.size(); ++firstPoints) {

                if( pointInRange(&((*firstObjectIt).points.at(firstPoints)), &(msg->points.at(i)), xySearchDistance))   { // if point within range of another point
                
                    // if true, the coordinate is within xySearchDistance of any of the coordinates contained in the object and we should add it to that object
                    bool addOriginalPoint = true;

                    if (pointExists(&(msg->points.at(i)), &((*firstObjectIt).points))) { // do not add duplicates
                        addOriginalPoint = false; // point is already in the object, prevent adding it twice 
                    }

                    objectHandle = (*firstObjectIt); // save object for publishing later

                    // point is added to object, now check if the point is also in range of another object
                    vector<skynav_msgs::Object>::iterator secondObjectIt;

                    for (secondObjectIt = firstObjectIt + 1; secondObjectIt != mObjects.end();) {
                        bool erasedObject = false;

                        for (uint secondPoints = 0; secondPoints < (*secondObjectIt).points.size(); ++secondPoints) {
                            if( pointInRange(&((*secondObjectIt).points.at(secondPoints)), &(msg->points.at(i)), xySearchDistance))   {
                            
                                // another object in range was found

                                // to prevent the original point being added twice
                                if (pointExists( &(msg->points.at(i)), &((*secondObjectIt).points))) { // do not add duplicates
                                    addOriginalPoint = false;
                                }

                                //                                ROS_INFO("merged objects");

                                // merge the two objects
                                (*firstObjectIt).points.insert((*firstObjectIt).points.end(), (*secondObjectIt).points.begin(), (*secondObjectIt).points.end()); // add all points from 2nd vector to the first
                                (*secondObjectIt).points.clear(); // empty vector

                                // publish the second object with an empty points[], this will cause python to drop the object
                                pubObjects.publish((*secondObjectIt));


                                erasedObject = true;
                                secondObjectIt = mObjects.erase(secondObjectIt); // remove locally, can re-use the index for it

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

                    goto objectFound;
                }
            }
        }

        // no objects nearby were found, check if there are any single points nearby to create a new object out of

        set<Point32>::iterator sensor_it;
        for (sensor_it = mSensorData.begin(); sensor_it != mSensorData.end(); ++sensor_it) {

            if( pointInRange( &(*sensor_it), &(msg->points.at(i)), xySearchDistance))   {       //if point near other point

                foundPoints.push_back((*sensor_it));
                mSensorData.erase(sensor_it--); // is this safe?
            }
        }

        if (foundPoints.size() > 0) { // if points were found nearby

            foundPoints.push_back(msg->points.at(i)); // add the reference point, since it was not added anywhere yet

            // init new object

            objectHandle.uniqueID = mObjects.size();
            objectHandle.points = foundPoints;
            mObjects.push_back(objectHandle);

        } else { // no nearby points found

            // if no object found either, save the point for future searching
            mSensorData.insert(msg->points.at(i)); // note that a set is used because this prevents duplicates
            return; // we dont want to publish because there is no valid object
        }
    }

objectFound: // so the rest of the for loops are skipped rather than executed pointlessly


    pubObjects.publish(objectHandle); // this causes issues in the python GUI (cannot find the point[] for unknown reason), so workaround: publish each point seperately with an ID

}

void subObstacleDetectionCallback(const skynav_msgs::Object::ConstPtr & msg) {
    // receive all objects and determine if they are obstacles (on the path)

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n("/localnav");
    ros::NodeHandle n_control("/control");
    ros::NodeHandle n_SLAM("/SLAM");

    //pubs
    pubSensorData = n.advertise<sensor_msgs::PointCloud>("sensor_data", 1024);
    pubObjects = n.advertise<skynav_msgs::Object>("objects", 256);
    pubObstacles = n.advertise<skynav_msgs::Object>("obstacles", 256);

    //subs
    ros::Subscriber subSensors = n_control.subscribe("sensors", 1024, subSensorCallback); // raw unprocessed sensor values
    ros::Subscriber subSensorData = n.subscribe("sensor_data", 256, subObjectDetectionCallback);
    //    ros::Subscriber subObjects = n.subscribe("objects", 256, subObstacleDetectionCallback);

    //services
    servClientCurrentPose = n_SLAM.serviceClient<skynav_msgs::current_pose>("current_pose");

    ros::spin();

    //    ros::Rate loop_rate(10);    // to update every 100ms rather than whenever new sensor data comes in (which is far too often))
    //
    //    while (ros::ok()) {
    //        
    //        ros::spinOnce();
    //        
    //        loop_rate.sleep();
    //    }
    //    


    return 0;
}
