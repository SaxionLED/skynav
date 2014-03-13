#include <ros/ros.h>
#include <skynav_msgs/current_pose.h>
#include <string.h>
#include <list>
#include <boost/algorithm/string.hpp>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// defines
#define MOTION_VELOCITY                	0.2                      // speed in m/s
#define TURN_VELOCITY					0.5						// turn speed in rad/s
#define ANGLE_ERROR_ALLOWED             M_PI / 180 * 0.5          // rads	// if too small, the robot may rotate infinitely
#define DISTANCE_ERROR_ALLOWED          0.1                    // in meters

#define ROBOT_WAYPOINT_ACCURACY         false                   // if true, robot will continue trying to reach target goal within error values until proceeding to next target

enum NAVIGATION_STATE { //TODO move this into a shared container (skynav_msgs perhaps?)
    NAV_READY = 0,
    NAV_MOVING = 1,
    NAV_AVOIDING = 2,
    NAV_POSE_REACHED = 3,
    NAV_ERROR = 4,
    NAV_UNREACHABLE = 5,
    NAV_OBSTACLE_DETECTED = 6,
    NAV_CHECKING_OBSTACLE = 7
};

using namespace geometry_msgs;
using namespace std;

// VARS
ros::NodeHandle* mNode;
ros::NodeHandle* mNodeSLAM;

ros::ServiceClient servClientCurrentPose;
ros::Publisher pubCmdVel, pubNavigationState, pubTargetPoseStamped;

list<PoseStamped>* mCurrentPath = new list<PoseStamped>;
double mEndOrientation = 0;

ros::Timer mCmdVelTimeout;
uint8_t mNavigationState = NAV_READY; // initial state

void subCheckedWaypointsCallback(const nav_msgs::Path::ConstPtr& msg) {

    //    for (uint i = 0; i < msg->poses.size(); i++) { //copy path, drop stamps, dont need em
    //
    //        mCurrentPath->push_back(msg->poses.at(i));
    //    }

    mCurrentPath = new list<PoseStamped>(msg->poses.begin(), msg->poses.end());

    mEndOrientation = mCurrentPath->back().pose.orientation.z;

    ROS_INFO("path received by motion control");
}

void subNavigationStateCallback(const std_msgs::UInt8::ConstPtr& msg) {

    //    ROS_INFO("NAV STATE CALLBACK (%d)", msg->data);
    mNavigationState = msg->data;
}

void pubNavigationStateHelper(NAVIGATION_STATE state) {

    // set the local state to ensure there is no delay
    mNavigationState = state;

    std_msgs::UInt8 pubmsg;
    pubmsg.data = state;
    pubNavigationState.publish(pubmsg);
    //ROS_INFO("nav state changed to %u", state);
}

void clearPath() {

    mCurrentPath->clear(); // clear the path
}

void cmdVelTimeoutCallback(const ros::TimerEvent&) {

    mCmdVelTimeout.stop();
    pubNavigationStateHelper(NAV_READY); //TODO naming is poor here
    
    Twist twist;	// empty twist, stops the robot
    pubCmdVel.publish(twist);
    
    ROS_INFO("stopped timer");
}

Pose getCurrentPose() {

    Pose currentPose;

    skynav_msgs::current_pose poseService;

    if (servClientCurrentPose.call(poseService)) {

        currentPose = poseService.response.pose;

        //        ROS_INFO("Current pose (x %f) (y %f) (theta %f)", poseService.response.pose.x, poseService.response.pose.y, poseService.response.pose.theta);
    } else {
        ROS_ERROR("Failed to call current_pose service from motion control");
        ros::shutdown();
    }

    return currentPose;
}

bool posesEqual(Pose currentPose, Pose targetPose) {

    if (sqrt(pow(targetPose.position.x - currentPose.position.x, 2) + pow(targetPose.position.y - currentPose.position.y, 2)) < DISTANCE_ERROR_ALLOWED) {
        return true;
    }

    return false;
}

double calcShortestTurn(double a1, double a2) {

    double dif = (a1 - a2); // calc shortest turn
    if (dif > M_PI)
        dif -= (M_PI * 2);
    if (dif < -M_PI)
        dif += (M_PI * 2);

    return dif;
}

void publishCmdVel(Twist twist, double t)	{
	
	//publish to cmd_vel
	pubCmdVel.publish(twist);

	// this timer is to timeout the target location
	mCmdVelTimeout = mNode->createTimer(ros::Duration(t), cmdVelTimeoutCallback); //TODO timer value
}


void motionTurn(const double theta) {	
	
	ros::Rate minimumSleep(1000); //1ms
	Pose currentPose = getCurrentPose();
	
	Twist twist;
	
	if(fmod(currentPose.orientation.z - theta + (M_PI*2), M_PI*2) <= M_PI)	{
		twist.angular.z = -TURN_VELOCITY;
	} else {
		twist.angular.z = TURN_VELOCITY;
	}
	
	pubCmdVel.publish(twist);

	
	while(currentPose.orientation.z > theta + ANGLE_ERROR_ALLOWED || currentPose.orientation.z < theta - ANGLE_ERROR_ALLOWED)	{
		currentPose = getCurrentPose();

		minimumSleep.sleep();
	}
	
	Twist twist_stop; // stop moving
	pubCmdVel.publish(twist_stop);

}


bool motionForward(const Point target) {

    if (target.x > DISTANCE_ERROR_ALLOWED) {
		
		 const Pose originalPose = getCurrentPose();
		
		double distanceLeft = target.x;
		double rateHz = 0.2;
		ros::Rate rate(1 / rateHz);
		
		for(double s = MOTION_VELOCITY / 16; s <= MOTION_VELOCITY; s *= 2)	{	// total distance travelled during this phase should be 0.0775 meter
			Twist twist;
			twist.linear.x = s;
			
			pubCmdVel.publish(twist);
			
			rate.sleep();
		}
		
		ros::Rate minimumSleep(1000); //1ms
		Pose currentPose = getCurrentPose();
		//double t;
		while((target.x - sqrt(pow(currentPose.position.x - originalPose.position.x, 2) + pow(currentPose.position.y - originalPose.position.y, 2))) > 0.0775)	{	// TODO the 0.0775 should be a var and should be checked
			currentPose = getCurrentPose();
			minimumSleep.sleep();
		}
		rate.sleep();	// HACK: otherwise it wont reach target, i guess inertia etc
		
		for(double s = MOTION_VELOCITY; s >= MOTION_VELOCITY / 16; s /= 2)	{
			
			Twist twist;
			twist.linear.x = s;
			
			pubCmdVel.publish(twist);
			
			rate.sleep();
		}
		
		//ROS_INFO("pose %f, %f", getCurrentPose().position.x, getCurrentPose().position.y);
		
		Twist twist; // stop moving
		pubCmdVel.publish(twist);
		
		pubNavigationStateHelper(NAV_POSE_REACHED);

        return true;
    } else
        return false;
}


void navigate() {

    switch (mNavigationState) {

        case NAV_UNREACHABLE:
        {
            ROS_INFO("NAV_UNREACHABLE");
            //            ROS_INFO("Could not reach target pose (%f, %f, %f)", mTargetPose.x, mTargetPose.y, mTargetPose.theta);
            //            clearPath();
            //            break;    //TEMP disabled because current_pose is NYI, target based on timeout now
        }
        case NAV_POSE_REACHED:
        {
            ROS_INFO("NAV_POSE_REACHED");

            if (mCurrentPath->size() == 1) { // just finished the last waypoint, about to remove it, but take orientation first

                ROS_INFO("waypoint list empty, assuming end orientation");

                motionTurn(mEndOrientation);
                
                mCurrentPath->pop_front();
                
                pubNavigationStateHelper(NAV_READY);

            } else {

                // check if pose was actually reached
                if (ROBOT_WAYPOINT_ACCURACY) {
                    if (posesEqual(getCurrentPose(), mCurrentPath->front().pose)) { // if within error value of target

                        mCurrentPath->pop_front();
                    }
                } else {
                    mCurrentPath->pop_front();
                }

                pubNavigationStateHelper(NAV_READY); // go into movement case again
                break;
            }

        }
        case NAV_READY: //since POSE_REACHED does not break, case READY itself is only used when the robot has no path yet
        {

            if (mCurrentPath->size() == 0)
                return;

            ROS_INFO("NAV_READY");

            PoseStamped absoluteTargetPose = mCurrentPath->front(); // always use index 0, if target reached, delete 0 and use the new 0
            absoluteTargetPose.header.frame_id = "/map";
            absoluteTargetPose.header.stamp = ros::Time::now();

            pubTargetPoseStamped.publish(absoluteTargetPose);

            Pose currentPose = getCurrentPose();
            Pose relativeTargetPose;

            if (posesEqual(currentPose, absoluteTargetPose.pose)) {
                ROS_INFO("already at targetPose, skipping");
                pubNavigationStateHelper(NAV_POSE_REACHED);
                return; // need to go to the new case before going back into this case
            }

            ROS_INFO("current target pose: (%f, %f)", absoluteTargetPose.pose.position.x, absoluteTargetPose.pose.position.y);

            double a1 = atan2(absoluteTargetPose.pose.position.y - currentPose.position.y, absoluteTargetPose.pose.position.x - currentPose.position.x); // calc turn towards next point
            double a2 = currentPose.orientation.z;

            //relativeTargetPose.orientation.z = calcShortestTurn(a1, a2); // determine shortest turn (CW or CCW)
			
			// turn to absolute theta
            motionTurn(a1);// { //if false, angle falls within the allowed error value, so now we move forward

			//ROS_INFO("skipping turn of %f degrees", relativeTargetPose.orientation.z * 180 / M_PI);

			//relativeTargetPose.orientation.z = 0; // theta no longer used

			relativeTargetPose.position.x = sqrt(pow(absoluteTargetPose.pose.position.x - currentPose.position.x, 2) + pow(absoluteTargetPose.pose.position.y - currentPose.position.y, 2));



			if (!motionForward(relativeTargetPose.position)) { // if false, distance falls within the allowed error value, so we have reached the pose

				ROS_INFO("skipping distance of %fm", relativeTargetPose.position.x);
				pubNavigationStateHelper(NAV_POSE_REACHED); // turn was within error value, so was distance. so pose is reached within error values
				return; // skip the timer and nav state
			}
            
            
        }
            break;
        case NAV_AVOIDING: //NYI
            break;
        case NAV_OBSTACLE_DETECTED: //NYI
            break;
        case NAV_CHECKING_OBSTACLE: //NYI
            break;
        case NAV_MOVING: //deprecated?
        {
            // do nothing
        }
            break;
        case NAV_ERROR:
        {

            ROS_INFO("An error occurred while traversing the path, halting");
            clearPath();
        }
            break;
        default:
            ROS_ERROR("unrecognized navigation state found");
    }
}

void placeholder_ManualTargetPose() {

    ROS_INFO("please input target coordinates: x");
    string userInput;
    cin >> userInput;

    //        vector<string> stringInput;
    //        boost::split(stringInput, userInput, boost::is_any_of(","));

    Pose pose;
    pose.position.x = atof(userInput.c_str());
	motionForward(pose.position);
    

    //        ROS_INFO("please input target theta (degrees)");
    //        cin >> userInput;
    //targetPose.pose.theta = 0;//M_PI / 180 * atof(userInput.c_str());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "motion_control");

	ros::NodeHandle n = ros::NodeHandle("");
    mNode = new ros::NodeHandle("/control");
    mNodeSLAM = new ros::NodeHandle("/slam");


    //pubs
    pubCmdVel = n.advertise<Twist>("/cmd_vel", 2);
    pubNavigationState = mNode->advertise<std_msgs::UInt8>("navigation_state", 0);
    pubTargetPoseStamped = mNode->advertise<geometry_msgs::PoseStamped>("target_pose", 4);

    //subs
    ros::Subscriber subCheckedWaypoints = mNode->subscribe("checked_waypoints", 32, subCheckedWaypointsCallback);
    ros::Subscriber subNavigationState = mNode->subscribe("navigation_state", 0, subNavigationStateCallback);

    //services
    servClientCurrentPose = mNodeSLAM->serviceClient<skynav_msgs::current_pose>("current_pose");

    // hz
    ros::Rate loop_rate(10); // loop every 100ms

    loop_rate.sleep(); // sleep a little to prevent double startup bug (calls loop twice in 1 spin)
    loop_rate.sleep();
    loop_rate.sleep();

    //placeholder_ManualTargetPose();

    while (ros::ok()) {

        ros::spinOnce(); // read topics

		navigate();

        loop_rate.sleep(); // sleep
    }

    delete mNode;
    delete mNodeSLAM;

    return 0;
}

