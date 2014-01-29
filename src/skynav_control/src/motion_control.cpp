#include <ros/ros.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/TimedPose.h>
#include <string.h>
#include <list>
#include <boost/algorithm/string.hpp>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// defines
#define SPEED_MULTIPLIER                10                      // 1 / multiplier = speed
#define ANGLE_ERROR_ALLOWED             M_PI / 180 * 5          // radss
#define DISTANCE_ERROR_ALLOWED          0.1                    // in meters

#define ROBOT_WAYPOINT_ACCURACY         true                   // if true, robot will continue trying to reach target goal within error values until proceeding to next target

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
ros::Publisher pubTargetMotion, pubNavigationState, pubTargetPoseStamped;

list<PoseStamped>* mCurrentPath = new list<PoseStamped>;
double mEndOrientation = 0;

ros::Timer mTargetPoseTimeout;
ros::Timer mTargetOrientationTimeout;
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
}

void clearPath() {

    mCurrentPath->clear(); // clear the path
}

void targetPoseTimeoutCallback(const ros::TimerEvent&) {

    mTargetPoseTimeout.stop();
    pubNavigationStateHelper(NAV_POSE_REACHED);
}

void targetOrientationTimeoutCallback(const ros::TimerEvent&) {

    mTargetOrientationTimeout.stop();
    pubNavigationStateHelper(NAV_READY); //TODO naming is poor here
}

Pose getCurrentPose() {

    Pose currentPose;

    skynav_msgs::current_pose poseService;

    if (servClientCurrentPose.call(poseService)) {

        currentPose = poseService.response.pose;

        //        ROS_INFO("Current pose (x %f) (y %f) (theta %f)", poseService.response.pose.x, poseService.response.pose.y, poseService.response.pose.theta);
    } else {
        ROS_ERROR("Failed to call current_pose service from motion control");
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

bool motionTurn(geometry_msgs::Quaternion q) {

    if (q.z > ANGLE_ERROR_ALLOWED || q.z < -ANGLE_ERROR_ALLOWED) {// this allows a theta error

        skynav_msgs::TimedPose relativeTargetPose;

        relativeTargetPose.pose.orientation = q;
        relativeTargetPose.actuationSeconds = fmax(abs(q.z) * 3, 2); // time to turn // TODO time value variable

        pubTargetMotion.publish(relativeTargetPose);

        // this timer is to timeout orientation towards target location
        mTargetOrientationTimeout = mNode->createTimer(ros::Duration(relativeTargetPose.actuationSeconds + 2), targetOrientationTimeoutCallback); //TODO timer value

        return true;
    } else
        return false;
}

bool motionForward(geometry_msgs::Point p) {

    if (p.x > DISTANCE_ERROR_ALLOWED) {

        skynav_msgs::TimedPose relativeTargetPose;

        relativeTargetPose.pose.position = p;
        relativeTargetPose.actuationSeconds = fmax(abs(relativeTargetPose.pose.position.x) * SPEED_MULTIPLIER, 2); // time to move straight ahead //TODO time

        pubTargetMotion.publish(relativeTargetPose);

        // this timer is to timeout the target location
        mTargetPoseTimeout = mNode->createTimer(ros::Duration(relativeTargetPose.actuationSeconds + 2), targetPoseTimeoutCallback); //TODO timer value

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

                geometry_msgs::Quaternion endOrientation;
                endOrientation.z = calcShortestTurn(mEndOrientation, getCurrentPose().orientation.z);

                motionTurn(endOrientation);
                
                mCurrentPath->pop_front();

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
            skynav_msgs::TimedPose relativeTargetPose;

            if (posesEqual(currentPose, absoluteTargetPose.pose)) {
                ROS_INFO("already at targetPose, skipping");
                pubNavigationStateHelper(NAV_POSE_REACHED);
                return; // need to go to the new case before going back into this case
            }

            ROS_INFO("current target pose: (%f, %f)", absoluteTargetPose.pose.position.x, absoluteTargetPose.pose.position.y);

            double a1 = atan2(absoluteTargetPose.pose.position.y - currentPose.position.y, absoluteTargetPose.pose.position.x - currentPose.position.x); // calc turn towards next point
            double a2 = currentPose.orientation.z;

            relativeTargetPose.pose.orientation.z = calcShortestTurn(a1, a2); // determine shortest turn (CW or CCW)




            if (!motionTurn(relativeTargetPose.pose.orientation)) { //if false, angle falls within the allowed error value, so now we move forward

                ROS_INFO("skipping turn of %f degrees", relativeTargetPose.pose.orientation.z * 180 / M_PI);

                relativeTargetPose.pose.orientation.z = 0; // theta no longer used

                relativeTargetPose.pose.position.x = sqrt(pow(absoluteTargetPose.pose.position.x - currentPose.position.x, 2) + pow(absoluteTargetPose.pose.position.y - currentPose.position.y, 2));



                if (!motionForward(relativeTargetPose.pose.position)) { // if false, distance falls within the allowed error value, so we have reached the pose

                    ROS_INFO("skipping distance of %fm", relativeTargetPose.pose.position.x);
                    pubNavigationStateHelper(NAV_POSE_REACHED); // turn was within error value, so was distance. so pose is reached within error values
                    return; // skip the timer and nav state
                }
            }

            //set state to moving
            pubNavigationStateHelper(NAV_MOVING);
        }
            break;
        case NAV_AVOIDING:
            break;
        case NAV_OBSTACLE_DETECTED:
            break;
        case NAV_CHECKING_OBSTACLE:
            break;
        case NAV_MOVING:
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

    skynav_msgs::TimedPose targetPose;
    targetPose.pose.position.x = atof(userInput.c_str());
    //        targetPose.pose.theta = atof(userInput.c_str());
    targetPose.actuationSeconds = targetPose.pose.position.x * 10;

    //        ROS_INFO("please input target theta (degrees)");
    //        cin >> userInput;
    //targetPose.pose.theta = 0;//M_PI / 180 * atof(userInput.c_str());

    pubTargetMotion.publish(targetPose);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "motion_control");

    mNode = new ros::NodeHandle("/control");
    mNodeSLAM = new ros::NodeHandle("/SLAM");


    //pubs
    pubTargetMotion = mNode->advertise<skynav_msgs::TimedPose>("target_motion", 32);
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

    //    placeholder_ManualTargetPose();

    while (ros::ok()) {

        ros::spinOnce(); // read topics

        navigate();

        loop_rate.sleep(); // sleep
    }

    delete mNode;
    delete mNodeSLAM;

    return 0;
}

