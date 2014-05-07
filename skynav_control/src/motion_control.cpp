#include <ros/ros.h>
#include <skynav_msgs/current_pose.h>
#include <skynav_msgs/current_velocity.h>
#include <skynav_msgs/waypoint_check.h>
#include <string.h>
#include <list>
#include <boost/algorithm/string.hpp>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// defines
#define MOTION_VELOCITY                	0.2                      // speed in m/s
#define MAX_VELOCITY					1.0						
#define TURN_VELOCITY					0.3						// turn speed in rad/s
#define ANGLE_ERROR_FIRST		  		M_PI / 180 * 20 		// rads // first stage angle check
#define ANGLE_ERROR_ALLOWED             M_PI / 180 * 1          // rads	// second stage angle check for precision // if too small, the robot may rotate infinitely
#define DISTANCE_ERROR_ALLOWED          0.1                    // in meters

#define ROBOT_WAYPOINT_ACCURACY         false                   // if true, robot will continue trying to reach target goal within error values until proceeding to next target

enum NAVIGATION_STATE { 			//TODO move this into a shared container (skynav_msgs perhaps?)
    NAV_READY = 0,					//check for new path recieved
    NAV_MOVING = 1,					//calculate path to next waypoint, turn, accel, drive and decel.
    NAV_AVOIDING = 2,				//reroute past detected object
    NAV_POSE_REACHED = 3,			//(intermediate) waypoint has been reached, check
    NAV_ERROR = 4,					
    NAV_UNREACHABLE = 5,			
    NAV_OBSTACLE_DETECTED = 6,		//obstacle has been detected
    NAV_CHECKING_OBSTACLE = 7,		//Depricated?!
    NAV_STOP = 8					//stop everything before continue with whatever
};

enum MOVEMENT_STATE{
	MOV_READY = 0,					//calculate path to next waypoint
	MOV_TURN = 1,					//turn appropriate angle
	MOV_ACCEL = 2,					//accelerate to certain velocity
	MOV_STEADY = 3,					//drive steady at certain velocity
	MOV_DECEL = 4,					//decelerate until full stop
	MOV_REACHED = 5					//done with moving
};

using namespace geometry_msgs;
using namespace std;

// VARS
ros::NodeHandle* mNode;
ros::NodeHandle* mNodeSLAM;

ros::ServiceClient servClientCurrentPose, servClientWaypointCheck, servClientCurrentVelocity;
ros::Publisher pubCmdVel, pubNavigationState, pubTargetPoseStamped;

list<PoseStamped>* mCurrentPath = new list<PoseStamped>;
list<PoseStamped>* mOriginalPath = new list<PoseStamped>;
double mEndOrientation = 0;

ros::Timer mCmdVelTimeout;
uint8_t mNavigationState = NAV_READY; 	// initial navigation_state
uint8_t mMovementState = MOV_READY;		// initial movement_state

void subCheckedWaypointsCallback(const nav_msgs::Path::ConstPtr& msg) {

    mCurrentPath = new list<PoseStamped>(msg->poses.begin(), msg->poses.end());
	mOriginalPath = new list<PoseStamped>(msg->poses.begin(), msg->poses.end());
    mEndOrientation = mOriginalPath->back().pose.orientation.z;

    ROS_INFO("path received by motion control");
}

//callback to change navigationstate on an interrupt from outside
void subNavigationStateInterruptCallback(const std_msgs::UInt8::ConstPtr& msg) {
	mNavigationState = msg->data;
	ROS_WARN("nav_state interrupted, changed to (%u)", msg->data);
    pubNavigationState.publish(msg);
	return;
}

//function to change navigationstate from within this node, and publish current state to the outside
void pubNavigationStateHelper(NAVIGATION_STATE state) {

    // set the local state to ensure there is no delay
    mNavigationState = state;

    std_msgs::UInt8 pubmsg;
    pubmsg.data = state;
    pubNavigationState.publish(pubmsg);
    //ROS_INFO("nav state function changed to %u", state);
}

void setMovementState(MOVEMENT_STATE moveState) {

    // set the local state to ensure there is no delay
    mMovementState = moveState;
    //ROS_INFO("mov_state changed to %u", moveState);
}

void clearPath() {

    mCurrentPath->clear(); // clear the path
    mOriginalPath->clear();
}

//calculate distance between two point with use of pythagoras
double calcDistance(Point a, Point b){
	return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

void publishCmdVel(Twist twist)	{
	
	if(twist.linear.x > MOTION_VELOCITY){
		ROS_ERROR("Attempt to send cmdvel %f, which is higher than maximum intended velocity. Changing to intended velocity of: %f",twist.linear.x,MOTION_VELOCITY);
		twist.linear.x = MOTION_VELOCITY;
	}	
	//publish to cmd_vel
	pubCmdVel.publish(twist);

	// this timer is to timeout the target location
	//mCmdVelTimeout = mNode->createTimer(ros::Duration(t), cmdVelTimeoutCallback); //TODO timer value
}

void cmdVelTimeoutCallback(const ros::TimerEvent&) {

    mCmdVelTimeout.stop();
    pubNavigationStateHelper(NAV_READY); 
    
    Twist twist_stop;
    pubCmdVel.publish(twist_stop);
    
    ROS_INFO("stopped timer");
}

Pose getCurrentPose() {
	try{
		Pose currentPose;

		skynav_msgs::current_pose poseService;

		if (servClientCurrentPose.call(poseService)) {

			currentPose = poseService.response.pose;

		} else {
			ROS_ERROR("Failed to call current_pose service from motion control");
			ros::shutdown();
		}
		return currentPose;
	}catch(exception& e){
		ROS_ERROR("exception caught: %s",e.what());
		ros::shutdown();
	}
}

Twist getCurrentVelocity() {
	try{
		Twist currentVelocity;

		skynav_msgs::current_velocity velService;

		if (servClientCurrentVelocity.call(velService)) {

			currentVelocity = velService.response.velocity;

		} else {
			ROS_ERROR("Failed to call current_velocity service from motion control");
			ros::shutdown();
		}
		return currentVelocity;
	}catch(exception& e){
		ROS_ERROR("exception caught: %s",e.what());
		ros::shutdown();
	}
}

bool posesEqual(Pose currentPose, Pose targetPose) {

    if (calcDistance(targetPose.position,currentPose.position)< DISTANCE_ERROR_ALLOWED) {
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

//semi depricated function as this is integrated into nav_moving state
void motionTurn(const double theta) {	
	
	ros::Rate minimumSleep(1000); //1ms
	Pose currentPose = getCurrentPose();
	
	Twist twist;
	
	if(fmod(currentPose.orientation.z - theta + (M_PI*2), M_PI*2) <= M_PI)	{
		twist.angular.z = -TURN_VELOCITY;
	} else {
		twist.angular.z = TURN_VELOCITY;
	}
	
	publishCmdVel(twist);

	
	while(currentPose.orientation.z > theta + ANGLE_ERROR_ALLOWED || currentPose.orientation.z < theta - ANGLE_ERROR_ALLOWED)	{
		currentPose = getCurrentPose();

		minimumSleep.sleep();
	}
	
	Twist twist_stop; // stop moving
	publishCmdVel(twist_stop);

}

void navigate() {

    switch (mNavigationState) {

        case NAV_UNREACHABLE:
        {
            ROS_WARN("NAV_UNREACHABLE");
			ROS_INFO("Could not reach target pose");
			clearPath();
        }
        break;
        case NAV_POSE_REACHED:
        {
			ROS_WARN("NAV_POSE_REACHED");
            if (mCurrentPath->size() == 1) { // just finished the last waypoint, about to remove it, but take orientation first
                ROS_INFO("waypoint list empty, assuming end orientation");
                
                //motionTurn(mEndOrientation); //function is blocking when mEndOrientation is not reached
               
                mCurrentPath->pop_front();
                ROS_INFO("Target reached");                

            } else {
                // check if pose was actually reached
                if (ROBOT_WAYPOINT_ACCURACY) {
					Pose currentPose = getCurrentPose();
                    if (posesEqual(currentPose, mCurrentPath->front().pose)) { // if within error value of target
                        mCurrentPath->pop_front();
                    }else{
						ROS_ERROR("target not quite reached; compensating");
						ros::Rate(5).sleep();
					}
                } else {
				    ROS_INFO("nav pose reached unchecked, continue"); 
					mCurrentPath->pop_front();					
					}
            }            
			pubNavigationStateHelper(NAV_READY);
        }
        break;
        case NAV_READY: 	//IDLE state if there is no path currently available
        {						
            if (!mCurrentPath->size() == 0){ 
				ROS_WARN("NAV_READY");
				
				PoseStamped absoluteTargetPose = mCurrentPath->front(); // always use index 0, if target reached, delete 0 and use the new 0
				
				absoluteTargetPose.header.frame_id = "/map";
				absoluteTargetPose.header.stamp = ros::Time::now();

				pubTargetPoseStamped.publish(absoluteTargetPose);          
				ROS_INFO("target pose: (%f, %f)", absoluteTargetPose.pose.position.x, absoluteTargetPose.pose.position.y);
				setMovementState(MOV_READY);
				pubNavigationStateHelper(NAV_MOVING);
			}else{
				//IDLE
				 return; 
			}
        }
		break;
		case NAV_MOVING:
        {			
			ROS_WARN("NAV_MOVING");
			ros::Rate minimumSleep(1000); 	//1ms
			double rateHz = 0.2;
			ros::Rate rate(1 / rateHz);
			
			Pose currentPose;			
			Pose originalPose;				
			PoseStamped absoluteTargetPose;
			Pose relativeTargetPose;
			Twist currentVelocity;
			
			double theta;						//relative angle to target
			float breakDist;					//distance for robot to decel 
			double steadyVelocity = MOTION_VELOCITY;
			
			Twist twist_stop;
			
			bool in_motion = false;		//"is currently busy"  boolean
			setMovementState(MOV_READY);
			
			while(mNavigationState == NAV_MOVING){ 		//while(!interrupted){
				currentPose = getCurrentPose();
				currentVelocity = getCurrentVelocity();
				
				switch(mMovementState){
					case MOV_READY:
					{
						//retrieve abs_target pose
						absoluteTargetPose = mCurrentPath->front(); // always use index 0, if target reached, delete 0 and use the new 0
						//retrieve current pose
						originalPose = currentPose;  
						
						if (posesEqual(currentPose, absoluteTargetPose.pose)) {
							ROS_INFO("already at target pose, skipping");
							pubNavigationStateHelper(NAV_POSE_REACHED);
							break; // need to go to the new case before going back into this case
						}

						theta =  atan2(absoluteTargetPose.pose.position.y - currentPose.position.y, absoluteTargetPose.pose.position.x - currentPose.position.x); // calc turn towards next point
			
						setMovementState(MOV_TURN);			
					}
					break;
					case MOV_TURN:
					{
						if(!in_motion){
							ROS_INFO("mov_turn");
							Twist twist;
							if(fmod(currentPose.orientation.z - theta + (M_PI*2), M_PI*2) <= M_PI)	{
								twist.angular.z = -TURN_VELOCITY;
							} else {
								twist.angular.z = TURN_VELOCITY;
							}
							publishCmdVel(twist);
							in_motion = true;
						}						

						//turn rough angle
						if(!(currentPose.orientation.z < theta + ANGLE_ERROR_FIRST && currentPose.orientation.z > theta - ANGLE_ERROR_FIRST))	{
							//do nothing, continue turning
						}else{	
							//ROS_INFO("1st stage rough turning done");					
							publishCmdVel(twist_stop); //stop movement
							rate.sleep();

							currentPose = getCurrentPose();
							Twist twist;
							
							//if the angle already falls within accepted error margin, continue
							if(currentPose.orientation.z > theta + ANGLE_ERROR_ALLOWED || currentPose.orientation.z < theta - ANGLE_ERROR_ALLOWED)	{
								
								if(fmod(currentPose.orientation.z - theta + (M_PI*2), M_PI*2) <= M_PI)	{
									twist.angular.z = -0.5*TURN_VELOCITY;
								} else {
									twist.angular.z =  0.5*TURN_VELOCITY;
								}
								publishCmdVel(twist);
								
								while(currentPose.orientation.z > theta + ANGLE_ERROR_ALLOWED || currentPose.orientation.z < theta - ANGLE_ERROR_ALLOWED)	{
									//do nothing, continue turning
									currentPose = getCurrentPose();
									minimumSleep.sleep();
								}
							}
							publishCmdVel(twist_stop); //stop movement

							//ROS_INFO("2nd stage precision turning done");
							
							in_motion = false;
							
							rate.sleep();
							setMovementState(MOV_ACCEL);
						}
					}
					break;
					case MOV_ACCEL:
					{
						ROS_INFO("mov_accel");

						//relative target pose is a point on the relative x-axle of the robot.
						relativeTargetPose.position.x = calcDistance(absoluteTargetPose.pose.position, currentPose.position);
						relativeTargetPose.position.y = 0;				
						
						if (relativeTargetPose.position.x <= DISTANCE_ERROR_ALLOWED){
							ROS_WARN("target lies within treshold distance, skipping %fm",relativeTargetPose.position.x);
							setMovementState(MOV_REACHED);
							break;
						}
						in_motion = true;							

						Twist twist;
											
						for(double s = MOTION_VELOCITY / 16; s <= MOTION_VELOCITY; s *= 2)	{	
							
							twist.linear.x = s;
							publishCmdVel(twist);
							
							rate.sleep();
						}
						in_motion = false;
						setMovementState(MOV_STEADY);
					}
					break;
					case MOV_STEADY:
					{
						if(!in_motion){
							ROS_INFO("mov_steady");
							in_motion = true;
						}
													
						//calculate breaking distance
						breakDist = 0;						
						for(int i=0; i<=4;++i){
								breakDist+= (abs(currentVelocity.linear.x / (pow(2,i)))*0.2);
						}
							
						double dist_traversed = relativeTargetPose.position.x - (calcDistance(currentPose.position,originalPose.position));
						if(dist_traversed > breakDist){
							//do nothing, continue moving	
							//ROS_INFO("curVelocity %f",currentVelocity.linear.x);

						}else{
							steadyVelocity = currentVelocity.linear.x;
							setMovementState(MOV_DECEL);
							in_motion = false;
						}	
					}
					break;
					case MOV_DECEL:
					{
						ROS_INFO("mov_decel");
						Twist twist;
						for(double s = steadyVelocity; s >= steadyVelocity / 16; s /= 2)	{
			
							twist.linear.x = s;							
							publishCmdVel(twist);
							
							rate.sleep();
						}						
						publishCmdVel(twist_stop);
						in_motion = false;
						setMovementState(MOV_REACHED);	
					}
					break;
					case MOV_REACHED:
					{
						ROS_INFO("mov_reached");
						pubNavigationStateHelper(NAV_POSE_REACHED); // turn was within error value, so was distance. so pose is reached within error values
					}
					break;		
					default:
					{
						ROS_ERROR("unrecognized movement state found");
						
					break;
					}		
				}
				ros::spinOnce();	//check for callbacks
				minimumSleep.sleep();
			}		
		}
		break;
		case NAV_OBSTACLE_DETECTED:
		{	
			ROS_WARN("NAV_OBSTACLE_DETECTED");	
			Twist twist_stop;	
			float curVel = getCurrentVelocity().linear.x;
			if(abs(curVel) > 0.05){
				ROS_INFO("current velocity is: %f ,breaking", curVel);
				double rateHz = 0.2;
				ros::Rate rate(1 / rateHz);
				Twist twist;
				for(double s = curVel; s >= curVel / 16; s /= 2)	{
	
					twist.linear.x = s;							
					publishCmdVel(twist);
					
					rate.sleep();
				}
			}
			publishCmdVel(twist_stop);
			pubNavigationStateHelper(NAV_AVOIDING);
        }
		break;
        case NAV_AVOIDING:
		{	
			ROS_WARN("NAV_AVOIDING");
			ros::Duration(1).sleep();	//sleep 1 second for the sensors to calibrate		
			Pose currentPose = getCurrentPose();
			Pose absTarget = mCurrentPath->front().pose;

			skynav_msgs::waypoint_check srv;
			srv.request.currentPos = currentPose.position;
			srv.request.targetPos  = absTarget.position;
			
			//re-check obstruction of the path and calculate the detour. 
			//TODO create a check if new path is inefficient. in that case, dont push_front
			if(servClientWaypointCheck.call(srv)){
				if(srv.response.pathChanged){
					PoseStamped nwWaypoint;
					nwWaypoint.pose.position = srv.response.newPos;
					mCurrentPath->push_front(nwWaypoint);	// be careful here!	
					
					//todo check if path from nwWaypoint to next doesnt collide.. if true, calculate new nwWaypoint and push after current nwWaypoint

				}else{
					ROS_INFO("False alarm");
				}
			}else{
				ROS_ERROR("failed to call waypoint check service from motion_control NAV_AVOIDING state ");
			}
			pubNavigationStateHelper(NAV_READY);
        }   
		break;      
        case NAV_CHECKING_OBSTACLE: //DEPRECATED
        {	
			ROS_WARN("NAV_OBSTACLE_DETECTED");
		}
		break;
        case NAV_ERROR:
        {
			ROS_WARN("NAV_ERROR");
            clearPath();
            
        }
        break;
        case NAV_STOP:
        {
			ROS_WARN("NAV_STOP");
			double currentVelocity = getCurrentVelocity().linear.x;
			Twist twist_stop;
			Twist twist;
			   ros::Rate rate(5); // each 200ms

			for(double s = currentVelocity; s >= currentVelocity / 16; s /= 2)	{

				twist.linear.x = s;							
				publishCmdVel(twist);
				
				rate.sleep();
			}						
			publishCmdVel(twist_stop);
			
			clearPath();
			
			pubNavigationStateHelper(NAV_READY);

            
        }
        break;
        default:
            ROS_ERROR("unrecognized navigation state found");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "motion_control");

	ros::NodeHandle n = ros::NodeHandle("");
    ros::NodeHandle n_localnav("/localnav");
    
    mNode = new ros::NodeHandle("/control");
    mNodeSLAM = new ros::NodeHandle("/slam");


    //pubs
    pubCmdVel = n.advertise<Twist>("/cmd_vel", 2);
    pubNavigationState = mNode->advertise<std_msgs::UInt8>("navigation_state", 0);
    pubTargetPoseStamped = mNode->advertise<geometry_msgs::PoseStamped>("target_pose", 4);

    //subs
    ros::Subscriber subCheckedWaypoints = mNode->subscribe("checked_waypoints", 1, subCheckedWaypointsCallback);
    ros::Subscriber subNavigationState = mNode->subscribe("navigation_state_interrupt", 0, subNavigationStateInterruptCallback);

    //services
    servClientCurrentPose = mNodeSLAM->serviceClient<skynav_msgs::current_pose>("current_pose");
    servClientCurrentVelocity = mNodeSLAM->serviceClient<skynav_msgs::current_velocity>("current_velocity");
    
    servClientWaypointCheck = n_localnav.serviceClient<skynav_msgs::waypoint_check>("path_check");

    // hz
    ros::Rate loop_rate(10); // loop every 100ms

	ros::Duration(1).sleep();

    while (ros::ok()) {

        ros::spinOnce(); // read topics

		navigate();

        loop_rate.sleep(); // sleep
    }

    delete mNode;
    delete mNodeSLAM;

    return 0;
}

