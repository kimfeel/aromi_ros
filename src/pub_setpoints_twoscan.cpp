/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "math.h"

double r;
double theta;
double count = 0;
double wn = 0;
bool offboard = 0;
bool rotate = 0;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped    localPose;
	
bool isTargetPos(geometry_msgs::PoseStamped TargetPose)
{
     if ( abs(TargetPose.pose.position.x - localPose.pose.position.x) < 0.1 &&
          abs(TargetPose.pose.position.y - localPose.pose.position.y) < 0.1 &&
          abs(TargetPose.pose.position.z - localPose.pose.position.z) < 0.1)
     {
        return true;
     }
    return false;   
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void LocalPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    localPose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber location_sub = nh.subscribe<geometry_msgs::PoseStamped>
    	    ("mavros/local_position/pose", 10, LocalPosition_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
	
//    nh.param("pub_setpoints_traj/wn", wn, 1.0);
    nh.param("pub_setpoints_traj/r", r, 1.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected!!");

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.8;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
	if( current_state.mode == "OFFBOARD" && offboard == 0 ) {
		last_request = ros::Time::now();
		offboard = 1;
		printf("Offboard!!\n");
	}
	if( offboard == 1 &&
            (ros::Time::now() - last_request > ros::Duration(10.0))) {
            printf("%f!!\n",theta);
	    rotate = 1;		
            last_request = ros::Time::now();
        }
	if (theta < 3.14) {
		theta = count*0.002; // one rotation 79 seconds: 0.114 deg/count
	    	pose.pose.position.x = 0;
	    	pose.pose.position.y = 0;
	    	pose.pose.position.z = 0.8;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = -sin(theta);
		pose.pose.orientation.w = -cos(theta);
		if (rotate == 1) {		
			count++;
		}
	}
	else {
		printf("Move to 2nd point!\n");
		pose.pose.position.x = 0;
	    	pose.pose.position.y = 0.3;
	    	pose.pose.position.z = 0.8;

		//once isTargetPos is true exit the loop
		if( isTargetPos == true){
		break;
		}
	}
    }
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
	if( offboard == 1 &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            printf("%f!!\n",theta);
	    rotate = 1;		
            last_request = ros::Time::now();
	    theta =0;
        }
	if (theta < 3.14) {
		theta = count*0.002; // one rotation 79 seconds: 0.114 deg/count
	    	pose.pose.position.x = 0;
	    	pose.pose.position.y = 0.3;
	    	pose.pose.position.z = 0.8;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = -sin(theta);
		pose.pose.orientation.w = -cos(theta);
		if (rotate == 1) {		
			count++;
		}
	}
	else {
		printf("Landing!!\n");
		pose.pose.position.x = 0;
	    	pose.pose.position.y = 0;
	    	pose.pose.position.z = 0;
//		if (isTargetPos(pose)) {
//			if( arming_client.call(disarm_cmd) &&
//                    		disarm_cmd.response.success){
//                    		ROS_INFO("disarmed!!");
//                	}
//		}
	}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

