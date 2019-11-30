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
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

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

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

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
	if (theta < 12.56) {
		theta = count*0.005;
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
		printf("Landing!!\n");
		pose.pose.position.x = 0;
	    	pose.pose.position.y = 0;
	    	pose.pose.position.z = 0;
	}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

