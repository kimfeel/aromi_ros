
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
double count=0.0;
double wn;

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
//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//            ("mavros/cmd/arming");
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
	
    nh.param("pub_setpoints_traj/wn", wn, 1.0);
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
    pose.pose.position.z = 0.6;

    //send a few setpoints before starting
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

		theta = wn*count*0.05;

	   	pose.pose.position.x = 0;//r*sin(theta);
	   	pose.pose.position.y = 0;//r*cos(theta);
	   	pose.pose.position.z = 0.6;//2;
//		pose.pose.orientation.x = 0;
//		pose.pose.orientation.y = 0;
//		pose.pose.orientation.z = 0;//sin(theta);
//		pose.pose.orientation.w = 1;//cos(theta);
		count++;

		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
   }

    return 0;
}


/*#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;

   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Rate loop_rate(100);
   ros::spinOnce();

   geometry_msgs::PoseStamped msg;
   int count = 1;
	
		//PositionReciever qp;:
		//Body some_object;
		//qp.connect_to_server();

	
   while(ros::ok()){
	   //some_object = qp.getStatus();
		// some_object.print();
		//printf("%f\n",some_object.position_x);
       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = 1;
       msg.pose.position.x = 0.0;//0.001*some_object.position_x;
       msg.pose.position.y = 0.0;//0.001*some_object.position_y;
       msg.pose.position.z = 0.6;//0.001*some_object.position_z;
       msg.pose.orientation.x = 0.0;
       msg.pose.orientation.y = 0.0;
       msg.pose.orientation.z = 0.0;
       msg.pose.orientation.w = 1.0;

       chatter_pub.publish(msg);
       ros::spinOnce();
       count++;
       loop_rate.sleep();
   }
   
      
   return 0;
}
*/
