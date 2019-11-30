/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

double r;
double theta;
int count = 0;
double wn;

mavros_msgs::State 			  current_state;
geometry_msgs::PoseStamped    targetPose;
geometry_msgs::PoseStamped    localPose;
	
bool isTargetPos()
{
     if ( abs(targetPose.pose.position.x - localPose.pose.position.x) < 0.1 &&
          abs(targetPose.pose.position.y - localPose.pose.position.y) < 0.1 &&
          abs(targetPose.pose.position.z - localPose.pose.position.z) < 0.1)
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

    geometry_msgs::PoseStamped TargetPose[2];

    TargetPose[0].pose.position.x = 0;
    TargetPose[0].pose.position.y = 0;
    TargetPose[0].pose.position.z = 1;

    TargetPose[1].pose.position.x = 0;
    TargetPose[1].pose.position.y = 0;
    TargetPose[1].pose.position.z = 1;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(TargetPose[0]);
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

	  if (isTargetPos())
      {
          count++;
      }
      targetPose.pose.position.x = TargetPose[count].pose.position.x;
      targetPose.pose.position.y = TargetPose[count].pose.position.y;
	  targetPose.pose.position.z = TargetPose[count].pose.position.z;
      targetPose.pose.orientation.x = 0;
	  targetPose.pose.orientation.y = 0;
	  targetPose.pose.orientation.z = 0;
	  targetPose.pose.orientation.w = 1;
			
      if (count >= 1)
	  {
		  count = 0;	
	  }
      local_pos_pub.publish(targetPose);
	  ros::spinOnce();
	  rate.sleep();
    }

    return 0;
}

