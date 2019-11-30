#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sys/time.h>
#include "hpcd.h"
#include "lapack.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

//int countLidar = 0;
//int countPose = 0;
double leaf_size = 0.001;
ros::Publisher *pubCloud;
sensor_msgs::PointCloud2 cloudMsg;
std::vector<Point> points;
//std::vector<Point> inc_points;
HPCD* hpcd = NULL;
tf::Vector3 translation = tf::Vector3(0,0,0);
//tf::Quaternion rotation = tf::Quaternion(0,0,0,1);
tf::Quaternion rotation = tf::Quaternion(0,1,0,0);
tf::Quaternion rot_correction;
tf::Quaternion translationOffset(0,0,-0.2,0);
//tf::Quaternion rotationOffset(0,0,1,0);
double previousTime=-1;
unsigned char* image_data = NULL;
int image_height,image_width;
tf::Quaternion image_translation;
tf::Quaternion image_rotation;
tf::Quaternion Twc(0,-0.1,-0.15,0);
//tf::Quaternion Rwc(sqrt(2)/2,0,0,sqrt(2)/2);
tf::Quaternion Rwc(0.6892349, 0.1579723, -0.2257698, 0.6700955 );
float focal_length = 971;

double offset_x = 0;//0.25*cos(50*M_PI/180);
double offset_y = 0;//-0.25*sin(50*M_PI/180);;
double rad_per_step = 5.625 / 64 * M_PI / 180;

double curr_x = 0;
double curr_y = 0;
double curr_z = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

void savePCD(const char* filename, std::vector<Point> *P) {
	FILE* f = fopen(filename, "w");
	if (!f) {
		printf("Cannot write to file: %s\n", filename);
		return;
	}
	fprintf(f, "# .PCD v0.7 - Point Cloud Data file format\n"
		"VERSION 0.7\n"
		"FIELDS x y z rgb\n"
		"SIZE 4 4 4 4\n"
		"TYPE F F F I\n"
		"COUNT 1 1 1 1\n"
		"WIDTH %lu\n"
		"HEIGHT 1\n"
		"VIEWPOINT 0 0 0 1 0 0 0\n"
		"POINTS %lu\n"
		"DATA ascii\n", P->size(), P->size());
	for (size_t i = 0; i<P->size(); i++) {
		Point* h = P->data() + i;
		if (h) {
			int rgb = (h->r << 16) | (h->g << 8) | h->b;
			fprintf(f, "%f %f %f %d\n", h->x, h->y, h->z, rgb);
		}
	}
	fclose(f);
	printf("Wrote %lu points to %s\n", P->size(), filename);
}

double get_walltime(void) {
	struct timeval tp;
	gettimeofday(&tp, NULL);
	double d = (double) (tp.tv_sec + tp.tv_usec/1.e6);
	return d;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
	curr_x = pose->pose.position.x;
	curr_y = pose->pose.position.y;
	curr_z = pose->pose.position.z;
	tf::Quaternion q(
		pose->pose.orientation.x,
		pose->pose.orientation.y,
		pose->pose.orientation.z,
		pose->pose.orientation.w*-1);
	tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
//	printf("%f, %f, %f, %f, %f, %f\n", curr_x, curr_y, curr_z, roll, pitch, yaw);
}

void publish_object(ros::Publisher *pub, std::vector<Point> *P) {
	cloudMsg.header.stamp = ros::Time::now();
	cloudMsg.width = P->size();
	cloudMsg.row_step = P->size() * cloudMsg.point_step;
	cloudMsg.data.clear();
	cloudMsg.data.resize(cloudMsg.row_step);
	float* float_data = (float*) cloudMsg.data.data();
	unsigned char* byte_data = cloudMsg.data.data();
	for (size_t i=0;i<P->size();i++) {
		float_data[0] = P->at(i).x;
		float_data[1] = P->at(i).y;
		float_data[2] = P->at(i).z;
		byte_data[12] = 255;//P->at(i).r;
		byte_data[13] = 255;//P->at(i).g;
		byte_data[14] = 255;//P->at(i).b;
		float_data +=  cloudMsg.point_step / sizeof(float);
		byte_data += cloudMsg.point_step;
	}
	pub->publish(cloudMsg);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidarMsg) {
	double angle = lidarMsg->angle_min;
//	inc_points.clear();
//	double startTime = get_walltime();
//	std::vector<Point> currentScan;
//	int countColor = 0;

	// angle_increment: 0.0043542264 (0.25deg), length:1440, 
	for (size_t i=0;i<lidarMsg->ranges.size();i++) {	
		angle = lidarMsg->angle_min + i*lidarMsg->angle_increment;
		double dist = lidarMsg->ranges[i];
		Point p = {0,0,0,0,0,0};
//		std::string tmp = lidarMsg->header.frame_id;
		if (angle < 0) 
			continue;
//		if (i < 720) {
//			printf("%f\n, ",angle*180/M_PI);
//			continue;
//		}
//		if (dist < 0.2) {
//			printf("%d, ",i);
//			continue; 
//		}
//		if (i>0&&i<lidarMsg->ranges.size()-1) {
//			double leftDelta = fabs(lidarMsg->ranges[i] - lidarMsg->ranges[i-1]);
//			double rightDelta = fabs(lidarMsg->ranges[i] - lidarMsg->ranges[i+1]);
//			if (leftDelta + rightDelta > 1)
//				continue;
//		}

		double temp_x = dist*sin(angle)*cos(yaw);
		double temp_y = dist*sin(angle)*sin(yaw);
		double temp_z = dist*cos(angle);

		p.x = -curr_x + cos(pitch)*temp_x + sin(pitch)*sin(roll)*temp_y + sin(pitch)*cos(roll)*temp_z;
		p.y = -curr_y + cos(roll)*temp_y - sin(roll)*temp_z;
		p.z = -curr_z - sin(pitch)*temp_x + cos(pitch)*sin(roll)*temp_y + cos(pitch)*cos(roll)*temp_z;
//		p.x = temp_x;
//		p.y = temp_y;
//		p.z = temp_z;

		tf::Quaternion pos(p.x,p.y,p.z,0); 
		pos = rotation * pos * rotation.inverse();
		Point currentPoint = {
			(float)(pos.getX() + translation.getX()),
			(float)(pos.getY() + translation.getY()),
			(float)(pos.getZ() + translation.getZ()),
			255, 255, 255
		};
		if (curr_z > 0.5)
			points.push_back(currentPoint);
	}
//	countLidar++;
//	double endTime = get_walltime();
	publish_object(pubCloud, &points);
//	toROSMsg(points,cloudMsg);
//	cloudMsg.header.stamp = lidarMsg->header.stamp;
//	cloudMsg.header.frame_id = "/map";
//	pubCloud->publish(cloudMsg);	
}

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"pcd");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	ros::Subscriber subLidar = n.subscribe("/scan",2,lidar_callback);
	ros::Subscriber subPose = n.subscribe("/mavros/local_position/pose",2,pose_callback);
	ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround",1);
	pubCloud = &pub1;
	
	cloudMsg.header.frame_id = "/map";
	cloudMsg.height = 1;
	cloudMsg.is_bigendian = false;
	cloudMsg.point_step = 32;
	cloudMsg.is_dense = true;
	sensor_msgs::PointField field;
	field.name = "x";
	field.offset = 0;
	field.datatype = 7;
	field.count = 1;
	cloudMsg.fields.push_back(field);
	field.name = "y";
	field.offset = 4;
	cloudMsg.fields.push_back(field);
	field.name = "z";
	field.offset = 8;
	cloudMsg.fields.push_back(field);
	field.name = "r";
	field.offset = 12;
	field.datatype = 2;
	cloudMsg.fields.push_back(field);
	field.name = "g";
	field.offset = 13;
	cloudMsg.fields.push_back(field);
	field.name = "b";
	field.offset = 14;
	cloudMsg.fields.push_back(field);

	hpcd = new HPCD();
	hpcd->leafSize = leaf_size;
	hpcd->maxSize = 8;
	hpcd->data = new HPoint*[8]();
	hpcd->deepCopy = true;
	
	ros::spin();
	savePCD("/home/rical/catkin_ws/src/rplidar_drone_ros/bag_pcd_files/mycloud.pcd", &points);
}
