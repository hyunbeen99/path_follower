#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"

#include "math.h"
#include "vector"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>

#include "path_follower/OdomDouble.h"

#define _USE_MATH_DEFINES
#define GLOBAL_PATH_FILE "/home/hyeonbeen/path.txt"

using namespace std;

double lx, ly, lz, yaw_d, yaw;
double pre_angle;

ros::Publisher marker_pub;
ros::Subscriber sub_o;
ackermann_msgs::AckermannDriveStamped ackerData_;

int path_flag = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg){
	lx = odomsg->pose.pose.position.x;
	ly = odomsg->pose.pose.position.y;
	lz = odomsg->pose.pose.position.z;
	tf::Quaternion q(
		odomsg->pose.pose.orientation.x,
		odomsg->pose.pose.orientation.y,
		odomsg->pose.pose.orientation.z,
		odomsg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);

	yaw_d = yaw*180/M_PI;
	//cout << yaw_d << endl;
}

double calcSteer(double ggx, double ggy){

	double dx = (ggx-lx);
    double dy = (ggy-ly);
	double local_x, local_y;
	double dist = sqrt(dx*dx + dy*dy);
	double dist_hp = 1; //50cm

	double ab_degree = abs(atan2(ggy-ly, ggx-lx));
	double true_angle;
	double degree = atan2(dy, dx);

	double steer;
	pre_angle = ackerData_.drive.steering_angle;

	if (dist < dist_hp){
 	    ackerData_.drive.speed = 0;
		ackerData_.drive.steering_angle = 0;
		ackerData_.drive.brake = 200;
	}
	else if(dist < 1.5 && dist > dist_hp){
		ackerData_.drive.speed = 1;
		ackerData_.drive.steering_angle = pre_angle;

	}
	else{
	//	cout << "yaw: " << yaw_d << endl;
		if (yaw_d >= 0 && yaw_d < 90){	
			local_x = dx*cos(yaw) + dy*sin(yaw);
			local_y = -dx*sin(yaw) + dy*cos(yaw);
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
	//		cout << "state1" << endl;
		}
		else if (yaw_d >= 90 && yaw_d < 180){
			local_x = -dx*cos(M_PI-yaw) + dy*sin(M_PI-yaw);
			local_y = -dx*sin(M_PI-yaw) - dy*cos(M_PI-yaw);
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
	//		cout << "state2" << endl;
		}
		else if (yaw_d >= -90 && yaw_d < 0){
			local_x = dx*cos(abs(yaw)) - dy*sin(abs(yaw));
			local_y = dx*sin(abs(yaw)) + dy*cos(abs(yaw));
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
	//		cout << "state3" << endl;
		}
		else if (yaw_d >= -180 && yaw_d < -90){
			local_x = -dx*cos(M_PI-abs(yaw)) - dy*sin(M_PI-abs(yaw));
			local_y = dx*sin(M_PI-abs(yaw)) - dy*cos(M_PI-abs(yaw));
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
			/*if (dy<0 && dx<0){
				steer = -(M_PI-steer);
				cout << "####" << endl;}
			else if (dy>0 && dx<0) steer = M_PI-abs(steer);*/
	//		cout << "state4" << endl;
		}	
	}

	return -steer*180/M_PI;
}

void follow(vector<OdomDouble> path) {
	double dist = 0.5;

	for (int i=path_flag;i<path.size();i++) {
		double dist_l = sqrt(pow(path.at(i).getX() - lx, 2) + pow(path.at(i).getY() - ly, 2));
		if (dist > dist_l) {
			dist = dist_l;
			path_flag = i;
		}
	}

	ackerData_.drive.steering_angle = calcSteer(path.at(path_flag+1).getX(), path.at(path_flag+1).getY());
	ackerData_.drive.speed = 2;

	/*
	if (path.size() != path_flag+1) {
		double dist_gap = sqrt(pow(path.at(path_flag+1).getX() - path.at(path_flag).getX(), 2) + pow(path.at(path_flag+1).getY() - path.at(path_flag).getY(), 2));
		double dist_l = sqrt(pow(path.at(path_flag+1).getX() - lx, 2) + pow(path.at(path_flag+1).getY() - ly, 2));

		
		if (dist_gap > dist_l) { // add flag
			path_flag++;
		}
	}
		*/
}

void printGlobalPath(vector<OdomDouble> path) {
	cout << "global path loading..." << endl;

	for (auto o : path) {
		cout << endl;
		cout << "x : " << o.getX() << endl;
		cout << "y : " << o.getY() << endl;
		cout << "z : " << o.getZ() << endl;
		cout << "o_x : " << o.getOX() << endl;
		cout << "o_y : " << o.getOY() << endl;
		cout << "o_z : " << o.getOZ() << endl;
		cout << "o_w : " << o.getOW() << endl;
	}

	cout << "global path loaded successfully" << endl;
	cout << endl;
}

vector<OdomDouble> loadGlobalPath() {
	vector<OdomDouble> path;
	ifstream file;
	file.open(GLOBAL_PATH_FILE);

	if (file.is_open()) {

		string line;

		while (getline(file, line)) {

			istringstream ss(line);

			vector<string> odomString;
			string stringBuffer;
			while (getline(ss, stringBuffer, ',')) {
				odomString.push_back(stringBuffer);
			}

			OdomDouble odomDouble(stod(odomString.at(0)), stod(odomString.at(1)), stod(odomString.at(2)), stod(odomString.at(3)), stod(odomString.at(4)), stod(odomString.at(5)), stod(odomString.at(6)));
			path.push_back(odomDouble);
		}

		file.close();
	}

	follow(path);
 //	printGlobalPath(path);

	return path;
}

/*
void visualize(double x, double y, double z){
	visualization_msgs::Marker points;

	points.header.frame_id = "map";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 1; 
	points.scale.y = 1;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	p.x = x;
	p.y = y;
	p.z = z;
	points.points.push_back(p);

	marker_pub.publish(points);
}
*/

void visualize(vector<OdomDouble> global_path){
	visualization_msgs::Marker points;

	points.header.frame_id = "map";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	for (auto point : global_path) {
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		points.points.push_back(p);
	}

	marker_pub.publish(points);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
    
	ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd",10);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    ackerData_.drive.steering_angle = 0;
    ackerData_.drive.speed = 0;

	while(true) {
		sub_o = nh.subscribe("/odom", 1 , odomCallback);
		visualize(loadGlobalPath());
		cout << path_flag << endl;
		pub.publish(ackerData_);
		ros::spinOnce();
	}

	return 0;
}
