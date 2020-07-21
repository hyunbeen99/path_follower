#include "path_follower/path_follower.h"

void PathFollower::initSetup(){
	sub_o_ = nh_.subscribe("/odom", 1 , &PathFollower::odomCallback, this);
	//sub_p_ = nh_.subscribe("/path", 1 , &PathFollower::followCallback, this);
	pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd",10);
}

// get current pose
void PathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg){
	lx = odomsg->pose.pose.position.x;
	ly = odomsg->pose.pose.position.y;
	lz = odomsg->pose.pose.position.z;
	tf::Quaternion q(
		odomsg->pose.pose.orientation.x,
		odomsg->pose.pose.orientation.y,
		odomsg->pose.pose.orientation.z,
		odomsg->pose.pose.orientation.w); tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
}

// follow final path
/*
void PathFollower::followCallback(vector<OdomDouble> path){
	
	// 2 method
	double dist = 20.0;
	for (int i=path_flag;i<path.size();i++) {
		double dist_l = sqrt(pow(path.at(i).getX() - lx, 2) + pow(path.at(i).getY() - ly, 2));
		if (dist > dist_l) {
			dist = dist_l;
			path_flag = i;
		}
	}

	if (path_flag == path.size()-1) {
		ackerData_.drive.steering_angle = calcSteer(path.at(path_flag+1).getX(), path.at(path_flag+1).getY());
		pre_steer_ = ackerData_.drive.steering_angle;
		ackerData_.drive.speed = 2;
	} else {
		ackerData_.drive.steering_angle = pre_steer_;
		ackerData_.drive.speed = 2;
	}

	pub_.publish(ackerData_);
}
*/

// calculate steering angle between current pose and goal pose
double PathFollower::calcSteer(double ggx, double ggy){

	double dx = (ggx-lx);
    double dy = (ggy-ly);
	//double local_x, local_y;
	double dist = sqrt(dx*dx + dy*dy);

	double ab_degree = abs(atan2(ggy-ly, ggx-lx));
	double true_angle;
	double degree = atan2(dy, dx);

	double steer;
	pre_angle_ = ackerData_.drive.steering_angle;


	if (yaw >= 0 && yaw < M_PI/2){	
		local_x = dx*cos(yaw) + dy*sin(yaw);
		local_y = -dx*sin(yaw) + dy*cos(yaw);
		steer = atan(local_y/local_x);
		if (local_x>0) steer = steer; 
		else {
			if (local_y>0) steer = M_PI - abs(steer);
			else steer = -(M_PI - abs(steer));
	//		cout << "state1" << endl;
		}
	}
	else if (yaw >= M_PI/2 && yaw < M_PI){
		local_x = -dx*cos(M_PI-yaw) + dy*sin(M_PI-yaw);
		local_y = -dx*sin(M_PI-yaw) - dy*cos(M_PI-yaw);
		steer = atan(local_y/local_x);
		if (local_x>0) steer = steer; 
		else {
			if (local_y>0) steer = M_PI - abs(steer);
			else steer = -(M_PI - abs(steer));
	//		cout << "state2" << endl;
		}
	}
	else if (yaw >= -M_PI/2 && yaw < 0){
		local_x = dx*cos(abs(yaw)) - dy*sin(abs(yaw));
		local_y = dx*sin(abs(yaw)) + dy*cos(abs(yaw));
		steer = atan(local_y/local_x);
		if (local_x>0) steer = steer; 
		else{ 
			if (local_y>0) steer = M_PI - abs(steer);
			else steer = -(M_PI - abs(steer));
	//		cout << "state3" << endl;
			}
		}
	else if (yaw >= -M_PI && yaw < -M_PI/2){
		local_x = -dx*cos(M_PI-abs(yaw)) - dy*sin(M_PI-abs(yaw));
		local_y = dx*sin(M_PI-abs(yaw)) - dy*cos(M_PI-abs(yaw));
		steer = atan(local_y/local_x);
		if (local_x>0) steer = steer; 
		else{ 
			if (local_y>0) steer = M_PI - abs(steer);
			else steer = -(M_PI - abs(steer));
		}	
	}
	return -steer*180/M_PI;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "path_follower_node");
	
	PathFollower pf;
	pf.initSetup();
	ros::spin();
	return 0;
}
