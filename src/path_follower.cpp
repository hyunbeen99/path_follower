#include "path_follower/path_follower.h"

void PathFollower::initSetup(){
	sub_o_ = nh_.subscribe("/odom", 1 , &PathFollower::odomCallback, this);
//	sub_p_ = nh_.subscribe("/path", 1 , &PathFollower::followCallback, this);
	pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd",10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

	nh_.setParam("/isGlobalPathChanged", false);
	loadGlobalPath();
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
		odomsg->pose.pose.orientation.w); 
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
}

// follow final path
void PathFollower::follow(){
	bool isChanged;
	nh_.getParam("/isGlobalPathChanged", isChanged);

	if (isChanged) {
		loadGlobalPath();
		nh_.setParam("/isGlobalPathChanged", false);
	}
	
	// find closest point in global path
	double dist = 100.0;
	for (int i=path_flag;i<global_path_.size();i++) {
		double dist_l = sqrt(pow(global_path_.at(i).getX() - lx, 2) + pow(global_path_.at(i).getY() - ly, 2));
		if (dist > dist_l) {
			dist = dist_l;
			path_flag = i;
		}
	}


	cout << "0.x -> " << global_path_.at(0).getX() << endl;
	cout << "0.y -> " << global_path_.at(0).getY() << endl;
	cout << "path_flag point.x -> " << global_path_.at(path_flag).getX() << endl;
	cout << "path_flag point.y -> " << global_path_.at(path_flag).getY() << endl;
	cout << "last.x -> " << global_path_.back().getX() << endl;
	cout << "last.y -> " << global_path_.back().getY() << endl;
	cout << endl;

	cout << "path flag: " << path_flag << endl;
	cout << "size : " << global_path_.size() << endl;

	if (path_flag != global_path_.size()-1) {
		ackerData_.drive.steering_angle = calcSteer(global_path_.at(path_flag+1).getX(), global_path_.at(path_flag+1).getY());
		pre_steer_ = ackerData_.drive.steering_angle;
		ackerData_.drive.speed = 2;
	} else { // the last index
		ackerData_.drive.steering_angle = pre_steer_;
		ackerData_.drive.speed = 2;
	}

	visualize(global_path_);
	pub_.publish(ackerData_);
}

void PathFollower::loadGlobalPath() {
	if(obs_detect_flag_ < 1) {
		bool isChanged;
		nh_.getParam("/isGlobalPathChanged", isChanged);

		ifstream file;
		global_path_.clear();

		if (!isChanged) { // use initial global path
			file.open(GLOBAL_PATH_FILE);
		} else { // obstacle detected from planner -> use changed global path
			cout << "path changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			file.open(NEW_GLOBAL_PATH_FILE);
			obs_detect_flag_++;
		}
		
		if (file.is_open()) {

			string line;

			while (getline(file, line)) {

				istringstream ss(line);

				vector<string> odomString;
				string stringBuffer;
				while (getline(ss, stringBuffer, ',')) {
					odomString.push_back(stringBuffer);
				}

				OdomDouble odomDouble(stod(odomString.at(0)), stod(odomString.at(1)), stod(odomString.at(2)));
				global_path_.push_back(odomDouble);
			}

			file.close();
		}
	}
}

void PathFollower::visualize(vector<OdomDouble> path){
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
	points.color.b = 1.0f;

	geometry_msgs::Point p;

	for (auto point : path) {
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		points.points.push_back(p);
	}

	marker_pub_.publish(points);
}



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

	// warp
	while(ros::ok()) {
		pf.follow();
		ros::spinOnce();
	}

	return 0;
}
