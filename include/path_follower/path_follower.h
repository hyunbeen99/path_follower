#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
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
#define NEW_GLOBAL_PATH_FILE "/home/hyeonbeen/new_path.txt"
#define DIST_HP 0.5

using namespace std;

class PathFollower{
private:
    //node 
	ros::NodeHandle nh_;
    ros::Publisher pub_, marker_pub_;
    ros::Subscriber sub_o_;
    ros::Subscriber sub_p_;
    
    //value
    double lx, ly, lz;
    double roll, pitch, yaw;
    double pre_angle_;
    double local_x, local_y; 

	vector<OdomDouble> global_path_;

    //flag
    int path_flag = 0;
	double pre_steer_ = 0.0;
    int obs_detect_flag_ = 0;

	int temp_flag = 0;

    //messages
    ackermann_msgs::AckermannDriveStamped ackerData_;
                
public:
	bool start_flag_ = false;
    void initSetup();

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg);
    void follow();
	void loadGlobalPath();
    void visualize(vector<OdomDouble> path);
    double calcSteer(double ggx, double ggy); 
    PathFollower(){
        initSetup();
    }    
};
