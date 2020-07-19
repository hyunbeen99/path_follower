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

class PathFollower{
private:
    //node 
	ros::NodeHandle nh_;
    ros::Publisher pub_, marker_pub_;
    ros::Subscriber sub_o_;
    
    //value
    double lx, ly, lz;
    double roll, pitch, yaw;
    double pre_angle_;
    
    //flag
    int path_flag = 0;

    //messages
    ackermann_msgs::AckermannDriveStamped ackerData_;
                
public:
    void initSetup();
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg);
    void follow(vector<OdomDouble> path);
    void visualize(vector<OdomDouble> global_path);

    double calcSteer(double ggx, double ggy);
    
    vector<OdomDouble> loadGlobalPath();
};
