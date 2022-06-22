#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class TransformOdometry {
public:
    TransformOdometry();
private:
    ros::NodeHandle nh; 
    tf::TransformBroadcaster br;
    ros::Subscriber sub_odom;   // Subscriber to odometry incoming from the bag file

    void callback(const nav_msgs::Odometry::ConstPtr& msg);
};