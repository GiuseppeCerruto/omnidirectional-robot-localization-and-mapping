#include "omnirobot_loc_and_mapping/odom_tf.h"

TransformOdometry::TransformOdometry() {
	this->sub_odom = this->nh.subscribe("/odom", 1, &TransformOdometry::callback, this);
}

void TransformOdometry::callback(const nav_msgs::Odometry::ConstPtr& msg){

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));

  this->br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "odom_tf");
 	TransformOdometry my_transf_odom;
 	ros::spin();
 	return 0;
}
