#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

void odomCallback(const nav_msgs::Odometry& msg){
	// A pose atual do bot é dada pelas seguintes variáveis:
	ROS_INFO_STREAM("\nPosicao atual ("
					<< " [x]: " << msg.pose.pose.position.x 
					<< ", [y]: " << msg.pose.pose.position.y 
					<< ", [theta]: " << tf::getYaw( msg.pose.pose.orientation ) << " )");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "mybot_control");
	ros::NodeHandle nh;
	
	// Criar um objeto subscriber.
	ros::Subscriber odomSub = nh.subscribe("/odom", 1000, &odomCallback);

	ros::spin();
	return 0;
}
