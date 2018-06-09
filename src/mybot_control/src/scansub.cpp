#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>

void laserScanCallback(const sensor_msgs::LaserScan& msg){
	std::ostringstream oss;
	oss << "Ranges = [ ";
	for (int i = 0; i < msg.ranges.size(); i++){
		oss << msg.ranges[i] << " ";
	}
	oss << " ];";
	ROS_INFO_STREAM( oss.str() );
}

int main(int argc, char** argv){
	ros::init(argc, argv, "laser_scan_sub");
	ros::NodeHandle nh;
	ros::Subscriber scan_sub = nh.subscribe("mybot/laser/scan",
	1000, &laserScanCallback);
	ros::spin();
}
