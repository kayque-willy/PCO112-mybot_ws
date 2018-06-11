#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;	
		cv::imshow("view", frame);
		cv::waitKey(30);
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "camerasub");
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/mybot/camera1/image_raw", 1, &imageCallback);
	ros::spin();
	cv::destroyWindow("view");
	return 0;
}
