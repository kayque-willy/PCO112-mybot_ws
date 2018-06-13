#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
     
    image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/mybot/camera1/image_raw", 1);
	
	cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    
   ros::Rate loop_rate(5);
   while (nh.ok()) { 
       cap >> frame; // get a new frame from camera
       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
   
       pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
    }
    
}
