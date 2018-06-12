#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		cv::Mat gray, src = cv_bridge::toCvShare(msg, "bgr8")->image;
		std::ostringstream oss;	
		cv::vector<cv::Vec3f> circles;
		
		//Converte para escala cinza
		cv::cvtColor( src, gray, CV_BGR2GRAY );
		 
		// Reduz o ruido na detecao do circulo
		cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
		 
		// Aplica a Hough Transform para encontrar circulos
		cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );
		
		// Desenha os circulos detectados
		for( size_t i = 0; i < circles.size(); i++ ){
			  cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			  int radius = cvRound(circles[i][2]);     
			  cv::circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// circle center     
			  cv::circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline
			  oss << "center : " << center << "\nradius : " << radius << std::endl;
			  ROS_INFO_STREAM( oss.str() );
		}
	 
		// Mostra os resultados
		cv::imshow( "View", src );
		cv::waitKey(30);
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "camerasub");
	ros::NodeHandle nh;
	cv::namedWindow( "View", CV_WINDOW_NORMAL);
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/mybot/camera1/image_raw", 1, &imageCallback);
	ros::spin();
	cv::destroyWindow("View");
	return 0;
}
