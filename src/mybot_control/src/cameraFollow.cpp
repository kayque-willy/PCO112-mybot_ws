#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <stdio.h>

double velX = 0, velZ = 0;
int circuloEscolhido;
bool encontrou = false;

void procurarCirculo(){
	ROS_INFO_STREAM("Nenhum circulo encontrado, procurando...");
	velZ = 0.15;
	encontrou = false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try{
		//Objetos MAT que recebem a imagem gray que e a escala em cinza e src que e a imagem raw
		cv::Mat gray, src = cv_bridge::toCvShare(msg, "bgr8")->image; //Converte a mensagem com a imagem para opencv
		//Vetor que recebe o circulo
		cv::vector<cv::Vec3f> circles;
		
		//Converte src para escala cinza em gray
		cv::cvtColor( src, gray, CV_BGR2GRAY );
		// Reduz o ruido na detecao do circulo em gray
		cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
		// Aplica a Hough Transform para encontrar circulos de gray para o vetor circles
		cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );
		
		//Verifica se encontrou o circulo
		if(encontrou == false){
			if(circles.size() == 0){
				procurarCirculo();
			}else{
				ROS_INFO_STREAM("Circulo encontrado!");
				for( size_t i = 0; i < circles.size(); i++ ){
					circuloEscolhido = i;
				}
				encontrou = true;
				velZ = -0.15;
			}
			velX = 0;
		}else{
			if(circles.size() != 0){
				int radius = cvRound(circles[circuloEscolhido][2]); 
				if(radius < 230){
					ROS_INFO_STREAM("Circulo encontrado, indo em direcao.");
					if(cvRound(circles[circuloEscolhido][0])>440){
						velZ = 0.05;
						velX = 3;
					}else if(cvRound(circles[circuloEscolhido][0])<370){
						velZ = -0.05;
						velX = 3;
					}else{
						velZ = 0;
						velX = 10;
					}
				}else{
					ROS_INFO_STREAM("Circulo encontrado, chegou.");
					velZ = 0;
					velX = 0;
				}
			}else{
				encontrou = false;	
			}
		}
		
		// Desenha os circulos detectados em src 
		for( size_t i = 0; i < circles.size(); i++ ){
			//Ponto central do circulo
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			//Raio do circulo
			int radius = cvRound(circles[i][2]);     
			//Desenha o circulo em src para exibicao na tela
			cv::circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// circulo verde pequeno do centro     
			cv::circle(src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );// circulo vermelho ao redor
			ROS_INFO_STREAM("\nCirculo: " << (i + 1) << "\nCentro : " << center << "\nRaio : [" << radius << "]" << std::endl);
		}
		
		// Mostra os resultados no visualizador de imagem
		cv::imshow("View", src);
		cv::waitKey(30);
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Nao e possivel converter de '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "camerasub");
	ros::NodeHandle nh;
	geometry_msgs::Twist msg;
	
	//Publisher de velocidade
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mybot/cmd_vel", 10);
	
	//Subscribe da imagem da camera
	cv::namedWindow("View", CV_WINDOW_NORMAL);
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/mybot/camera1/image_raw", 1, &imageCallback);
	
	ros::Rate rate(2);
	while (ros::ok()){
			ros::spinOnce();	
			msg.linear.x = velX;
			msg.angular.z = velZ;
			ROS_INFO_STREAM("\nVelocidade: Angular: [" << velZ << "] Linear: [" << velX << "]");
			pub.publish(msg);
			rate.sleep();
	}
	
	cv::destroyWindow("View");
	return 0;
}
