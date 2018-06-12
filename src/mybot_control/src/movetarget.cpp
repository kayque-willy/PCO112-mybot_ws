#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

using namespace std;

double destX;
double destY;
double curX, curY, curDir;

void odomCallback(const nav_msgs::Odometry& msg){
	curX = msg.pose.pose.position.x;
	curY = msg.pose.pose.position.y;
	curDir = tf::getYaw( msg.pose.pose.orientation );
}

int main(int argc, char **argv){

    ros::init(argc, argv, "moveTarget");
    ros::NodeHandle nh;

    //Publisher de velocidade
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mybot/cmd_vel", 10);
	//Subscriber da posição
	ros::Subscriber sub = nh.subscribe("/mybot/odom", 10, &odomCallback);
	
	double max_vel=0.2, dist, theta, z;
	//ros::param::get("~max_vel", max_vel);
	//ROS_INFO_STREAM("Velocidade maxima configurada para " << max_vel);

	ros::Rate rate(1);
	geometry_msgs::Twist msg;
	while (ros::ok()){
		cout << "Entre com o destino X Y\n";
		cin >> destX >> destY;	
		while (ros::ok()){
			ros::spinOnce();
			
			dist = hypot(destX - curX, destY - curY); // distancia ate destino
			theta = atan2(destY - curY, destX - curX); // orientacao [0-2pi]	 
			z = theta - curDir; // correcao necessaria

			if (dist < 0.1) break;

			msg.linear.x = (dist > 1? max_vel : max_vel/2.0);
			msg.angular.z = (fabs(z) > 0.2 ? z : 0);
			
			ROS_INFO_STREAM("\nTheta: " << theta << " Angulo atual: " << curDir << "\nDistancia: " << dist << " Direcao: " << z << " Velocidade: " << msg.linear.x);
			
			pub.publish(msg);
			rate.sleep();
		}
	}
	return 0;
} 
