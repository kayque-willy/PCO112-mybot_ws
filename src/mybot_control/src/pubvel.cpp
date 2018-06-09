#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> // Para usar rand() e RAND_MAX

int main(int argc, char **argv)
{	
	// Inicializa 
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mybot/cmd_vel", 10);

	srand(time(0));

	ros::Rate rate(2);
	
	while (ros::ok())
	{
		geometry_msgs::Twist msg;
		msg.linear.x = rand()*1.0/RAND_MAX; // [0, 1]
		msg.angular.z = rand()*2.0/RAND_MAX - 1;  // [-1, 1]

		ROS_INFO_STREAM("Enviando comando de velocidade:" 
			<< "linear = " << msg.linear.x 
			<< " angular = " << msg.angular.z);
		
		vel_pub.publish(msg);

		rate.sleep();
	}
	return 0;
}
