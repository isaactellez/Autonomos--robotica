#include <ros/ros.h>
#include "std_msgs/geometry_msgs.h"

#include <iostream>
#include <vector>
#include <math.h>

std_msgs::geometry_msgs pose;
std_msgs::geometry_msgs w_pose; //wanted pose

float x;
float y;
float th;

float next_x;
float next_y;
float next_th;

void get_pose(const std_msgs::geometry_msgs& msg){

	x = msg.x;
	y = msg.y;
	th = msg.theta;

}

void get_next_pose(const std_msgs::geometry_msgs& msg){

	next_x = msg.x;
	next_y = msg.y;
	next_th = msg.theta;

}

int main (int argc, char **argv){

	ros::init(argc,argv,"robot_control");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("robot_control initialized");																																							
	ROS_INFO_STREAM(ros::this_node::getName());

	// ROS_INFO_STREAM
	ros::Subscriber pose = nh.subscribe("/robot/pose", 1, &get_pose);
	ros::Subscriber next_pose = nh.subscribe("/robot/next_pose", 1, &get_next_pose); 
	ros::Publisher pub = nh.advertise<PointCloud> ("pointCloud_vision", 1);

	ros::Rate loop_rate(rate_hz);
	
	ros::spinOnce();
	loop_rate.sleep ();

	//Falta suscribirse a robot/pose y robot/next_pose para obtener x, x* (next_x),
	//y, y* (next_y) y theta
	float dX = next_x - x;
	float dY = next_y - y;
	float dX2 = pow(dX,2);
	float dY2 = pow(dY,2);

	//Calculo de Ro
	float ro = sqrt(dX2 + dY2);

	//Calculo de Alfa
	float div = dY/dX;
	float alpha = atan(div) - th;

	//Calculo de Beta
	float beta = -th - alpha;

	//Coeficientes de control
	double kp;
	double ka;
	double kb;
	//Calculo del control (g=gamma)
	float v = kp * ro;
	float g = ka * alpha + kb * beta;

	//Falta publicar las señales de control

	return 0;
}
