#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <signal.h>

geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D w_pose; //wanted pose

float x;
float y;
float th;

float next_x;
float next_y;
float next_th;

double rate_hz = 10;
ros::Publisher pub_vl;

void get_pose(const geometry_msgs::Pose2D& msg){

	x = msg.x;
	y = msg.y;
	th = msg.theta * M_PI / 180;

}

void get_next_pose(const geometry_msgs::Pose2D& msg){

	next_x = msg.x;
	next_y = msg.y;
	next_th = msg.theta;

}

void mySigintHandler(int sig)
{
	std_msgs::Float32 last_vel;
	last_vel.data = 0;

	ROS_INFO_STREAM("Sending last vel: " << last_vel.data);
	pub_vl.publish(last_vel);
	ros::shutdown();
}


int main (int argc, char **argv){

	ros::init(argc,argv,"robot_control");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("robot_control initialized");																																							
	ROS_INFO_STREAM(ros::this_node::getName());

	// ROS_INFO_STREAM
	ros::Subscriber pose = nh.subscribe("/robot/pose", 1, &get_pose);
	ros::Subscriber next_pose = nh.subscribe("/robot/next_pose", 1, &get_next_pose); 
	ros::Publisher pub_st = nh.advertise<std_msgs::Float32> ("/AutoNOMOS_mini/manual_control/steering", 1);
	pub_vl = nh.advertise<std_msgs::Float32> ("/AutoNOMOS_mini/manual_control/velocity", 1);

	std_msgs::Float32 vel;
	std_msgs::Float32 ste;

	next_x = 0;
	next_y = 0;
	next_th = 0;

	signal(SIGINT, mySigintHandler);
	ros::Rate loop_rate(rate_hz);
	while (ros::ok())
	{
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
		double kp = 1;
		double ka = 50;
		double kb = 50;
		//Calculo del control (g=gamma)

		double x_pt_rob = cos(th) * next_x - sin(th) * next_y + x;
		double y_pt_rob = sin(th) * next_x + cos(th) * next_y + y;

		if (x_pt_rob < 0)
		{
			kp *= -1;
		}
		


		vel.data = kp * ro;
		ste.data = (ka * alpha + kb * beta) * 180 / M_PI;

		ROS_INFO_STREAM("next_pose: " << next_x << ", " << next_y << ", " << next_th);
		ROS_INFO_STREAM("actu_pose: " << x << ", " << y << ", " << th);
		ROS_INFO_STREAM("vel: " << vel.data);
		ROS_INFO_STREAM("ste: " << ste.data);

		pub_st.publish(ste);
		pub_vl.publish(vel);
		ros::spinOnce();
		loop_rate.sleep ();
	}
	//Falta publicar las señales de control

	return 0;
}
