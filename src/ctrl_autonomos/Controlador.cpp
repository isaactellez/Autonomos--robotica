#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <signal.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#define STEERING_MIN -28
#define STEERING_MAX 28

geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D w_pose; //wanted pose

float x;
float y;
float th;

float next_x;
float next_y;
float next_th;

float kp ;
float ka ;
float kb ;

uint32_t seq;

double rate_hz = 10;
ros::Publisher pub_vl;

std::string car_name = "/AutoNOMOS_mini";
std::string world = "world";

void get_pose(const geometry_msgs::Pose2D& msg){

	x = msg.x;
	y = msg.y;
	th = msg.theta;// * M_PI / 180;

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
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("robot_control initialized");																																							
	ROS_INFO_STREAM(ros::this_node::getName());

	nh.param<float>("kp", kp, 1);
	ROS_INFO_STREAM("The steering kp values is: " << kp);

	nh.param<float>("ka", ka, 0.1);
	ROS_INFO_STREAM("The steering ka values is: " << ka);

	nh.param<float>("kb", kb, -0.1);
	ROS_INFO_STREAM("The steering kb values is: " << kb);

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

	tf::TransformListener tfListener;
	geometry_msgs::PointStamped old_pt;
	geometry_msgs::PointStamped new_pt;
	// tfListener.waitForTransform(world, car_name, ros::Time::now(), ros::Duration(3.0) );


	while (ros::ok())
	{
		//Falta suscribirse a robot/pose y robot/next_pose para obtener x, x* (next_x),
		//y, y* (next_y) y theta
		try{
			// tfListener.transformPose("ackermann", pose_stp_w, pose_stp_ack);
			seq++;
			old_pt.header.seq = seq;
			// old_pt.header.stamp = ros::Time::now();
			old_pt.header.frame_id = "/ground_plane";//car_name;
			old_pt.point.x = next_x;
			old_pt.point.y = next_y;
			tfListener.transformPoint(car_name, old_pt, new_pt);

			ROS_INFO_STREAM("New Point: " << new_pt);
			// tfListener.lookupTransform("/ground_plane", "/steer_link", ros::Time(0), transform_steering);
			// ROS_INFO_STREAM("After transformPose: " << pose_stp_w << pose_stp_ack);
			// ROS_INFO_STREAM("After transformPose: " );

			// tf::Quaternion q = transform_steering.getRotation();
			// ROS_INFO_STREAM(msg.pose[steering_link].orientation) ;
			// ROS_INFO_STREAM(q);
			// tf::Matrix3x3 m(q);
		
			// m.getRPY(roll, pitch, yaw); //USING yaw
			// ROS_INFO_STREAM("-------------------------------------------------------");
			// ROS_INFO_STREAM("( " << transform_steering.getOrigin().x() << " , " << transform_steering.getOrigin().y() << " , " << transform_steering.getOrigin().z() << " )");
			// ROS_INFO_STREAM("( " << roll << " , " << pitch << " , " << yaw << " )");
			// ROS_INFO_STREAM("rads: " << -yaw << " deg: " << yaw * 180/ M_PI );
			// curr_steering = yaw * 180/ M_PI;
			// ROS_INFO_STREAM("-------------------------------------------------------");

			// car_pose.x = transform_steering.getOrigin().x();
			// car_pose.y = transform_steering.getOrigin().y();
			// car_pose.theta = yaw;

			// ROS_INFO_STREAM(transform_steering.getRotation() );
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			// ROS_INFO_STREAM("ZERO Quaternion!!");
		}

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
		
		//Calculo del control (g=gamma)

		double x_pt_rob = new_pt.point.x; //cos(th) * next_x - sin(th) * next_y + x;
		double y_pt_rob = new_pt.point.y; //sin(th) * next_x + cos(th) * next_y + y;
		
		if (x_pt_rob < 0)
		{
			vel.data = -kp * ro;
			ste.data = -(ka * alpha + kb * beta) * 180 / M_PI;	
		} else {
			vel.data = kp * ro;
			ste.data = (ka * alpha + kb * beta) * 180 / M_PI;
		}
		

		if (ste.data > STEERING_MAX)
		{
			ste.data = STEERING_MAX;
		} else if(ste.data < STEERING_MIN)
		{
			ste.data = STEERING_MIN;
		}
		ROS_INFO_STREAM("point_robot: " << x_pt_rob << ", " << y_pt_rob);
		ROS_INFO_STREAM("theta: " << th);
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
