/* * Author: Anis Koubaa
 * Year: 2016
 * Prince Sultan University/Gaitech Robotics. */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"
#include <sstream> 

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
nav_msgs::Odometry gazebo_pos;

const double PI = 3.14159265359;

void move(double speed, double distance);
void rotate (double angular_speed, double angle);
double degrees2radians(double angle_in_degrees);
double quad2rad(nav_msgs::Odometry msg);
double targetangle(double x5, double y5);
double angleerror(double x4,double y4);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
void moveGoal(double x3, double y3,double distance_tolerance);
void gridClean();

void poseCallback(const nav_msgs::Odometry::ConstPtr & msg){
	gazebo_pos=*msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_cleaner_node");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 50);
	pose_subscriber = n.subscribe("odom", 50, poseCallback);

	ros::Rate idle(0.2);
	idle.sleep();
	ROS_INFO("\n\n\n******START TESTING************\n");

	//moveGoal(4,4,0.2);

	gridClean(); 

	ros::spin();
	return 0;
}

double quad2rad(nav_msgs::Odometry msg){
	double xx=msg.pose.pose.orientation.x;
	double yy=msg.pose.pose.orientation.y;
	double zz=msg.pose.pose.orientation.z;
	double ww=msg.pose.pose.orientation.w;
	return atan2(2*(ww*zz+xx*yy),1-2*(yy*yy+zz*zz));
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

double getDistance(double x1, double y1){
	return sqrt(pow((x1-gazebo_pos.pose.pose.position.x),2)+pow((y1-gazebo_pos.pose.pose.position.y),2));
}

double angleerror (double x4,double y4){
	double currentangle=quad2rad(gazebo_pos);
	double angleto=targetangle(x4,y4);
	double finalerror=atan2(sin(angleto-currentangle),cos(angleto-currentangle));
	return finalerror;
}

double targetangle(double x5, double y5){
	return atan2(y5-gazebo_pos.pose.pose.position.y,x5-gazebo_pos.pose.pose.position.x);
}


void moveGoal(double x3, double y3, double distance_tolerance){

	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(4);
	double Ea = 0.0, Ea1=0, Ea2=0;
	double kpa = 0.1, kia=0.1,kda=0.1;
	double Ed = 0.0, Ed1=0, Ed2=0;
	double kpd = 0.1, kid=0.1,kdd=0.1;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	double thres=0;
	do{
		/****** Proportional Controller ******/
		if (getDistance(x3,y3)>1){thres=0.1;}
		else if (getDistance(x3,y3)>0.5){thres=0.25;}
		else{thres=0.60;}

		if (fabs(angleerror(x3,y3))>thres){
			vel_msg.linear.x =0;
			vel_msg.angular.z = kpa*angleerror(x3,y3);
			if (fabs(vel_msg.angular.z)<0.03){
				if (vel_msg.angular.z>0){vel_msg.angular.z=0.03;}
				else if(vel_msg.angular.z<0){vel_msg.angular.z=-0.03;}
				else {}
			}
		}
		else {
			vel_msg.linear.x =kpd*getDistance(x3,y3);
			if (vel_msg.linear.x>0.3){vel_msg.linear.x=0.3;}
			else if (vel_msg.linear.x<0.05){vel_msg.linear.x=0.05;}
			else{}
			vel_msg.angular.z = 0.2*angleerror(x3,y3);
		}
		velocity_publisher.publish(vel_msg);
		loop_rate.sleep();
		ros::spinOnce();
		ROS_INFO("direction: [%f],target:[%f],distance:[%f]",quad2rad(gazebo_pos),targetangle(x3,y3),getDistance(x3,y3));
	}while((getDistance(x3,y3)>distance_tolerance)&(ros::ok()));

	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();
}

void move(double speed, double distance){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =speed;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);//miliseconds
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();//publish from buffer
		loop_rate.sleep();
	}while((current_distance<distance)&(ros::ok()));
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();//publish from buffer
	loop_rate.sleep();
}

void rotate (double angular_speed, double relative_angle){

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =angular_speed;

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(200);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while((current_angle<relative_angle)&(ros::ok()));

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

void brake(){
	geometry_msgs::Twist vel_msg;
	ros::Rate loop(0.5);
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop.sleep();
}

void gridClean(){

    double accuracy=0.15;
	moveGoal(4.5,0,accuracy);
	moveGoal(4.5,4.5,accuracy);
	moveGoal(-4.5,4.5,accuracy);
	moveGoal(-4.5,-4.5,accuracy);
	moveGoal(-3.5,-4.5,accuracy);
	moveGoal(-3.5,3.5,accuracy);
	moveGoal(-2.5,3.5,accuracy);
	moveGoal(-2.5,-4.5,accuracy);
	moveGoal(-1.5,-4.5,accuracy);
	moveGoal(-1.5,3.5,accuracy);
	moveGoal(-0.5,3.5,accuracy);
	moveGoal(-0.5,-4.5,accuracy);
	moveGoal(0.5,-4.5,accuracy);
	moveGoal(0.5,3.5,accuracy);
	moveGoal(1.5,3.5,accuracy);
	moveGoal(1.5,-4.5,accuracy);
	moveGoal(2.5,-4.5,accuracy);
	moveGoal(2.5,3.5,accuracy);
	moveGoal(3.5,3.5,accuracy);
	moveGoal(3.5,-4.5,accuracy);
	moveGoal(4.5,-4.5,accuracy);
	moveGoal(4.5,0,accuracy);
	moveGoal(0,0,accuracy);
	
}