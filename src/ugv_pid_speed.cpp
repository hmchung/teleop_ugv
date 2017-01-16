#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "teleop_ugv/set_pid.h"

std::string node_name = "ugv_pid_speed";
int pid_freq = 50; //10Hz
double dt = 0.02;

bool has_ugv_vel = false, has_ref_vel = false;

ros::Time curr_vel_stamp, last_vel_stamp;
ros::Time curr_ref_stamp, last_ref_stamp;

double basic_gain_lin, basic_gain_ang;

double time_interval = 0; //seconds
double curr_vel_lin, curr_vel_ang, last_vel_lin, last_vel_ang;
double curr_err_lin, last_err_lin, curr_err_ang, last_err_ang;
double err_accu_lin, err_accu_ang, err_rate_lin, err_rate_ang;

double curr_ref_lin, curr_ref_ang, last_ref_lin, last_ref_ang;

double kp_lin, ki_lin, kd_lin;
double p_lin, i_lin, d_lin, pid_lin;
double kp_ang, ki_ang, kd_ang;
double p_ang, i_ang, d_ang, pid_ang;

std::string topic_sub_ugv_vel = "/ugv_1/ugv_vel";
std::string topic_sub_ref_vel = "/ugv_1/cmd_vel";
std::string topic_pub_ctrl_vel = "/ugv_1/raw_ctrl";

void init_pid();
void reset_I_lin();
void reset_I_ang();
void calculate_pid_lin();
void calculate_pid_ang();
bool update_pid_gains(teleop_ugv::set_pid::Request &req, teleop_ugv::set_pid::Response &res);

void ugv_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& _twist);
void target_vel_cb(const geometry_msgs::Twist::ConstPtr& _twist);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	ros::Subscriber sub_ugv_vel = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_ugv_vel, 10, ugv_vel_cb);
	ros::Subscriber sub_ref_vel = nh.subscribe<geometry_msgs::Twist>(topic_sub_ref_vel, 10, target_vel_cb);
	ros::Publisher pub_ctrl_vel = nh.advertise<geometry_msgs::Twist>(topic_pub_ctrl_vel, 10);

	ros::ServiceServer pid_service = nh.advertiseService("set_pid_vel", update_pid_gains);

	ros::Rate loop_rate(pid_freq);

	init_pid();
	while (ros::ok()){
		if (has_ugv_vel && has_ref_vel){
			calculate_pid_lin();
			calculate_pid_ang();
			geometry_msgs::Twist ctrl_msg;
			ctrl_msg.linear.x = pid_lin;
			ctrl_msg.angular.z = pid_ang;
			pub_ctrl_vel.publish(ctrl_msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void init_pid(){
	basic_gain_lin = 0.055, basic_gain_ang = 0.225;

	kp_lin = 0.06, ki_lin = 0.0005, kd_lin = 0.0;
	p_lin = 0.0, i_lin = 0.0, d_lin = 0.0, pid_lin = 0.0;
	kp_ang = 0.01, ki_ang = 0.0003, kd_ang = 0.0;
	p_ang = 0.0, i_ang = 0.0, d_ang = 0.0, pid_ang = 0.0;
}

bool update_pid_gains(teleop_ugv::set_pid::Request &req, teleop_ugv::set_pid::Response &res){
	int _sys_id = req.sys_id;
	if (_sys_id == 0 || _sys_id == 1){
		double _basic_gain = req.basic_gain;
		double _k_p = req.k_p;
		double _k_i = req.k_i;
		double _k_d = req.k_d;
		if (_sys_id == 0) { //linear
			basic_gain_lin = _basic_gain;
			kp_lin = _k_p;
			ki_lin = _k_i;
			kd_lin = _k_d;
			ROS_INFO("[pid_speed_controller] Linear pid gains set");
		}
		if (_sys_id == 1) { //angular
			basic_gain_ang = _basic_gain;
			kp_ang = _k_p;
			ki_ang = _k_i;
			kd_ang = _k_d;
			ROS_INFO("[pid_speed_controller] Angular pid gains set");
		}
		return true;
	} else {
		return false;
	}
}

void reset_I_lin(){
	err_accu_lin = 0.0;
}

void reset_I_ang(){
	err_accu_ang = 0.0;
}

void calculate_pid_lin(){
	last_err_lin = curr_err_lin;
	curr_err_lin = curr_ref_lin - curr_vel_lin;

	err_accu_lin = err_accu_lin + curr_err_lin;
	err_rate_lin = (curr_err_lin - last_err_lin) / dt;

	p_lin = curr_err_lin * kp_lin;
	i_lin = err_accu_lin * ki_lin;
	d_lin = err_rate_lin * kd_lin;
	if (curr_ref_lin == 0.0){
		pid_lin = p_lin + i_lin + d_lin;
	} else 
		pid_lin = p_lin + i_lin + d_lin + curr_ref_lin / fabs(curr_ref_lin) * basic_gain_lin;
}

void calculate_pid_ang(){
	last_err_ang = curr_err_ang;
	curr_err_ang = curr_ref_ang - curr_vel_ang;

	err_accu_ang = err_accu_ang + curr_err_ang;
	err_rate_ang = (curr_err_ang - last_err_ang) / dt;

	p_ang = curr_err_ang * kp_ang;
	i_ang = err_accu_ang * ki_ang;
	d_ang = err_rate_ang * kd_ang;
	if (curr_ref_ang == 0.0){
		pid_ang = p_ang + i_ang + d_ang;
	} else 
		pid_ang = p_ang + i_ang + d_ang + curr_ref_ang / fabs(curr_ref_ang) * basic_gain_ang;	
}

void ugv_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& _twist){
	last_vel_stamp = curr_vel_stamp;
	curr_vel_stamp = ros::Time::now();

	last_vel_lin = curr_vel_lin;
	last_vel_ang = curr_vel_ang;

	curr_vel_lin = _twist->twist.linear.x;
	curr_vel_ang = _twist->twist.angular.z;

	has_ugv_vel = true;
}

void target_vel_cb(const geometry_msgs::Twist::ConstPtr& _twist){
	last_ref_stamp = curr_ref_stamp;
	curr_ref_stamp = ros::Time::now();

	last_ref_lin = curr_ref_lin;
	last_ref_ang = curr_ref_ang;

	curr_ref_lin = _twist->linear.x;
	curr_ref_ang = _twist->angular.z;

	has_ref_vel = true;
}