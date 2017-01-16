//Communicate with UGV via ROSSERIAL
//Assuming UGV motors are controlled by Arduino/Servo.h
//Motor control value ranges from 0 to 180, integer
//Subscribe to:
//	- Abstract control signal: 	/cmd_vel
//	- Encoder value:			/[robot_name]/base/motor_enc
//Publish:
//	- Control signal: 			/[robot_name]/raw_control_signal
//	- Estimated odometry:		/[robot_name]/current_vel

//Openloop control mechanism
//	- Lookup table

//Feedback control mechanism
//Sources of feedback
//	- robot pose
//	- wheel odometry

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt16MultiArray.h>

#define pi 3.14

std::string nodeName = "speed_controller";
std::string robotName = "ugv_1";
std::string subTopic_targetVel = "/motor_pwr";
std::string subTopic_encoderVal = "/base/motor_enc";
std::string pubTopic_outputVel = "/raw_control_signal";
std::string pubTopic_wheelVel = "/wheel_vel";
int indoor_mode = 1;

void motorEnc_callback(const std_msgs::UInt16MultiArray::ConstPtr& _enc_val_array);
void targetVel_callback(const geometry_msgs::Twist::ConstPtr& _vel);
void publish_motorControlVal();
void publish_wheelVel();

int curr_enc_vals[4];
ros::Time curr_enc_time;

int last_enc_vals[4];
ros::Time last_enc_time;

geometry_msgs::Twist target_vel, current_vel;
ros::Time last_input_time;
int publish_rate = 1;
int default_rate = 10;
int loop_count = 0;
double lin_accel = 0.08, ang_accel = 0.1;
double lin_vel_lim = 1.0, ang_vel_lim = 1.0;

double v_wheel_FR, v_wheel_FL, v_wheel_BR, v_wheel_BL;
double v_wheels_R, v_wheels_L;
double v_lin, v_rot;
double wheel_dia = 0.270; //m
int tick_per_rev = 190;
long max_tick_val = 600000;
double dist_per_tick = 0.0;
double robot_w = 0.480;  //width, dist between 2 side wheels, unit: meter
double robot_l = 0.360; //lenth, dist between front & back wheels, unit: m

int physical_output[4];
int physical_output_compensator[4] = {0, 0, 0, 0};
int physical_output_max = 180;
int physical_output_min = 0;
double speed_limit_upper = 1.0;
double speed_limit_lower = -1.0;

void resetTargetVel();
void catchUp();
void prepare_raw_output();
double response_lookup(double desired_vel);

double map(double input, double input_min, double input_max, double output_min, double output_max);
double bound(double input, double bound_limit);
double min(double input_1, double input_2);
double max(double input_1, double input_2);

std_msgs::UInt16MultiArray motor_cmd_msg;
ros::Publisher control_vel_pub;
ros::Publisher wheel_vel_pub;

int main(int argc, char** argv){
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("robot_name", robotName, robotName);
	nh_param.param<std::string>("topic_input_vel", subTopic_targetVel, subTopic_targetVel);
	nh_param.param<std::string>("topic_input_enc", subTopic_encoderVal, subTopic_encoderVal);
	nh_param.param<std::string>("topic_output_vel", pubTopic_outputVel, pubTopic_outputVel);
	nh_param.param<std::string>("topic_wheel_vel", pubTopic_wheelVel, pubTopic_wheelVel);
	nh_param.param<int>("indoor_mode", indoor_mode, indoor_mode);
	nh_param.param<double>("lin_accel", lin_accel, lin_accel);
	nh_param.param<double>("ang_accel", ang_accel, ang_accel);
	nh_param.param<double>("lin_vel_lim", lin_vel_lim, lin_vel_lim);
	nh_param.param<double>("ang_vel_lim", ang_vel_lim, ang_vel_lim);

	subTopic_encoderVal = robotName + subTopic_encoderVal;
	subTopic_targetVel = "/turtlebot_teleop/cmd_vel";//robotName + subTopic_targetVel;
	pubTopic_outputVel = robotName + pubTopic_outputVel;
	pubTopic_wheelVel = robotName + pubTopic_wheelVel;

	//motor_cmd_msg.data_length = 4;
	//motor_cmd_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
	motor_cmd_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_msg.layout.dim[0].label = "motor_cmd";
	motor_cmd_msg.layout.dim[0].size = 4;
	motor_cmd_msg.layout.dim[0].stride = 1*4;
	motor_cmd_msg.layout.data_offset = 0;
	motor_cmd_msg.data.resize(4);// = (unsigned int *)malloc(sizeof(unsigned int)*4);

	ros::Subscriber motor_enc_sub = nh.subscribe<std_msgs::UInt16MultiArray>(subTopic_encoderVal, 10, motorEnc_callback);
	ros::Subscriber target_vel_sub = nh.subscribe<geometry_msgs::Twist>(subTopic_targetVel, 10, targetVel_callback);
	control_vel_pub = nh.advertise<std_msgs::UInt16MultiArray>(pubTopic_outputVel, 10);
	wheel_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(pubTopic_wheelVel, 10);

	//Util
	// dist_per_tick = dist_per_rev / tick_per_rev = (pi * d^2 / 4) / tick_per_rev
	dist_per_tick = (pi * wheel_dia * wheel_dia / 4.0) / (double)tick_per_rev;
	
	ros::Rate loop_rate(default_rate);
	while (ros::ok()){
		loop_count++;
		if (ros::Time::now() > last_input_time + ros::Duration(2.0)){
			resetTargetVel();
		}
		if(loop_count % publish_rate == 0){
			// catchUp();
			//control_vel_pub.publish(current_vel);
			prepare_raw_output();
			publish_motorControlVal();
			publish_wheelVel();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void targetVel_callback(const geometry_msgs::Twist::ConstPtr& _vel){
	target_vel = *_vel;
	target_vel.linear.x = bound(target_vel.linear.x, lin_vel_lim);
	target_vel.angular.z = bound(target_vel.angular.z, ang_vel_lim);
	last_input_time = ros::Time::now();
}

void motorEnc_callback(const std_msgs::UInt16MultiArray::ConstPtr& _enc_val_array){
	last_enc_time = curr_enc_time;
	curr_enc_time = ros::Time::now();
	for (int i = 0; i < 4; i++){
		last_enc_vals[i] = curr_enc_vals[i];
		curr_enc_vals[i] = _enc_val_array->data[i];
	}
	double temp_duration = curr_enc_time.toSec() - last_enc_time.toSec();
	v_wheel_FL = (double)(curr_enc_vals[0] - last_enc_vals[0]) * dist_per_tick / temp_duration;
	v_wheel_FR = (double)(curr_enc_vals[1] - last_enc_vals[1]) * dist_per_tick / temp_duration;
	v_wheel_BR = (double)(curr_enc_vals[2] - last_enc_vals[2]) * dist_per_tick / temp_duration;
	v_wheel_FL = (double)(curr_enc_vals[3] - last_enc_vals[3]) * dist_per_tick / temp_duration;  

	if (indoor_mode == 1){
		v_wheels_L = v_wheel_FL;
		v_wheels_R = v_wheel_FR;
	} else {
		v_wheels_L = (v_wheel_FL + v_wheel_BL) / 2.0;
		v_wheels_R = (v_wheel_FR + v_wheel_BR) / 2.0;
	}
	v_lin = (v_wheels_R + v_wheels_L) / 2.0;
	v_rot = (v_wheels_R - v_wheels_L) / robot_w;
}

void publish_motorControlVal(){
	for (int i = 0; i < 4; i++){
		motor_cmd_msg.data[i] = physical_output[i] - physical_output_compensator[i];
	}
	control_vel_pub.publish(motor_cmd_msg);
}

void publish_wheelVel(){
	geometry_msgs::TwistStamped tempTwist;
	tempTwist.header.stamp = ros::Time::now();
	tempTwist.twist.linear.x = v_lin;
	tempTwist.twist.angular.z = v_rot;
	wheel_vel_pub.publish(tempTwist);
}

void resetTargetVel(){
	target_vel.linear.x = 0.0;
	target_vel.angular.z = 0.0;
}

void catchUp(){
	double target_speed = target_vel.linear.x;
	double control_speed = current_vel.linear.x;
	double target_turn = target_vel.angular.z;
	double control_turn = current_vel.angular.z;

  	if 		(target_speed > control_speed)  control_speed = min( target_speed, control_speed + lin_accel);
	else if (target_speed < control_speed)	control_speed = max( target_speed, control_speed - lin_accel);
	else 								    control_speed = target_speed;

	if 		(target_turn > control_turn)	control_turn = min( target_turn, control_turn + ang_accel);
	else if (target_turn < control_turn)	control_turn = max( target_turn, control_turn - ang_accel);
	else 								    control_turn = target_turn;

	current_vel.linear.x = control_speed;
	current_vel.angular.z = control_turn;
}

void prepare_raw_output(){
	double motor_vals[4];
	double control_speed = target_vel.linear.x;
	double control_turn = target_vel.angular.z;

	motor_vals[0] = control_speed - control_turn;
	motor_vals[1] = control_speed + control_turn;
	motor_vals[2] = control_speed + control_turn;
	motor_vals[3] = control_speed - control_turn;

	for (int i = 0; i < 4; i++){
		physical_output[i] = motor_vals[i];
	}
}

double map(double input, double input_min, double input_max, double output_min, double output_max){
	if (input > input_max) input = input_max;
	if (input < input_min) input = input_min;
	return output_min + (input - input_min) / (input_max - input_min) * (output_max - output_min);
}

double bound(double input, double bound_limit){
	if (input > abs(bound_limit)) return bound_limit;
	else if (input < -abs(bound_limit)) return -bound_limit;
	else return input;
}
double min(double input_1, double input_2){
	if(input_1 >= input_2) return input_2;
	else return input_1;
}
double max(double input_1, double input_2){
	if (input_1 >= input_2) return input_1;
	else return input_2;
}