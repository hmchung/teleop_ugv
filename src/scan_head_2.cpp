#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

using namespace tf;
using namespace std;

#define PI 3.1415926

std::string nodeName = "scan_head";
std::string robotName = "ugv_1";
std::string headName = "head_tilt_joint";
std::string topicName_pub_tiltAngle = "ugv_1/tilt_angle_raw";

int control_frequency = 250; //Hz
int control_period = 20; //millisec
bool scanning_mode = true;
bool current_sweep_direction = true; //true ~ up
double current_target_angle = 0.0; //radian
double increment_per_cycle = 0.0;

bool broadcast_tf = true;
std::string frame_name_parent = "stator";
std::string frame_name_child = "rotor";

bool publish_virtual_imu = true;
std::string topic_pub_virtual_imu = "scan_head/virtual_imu";

double angle_lower_deg = -90.0;
double angle_upper_deg = 90.0;
double scan_rate_deg = 10.0; 	// deg/sec

double angle_lower_rad = -1.57;
double angle_upper_rad = 1.57;
double scan_rate_rad = 0.175;

bool input_unit_rad = true;
double angle_lower_input = -1.57;
double angle_upper_input = 1.57;
double scan_rate_input = 0.175;

std_msgs::Float64 angle_msg;

int main(int argc, char** argv){
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("robot_name", robotName, robotName);
	nh_param.param<std::string>("topic_pub_tilt_angle", topicName_pub_tiltAngle, topicName_pub_tiltAngle);
	nh_param.param<bool>("scanning_mode", scanning_mode, scanning_mode);
	nh_param.param<bool>("input_unit_rad", input_unit_rad, input_unit_rad);
	nh_param.param<double>("angle_lower_input", angle_lower_input, angle_lower_input);
	nh_param.param<double>("angle_upper_input", angle_upper_input, angle_upper_input);
	nh_param.param<double>("scan_rate_input", scan_rate_input, scan_rate_input);

	nh_param.param<bool>("broadcast_tf", broadcast_tf, broadcast_tf);
	nh_param.param<std::string>("frame_name_parent", frame_name_parent, frame_name_parent);
	nh_param.param<std::string>("frame_name_child", frame_name_child, frame_name_child);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	if (broadcast_tf == true){
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child));
	}

	if (input_unit_rad == true){
		angle_lower_rad = angle_lower_input;
		angle_upper_rad = angle_upper_input;
		scan_rate_rad = scan_rate_input;
	} else {
		angle_lower_rad = angle_lower_input / 180.0 * PI;
		angle_upper_rad = angle_upper_input / 180.0 * PI;
		scan_rate_rad = scan_rate_input / 180.0 * PI;
	}
	control_period = 1000 / control_frequency;
	increment_per_cycle = scan_rate_rad / 1000.0 * (control_period);	

	ros::Publisher tilt_angle_pub = nh.advertise<std_msgs::Float64>(topicName_pub_tiltAngle, 10);
	angle_msg.data = current_target_angle;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("[%s] Start scanning", robotName.c_str());
	ROS_INFO("[%s] Scan from %.3f to %.3f, %.3f rad/sec,  %.3f rad per cycle", 
		robotName.c_str(), angle_lower_rad, angle_upper_rad, scan_rate_rad, increment_per_cycle);
	while (ros::ok()){
		if (scanning_mode == true){
			if (current_target_angle >= angle_upper_rad) current_sweep_direction = false;
			if (current_target_angle <= angle_lower_rad) current_sweep_direction = true;
			if (current_sweep_direction == true){
				current_target_angle += increment_per_cycle;
			} else {
				current_target_angle -= increment_per_cycle;
			}
			angle_msg.data = (current_target_angle / PI * 180) + 90;
			// if (current_target_angle >= angle_upper_rad - 0 * increment_per_cycle
			// 	|| current_target_angle <= angle_lower_rad + 0 * increment_per_cycle)
			tilt_angle_pub.publish(angle_msg);

			if (broadcast_tf == true){
				q.setRPY(0, -current_target_angle, 0);
				transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child));
			}			
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}
}