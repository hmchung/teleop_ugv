//patrol and find objects
//		define patrol rotation rate
//if object detected, rotate to the direction of object
//		detection by ROI
//		define satisfactory window
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#define image_width 640
#define image_height 480
#define image_diagonal 800
#define cam_view_angle 1.3962634

#define PI 3.1415926

std::string node_name = "scan_head_1";

std::string topic_sub_roi = "/gun_cam/hole/roi";
std::string topic_sub_start_flag = "/start_search_and_shoot";
std::string topic_pub_finished_search_and_shoot = "/finished_search_and_shoot";
std::string topic_pub_shooting = "/shooting";
std::string topic_pub_aim_angle = "/head_tilt_joint/command";

double angle_per_pixel = 0.001745329;
double scale_factor = 0.7;
double allowable_offset_alpha = 0.03;
double allowable_offset_beta = 0.06;

double target_aim_angle = 0.0;
double angle_lower_rad = -1.57;
double angle_upper_rad = 1.57;
double scan_rate_rad = 0.175;
int control_frequency = 20; //Hz
int control_period = 250; //millisec
bool scanning_mode = true;
bool current_sweep_direction = true; //true ~ up
double current_target_angle = 0.0; //radian
double increment_per_cycle = 0.006;

std_msgs::Float64 aim_angle_msg;

bool object_detected_flag = false;
ros::Time last_detection_time;
ros::Time last_shooting_time;

double detection_timeout = 0.5; //sec
double shooting_inteval = 2; //sec

sensor_msgs::RegionOfInterest current_roi;
bool allow_shooting = false;


void object_detection_cb(const sensor_msgs::RegionOfInterest::ConstPtr& _roi);
void start_mission_flag_cb(const std_msgs::Int8::ConstPtr& _flag);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	ros::Subscriber roi_sub = nh.subscribe<sensor_msgs::RegionOfInterest>
		(topic_sub_roi, 10, object_detection_cb);
	ros::Subscriber mission_start_sub = nh.subscribe<std_msgs::Int8>
		(topic_sub_start_flag, 10, start_mission_flag_cb);

	ros::Publisher shooting_pub = nh.advertise<std_msgs::UInt8>
		(topic_pub_shooting, 10);
	ros::Publisher mission_finished_pub = nh.advertise<std_msgs::Int8>
		(topic_pub_finished_search_and_shoot, 10);

	ros::Publisher aim_angle_pub = nh.advertise<std_msgs::Float64>
		(topic_pub_aim_angle, 10);

	ros::Rate loop_rate(control_frequency);
	while (ros::ok()){
		//keep patroling and keeping track of current angle and sweeping direction
		//if roi detect, stop and go to roi, keep track of current roi
		//if object within window, and allow shooting, shoot
		if (ros::Time::now().toSec() - last_detection_time.toSec() > detection_timeout){
			object_detected_flag = false;
		}
		if (object_detected_flag == true){
			//Check the direction to ROI
			int x_offset = current_roi.x_offset;
			int y_offset = current_roi.y_offset;
			int height = current_roi.height;
			int width = current_roi.width;

			double scale_factor = 0.8;

			double beta_hi_lim = (image_width/2 - x_offset) * angle_per_pixel * scale_factor;
			double beta_lo_lim = (image_width/2 - x_offset - width) * angle_per_pixel * scale_factor;
			double alpha_hi_lim = (image_width/2 - y_offset) * angle_per_pixel * scale_factor;
			double alpha_lo_lim = (image_width/2 - y_offset - height) * angle_per_pixel * scale_factor; 

			double beta_center = (beta_hi_lim + beta_lo_lim) / 2.0;
			double alpha_center = (alpha_hi_lim + alpha_lo_lim) / 2.0;

			//ROS_INFO("Beta_center = %.3f , current center = %.3f", beta_center, current_target_angle);
			double angle_difference = beta_center;// - current_target_angle;
			ROS_INFO("Angle difference = %.3f", angle_difference);
			if (beta_center > allowable_offset_beta){
				current_target_angle += increment_per_cycle;
				if (current_target_angle >= angle_upper_rad) current_target_angle = angle_upper_rad; 	
			}
			if (beta_center < -allowable_offset_beta){
				current_target_angle -= increment_per_cycle;
				if (current_target_angle <= angle_lower_rad) current_target_angle = angle_lower_rad;
			}
			if (fabs(angle_difference) < allowable_offset_beta){
				if (allow_shooting == true && 
					ros::Time::now().toSec() - last_shooting_time.toSec() > shooting_inteval){ 
					std_msgs::UInt8 shooting_msg;
					if (alpha_center >= 0.0){
						shooting_msg.data = 180;
					}
					if (alpha_center < 0.0){
						if (alpha_center > - allowable_offset_alpha) shooting_msg.data = 150;
						else shooting_msg.data = 120;
					}
					shooting_pub.publish(shooting_msg);
					last_shooting_time = ros::Time::now();
				}
			}
		} else {
			// object_detected_flag = false;
			if (current_target_angle >= angle_upper_rad) current_sweep_direction = false;
			if (current_target_angle <= angle_lower_rad) current_sweep_direction = true;
			if (current_sweep_direction == true){
				current_target_angle += increment_per_cycle;
			} else {
				current_target_angle -= increment_per_cycle;
			}
		}
		aim_angle_msg.data = current_target_angle; //(current_target_angle / PI * 180) + 90;
		aim_angle_pub.publish(aim_angle_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void object_detection_cb(const sensor_msgs::RegionOfInterest::ConstPtr& _roi){
	last_detection_time = ros::Time::now();
	current_roi = *_roi;
	object_detected_flag = true;
}

void start_mission_flag_cb(const std_msgs::Int8::ConstPtr& _flag){
	int received_signal = _flag->data;
	switch(received_signal){
		case 0: allow_shooting = false; break;
		case 1: allow_shooting = true; break;
		default: allow_shooting = false;
	}
}