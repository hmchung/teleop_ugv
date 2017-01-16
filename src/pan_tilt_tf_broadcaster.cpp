#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

std::string node_name = "tilt_tf_broadcaster";
int publish_rate = 10; //Hz

std::string frame_name_parent_pan = "pt_pan_stator";
std::string frame_name_child_pan = "pt_pan_rotor";

std::string frame_name_parent_tilt = "pt_tilt_stator";
std::string frame_name_child_tilt = "pt_tilt_rotor";

std::string topic_sub_pan_angle = "/ugv_1/cam_pan_state";
std::string topic_sub_tilt_angle = "/ugv_1/cam_tilt_state";

double pan_last_angle = 0.0;
double pan_curr_angle = 0.0;
double tilt_last_angle = 0.0;
double tilt_curr_angle = 0.0;

double temp_pan_angle = 0.0;
double temp_tilt_angle = 0.0;

ros::Time last_pan_msg_time;
ros::Time curr_pan_msg_time;
ros::Time last_tilt_msg_time;
ros::Time curr_tilt_msg_time;

ros::Time temp_time;

double pan_rate = 0.0;
double tilt_rate = 0.0;

void pan_angle_cb(const std_msgs::UInt8::ConstPtr _pan);
void tilt_angle_cb(const std_msgs::UInt8::ConstPtr _tilt);
double deg_to_rad(double _deg);

int main(int argc, char **argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param ("~");

	nh_param.param<std::string>("pan_topic", topic_sub_pan_angle, topic_sub_pan_angle);
	nh_param.param<std::string>("tilt_topic", topic_sub_tilt_angle, topic_sub_tilt_angle);
	nh_param.param<std::string>("parent_frame_pan", frame_name_parent_pan, frame_name_parent_pan);
	nh_param.param<std::string>("child_frame_pan", frame_name_child_pan, frame_name_child_pan);
	nh_param.param<std::string>("parent_frame_tilt", frame_name_parent_tilt, frame_name_parent_tilt);
	nh_param.param<std::string>("child_frame_tilt", frame_name_child_tilt, frame_name_child_tilt);

	ros::Subscriber pan_angle_sub = nh.subscribe<std_msgs::UInt8>
		(topic_sub_pan_angle, 10, pan_angle_cb);
	ros::Subscriber tilt_angle_sub = nh.subscribe<std_msgs::UInt8>
		(topic_sub_tilt_angle, 10, tilt_angle_cb);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent_pan, frame_name_child_pan)); 
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent_tilt, frame_name_child_tilt));
	ros::Rate loop_rate(publish_rate);
	while (ros::ok()){
		temp_time = ros::Time::now();
		temp_pan_angle = pan_curr_angle + pan_rate * (temp_time.toSec() - curr_pan_msg_time.toSec());
		temp_tilt_angle = tilt_curr_angle + tilt_rate * (temp_time.toSec() - curr_tilt_msg_time.toSec());

		q.setRPY(0.0, temp_tilt_angle, 0.0);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent_tilt, frame_name_child_tilt));	

		q.setRPY(0.0, 0.0, temp_pan_angle);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent_pan, frame_name_child_pan));	

		ros::spinOnce();
		loop_rate.sleep();
	}	

	return 0;
}

void pan_angle_cb(const std_msgs::UInt8::ConstPtr _pan){
	last_pan_msg_time = curr_pan_msg_time;
	curr_pan_msg_time = ros::Time::now();
	pan_last_angle = pan_curr_angle;
	pan_curr_angle = deg_to_rad(_pan->data - 90);
	// ROS_INFO("Pan angle: %.3f rad", pan_curr_angle);
	pan_rate = (pan_curr_angle - pan_last_angle) / (curr_pan_msg_time.toSec() - last_pan_msg_time.toSec());
}
void tilt_angle_cb(const std_msgs::UInt8::ConstPtr _tilt){
	last_tilt_msg_time = curr_tilt_msg_time;
	curr_tilt_msg_time = ros::Time::now();
	tilt_last_angle = tilt_curr_angle;
	tilt_curr_angle = deg_to_rad(90 - _tilt->data);
	// ROS_INFO("Tilt angle: %.3f rad", tilt_curr_angle);
	tilt_rate = (tilt_curr_angle - tilt_last_angle) / (curr_tilt_msg_time.toSec() - last_tilt_msg_time.toSec());
}

double deg_to_rad(double _deg){
	return _deg * 0.0174533;
}