//Subscribe to angle command, twist
//Publish to head controller, range 0 -> 90 (deg)
//Publish tf
//		- pan_stator -> pan_rotor
//		- tilt_stator -> tilt_rotor

#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#define pi 3.14159265359

std::string node_name = "pan_tilt_cam_controller";
int loop_count = 0;
int broad_cast_freq = 20; 	//Hz
int servo_control_freq = 2; //Hz

double angle_rad_in_pan = 0.0;
double angle_rad_in_tlt = 0.0;

int angle_deg_out_pan = 90;
int angle_deg_out_tlt = 90;

std::string topic_sub_pan_angle = "/pan_tilt_head/control_cmd/pan";
std::string topic_sub_tlt_angle = "/pan_tilt_head/control_cmd/tlt";
std::string topic_sub_pan_tilt_twist = "/pan_tilt_head/control_cmd/twist";

std::string topic_pub_pan_angle = "/ugv_1/cam_servo/pan";
std::string topic_pub_tlt_angle = "/ugv_1/cam_servo/tlt";

std::string pan_parent_frame = "pan_stator";
std::string pan_child_frame = "pan_rotor";
std::string tlt_parent_frame = "tlt_stator";
std::string tlt_child_frame = "tlt_rotor";

void pan_tilt_twist_cb(const geometry_msgs::Twist::ConstPtr& _twist);
int rad_to_deg(double rad);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	ros::Subscriber pan_angle_cmd_sub = nh.subscribe<geometry_msgs::Twist>
		(topic_sub_pan_tilt_twist, 10, pan_tilt_twist_cb);

	ros::Publisher pan_angle_cmd_pub = nh.advertise<std_msgs::Int8>
		(topic_pub_pan_angle, 10);

	ros::Publisher tlt_angle_cmd_pub = nh.advertise<std_msgs::Int8>
		(topic_pub_tlt_angle, 10);	

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	double x, y, z, w;
	x = q.getX();	y = q.getY();	z = q.getZ();	w = q.getW();

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), pan_parent_frame, pan_child_frame)); 
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tlt_parent_frame, tlt_child_frame)); 

	ros::Rate loop_rate(broad_cast_freq);
	loop_count = 0;

	int count_every_control = broad_cast_freq / servo_control_freq;
	//Broad cast tf at 20Hz
	//Control servo at 2Hz;
	while (ros::ok()){
		if (loop_count % count_every_control == 0){
			angle_deg_out_pan = 90 + rad_to_deg(angle_rad_in_pan);
			angle_deg_out_tlt = 90 + rad_to_deg(angle_rad_in_tlt);

			std_msgs::Int8 temp_msg_pan, temp_msg_tlt;
			temp_msg_pan.data = angle_deg_out_pan;
			temp_msg_tlt.data = angle_deg_out_tlt; 
			pan_angle_cmd_pub.publish(temp_msg_pan);
			tlt_angle_cmd_pub.publish(temp_msg_tlt);
		}

		q.setRPY(0.0, 0.0, angle_rad_in_pan);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), pan_parent_frame, pan_child_frame));	

		q.setRPY(0.0, angle_rad_in_tlt, 0.0);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tlt_parent_frame, tlt_child_frame));	

		loop_count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void pan_tilt_twist_cb(const geometry_msgs::Twist::ConstPtr& _twist){
	angle_rad_in_pan = _twist->angular.z;
	angle_rad_in_tlt = _twist->angular.y;

	if (angle_rad_in_pan > (pi/2)) angle_rad_in_pan = pi/2;
	if (angle_rad_in_pan < (-pi/2)) angle_rad_in_pan = -pi/2;

	if (angle_rad_in_tlt > (pi/2)) angle_rad_in_tlt = pi/2;
	if (angle_rad_in_tlt < (-pi/2)) angle_rad_in_tlt = -pi/2;	
}

int rad_to_deg(double rad){
	double temp_deg = rad / pi * 180.0;
	return temp_deg;
}