#include <ros/ros.h>
#include <std_msgs/UInt8.h>

std::string nodeName = "ugv_heart";
std::string robotName = "ugv_1";
std::string topicName_pub_heartbeat = "/heart_beat";

std_msgs::UInt8 heart_beat;

int main(int argc, char** argv){
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("robot_name", robotName, robotName);
	nh_param.param<std::string>("topic_heartbeat", topicName_pub_heartbeat, topicName_pub_heartbeat);

	ros::Publisher heartbeat_pub = nh.advertise<std_msgs::UInt8>(robotName + topicName_pub_heartbeat, 10);
	heart_beat.data = 1;

	ros::Rate loop_rate(2);

	ROS_INFO("[%s] Heart start beating", robotName.c_str());
	while (ros::ok()){
		heartbeat_pub.publish(heart_beat);
		ros::spinOnce();
		loop_rate.sleep();
	}
}