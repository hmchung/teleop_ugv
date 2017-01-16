#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

std::string node_name = "tilt_tf_broadcaster";

std::string frame_name_parent = "stator";
std::string frame_name_child = "rotor";

std::string topic_name_head_imu = "/imu/data";

double x=0.0, y=0.0, z=0.0, w=0.0;
double rol=0.0, pit=0.0, yaw=0.0;

void head_imu_cb(const sensor_msgs::Imu::ConstPtr _imu_data);

int main(int argc, char **argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param ("~");

	nh_param.param<std::string>("imu_topic", topic_name_head_imu, topic_name_head_imu);
	nh_param.param<std::string>("parent_frame", frame_name_parent, frame_name_parent);
	nh_param.param<std::string>("child_frame", frame_name_child, frame_name_child);

	ros::Subscriber head_imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_name_head_imu, 10, head_imu_cb);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	x = q.getX();	y = q.getY();	z = q.getZ();	w = q.getW();

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child)); 

	ros::Rate loop_rate(30);
	while (ros::ok()){
		//q = tf::Quaternion(x, y, z, w);
		q.setRPY(0.0, pit, 0.0);
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );	
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child));	
		ros::spinOnce();
		loop_rate.sleep();
	}	

	return 0;
}

void head_imu_cb(const sensor_msgs::Imu::ConstPtr _imu_data){
	x = _imu_data->orientation.x;
	y = _imu_data->orientation.y;
	z = _imu_data->orientation.z;
	w = _imu_data->orientation.w;

	tf::Quaternion q = tf::Quaternion(x, y, z, w);
	tf::Matrix3x3 m(q);
	double m_roll, m_pitch, m_yaw;
	m.getRPY(m_roll, m_pitch, m_yaw);

	rol = m_roll;
	pit = m_pitch;
	yaw = m_yaw;
}