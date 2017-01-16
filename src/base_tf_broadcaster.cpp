#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

std::string node_name = "tilt_tf_broadcaster";

std::string frame_name_parent = "stator";
std::string frame_name_child = "rotor";

std::string topic_name_head_imu = "/base_imu/global_position/local";
std::string topic_name_station = "/station_imu/global_position/local";

double x=0.0, y=0.0, z=0.0, w=0.0;
double t_x = 0.0, t_y = 0.0, t_z = 0.0;
double t_x_corrected, t_y_corrected, t_z_corrected;
double rol=0.0, pit=0.0, yaw=0.0;

double correction_t_x = 0.0, correction_t_y = 0.0, correction_t_z = 0.0;
double origin_t_x = 0.0, origin_t_y = 0.0, origin_t_z = 0.0;
double common_origin_t_x = 0.0, common_origin_t_y = 0.0, common_origin_t_z = 0.0;
double origin_station_t_x = 0.0, origin_station_t_y = 0.0, origin_station_t_z = 0.0;
bool has_data_imu = false;
bool has_data_station = false;
int data_count_imu = 0;
int data_count_station = 0;
int data_count_required_imu = 100;
int data_count_required_station = 50;

bool differential_mode = false;

void base_pose_cb(const nav_msgs::Odometry::ConstPtr _imu_data);
void station_pose_cb(const nav_msgs::Odometry::ConstPtr _imu_data);

int main(int argc, char **argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param ("~");

	nh_param.param<std::string>("imu_topic", topic_name_head_imu, topic_name_head_imu);
	nh_param.param<std::string>("parent_frame", frame_name_parent, frame_name_parent);
	nh_param.param<std::string>("child_frame", frame_name_child, frame_name_child);

	nh_param.param<bool>("differential_mode", differential_mode, differential_mode);
	nh_param.param<std::string>("station_topic", topic_name_station, topic_name_station);

	ros::Subscriber head_imu_sub = nh.subscribe<nav_msgs::Odometry>(topic_name_head_imu, 10, base_pose_cb);
	ros::Subscriber station_imu_sub = nh.subscribe<nav_msgs::Odometry>(topic_name_station, 10, station_pose_cb);
	if (differential_mode == true){
		ROS_WARN("[base_tf_broadcaster] DIFFERNTIAL MODE");
	} else {
		// has_data_imu = true;
		has_data_station = true;
	}

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
		if (has_data_imu && has_data_station){
			q.setRPY(0.0, 0.0, yaw);
			transform.setOrigin( tf::Vector3(t_x_corrected, t_y_corrected, 0.0) );	
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child));	
			
			// transform.setOrigin( tf::Vector3(t_x, t_y, t_z) );	
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, "/imu_actual"));
				
			// transform.setOrigin( tf::Vector3(correction_t_x, correction_t_y, correction_t_z) );	
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, "/station_correction"));
		}
		ros::spinOnce();
		loop_rate.sleep();
	}	

	return 0;
}

void base_pose_cb(const nav_msgs::Odometry::ConstPtr _pose){
	x = _pose->pose.pose.orientation.x;
	y = _pose->pose.pose.orientation.y;
	z = _pose->pose.pose.orientation.z;
	w = _pose->pose.pose.orientation.w;

	tf::Quaternion q = tf::Quaternion(x, y, z, w);
	tf::Matrix3x3 m(q);
	double m_roll, m_pitch, m_yaw;
	m.getRPY(m_roll, m_pitch, m_yaw);

	rol = m_roll;
	pit = m_pitch;
	yaw = m_yaw;

	if (has_data_imu == false){
		t_x = _pose->pose.pose.position.x;
		t_y = _pose->pose.pose.position.y;
		t_z = _pose->pose.pose.position.z;

		if (data_count_imu >= data_count_required_imu){
			has_data_imu = true;			
			ROS_WARN("[base_imu] started: imu UTM origin x=%.3f, y=%.3f, z=%.3f", origin_t_x, origin_t_y, origin_t_z);
		} else {
			data_count_imu += 1;

			origin_t_x = origin_t_x * (data_count_imu - 1) / data_count_imu + t_x / data_count_imu; 
			origin_t_y = origin_t_y * (data_count_imu - 1) / data_count_imu + t_y / data_count_imu;
			origin_t_z = origin_t_z * (data_count_imu - 1) / data_count_imu + t_z / data_count_imu;			
		}

	} else {
		t_x = _pose->pose.pose.position.x - origin_t_x;
		t_y = _pose->pose.pose.position.y - origin_t_y;
		t_z = _pose->pose.pose.position.z - origin_t_z;
	}

	if (has_data_station == true){
		t_x_corrected = t_x - correction_t_x;
		t_y_corrected = t_y - correction_t_y;
		t_z_corrected = t_z - correction_t_z;
	}		   
}

void station_pose_cb(const nav_msgs::Odometry::ConstPtr _pose){
	double station_t_x = _pose->pose.pose.position.x;
	double station_t_y = _pose->pose.pose.position.y;
	double station_t_z = _pose->pose.pose.position.z;

	data_count_station += 1;
	origin_station_t_x = (origin_station_t_x * (data_count_station - 1) + station_t_x) / data_count_station;
	origin_station_t_y = (origin_station_t_y * (data_count_station - 1) + station_t_y) / data_count_station;
	origin_station_t_z = (origin_station_t_z * (data_count_station - 1) + station_t_z) / data_count_station;				
 
	if (has_data_station == false){
		if (differential_mode == true){	
			if (data_count_station >= data_count_required_station){
				has_data_station = true;			
				ROS_WARN("[base_imu] started: station UTM origin x=%.3f, y=%.3f, z=%.3f", origin_station_t_x, origin_station_t_y, origin_station_t_z);
			} 
		}

	} else {
		correction_t_x = _pose->pose.pose.position.x - origin_station_t_x;
		correction_t_y = _pose->pose.pose.position.y - origin_station_t_y;
		correction_t_z = _pose->pose.pose.position.z - origin_station_t_z;		
	}	
}