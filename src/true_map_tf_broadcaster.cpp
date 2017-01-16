#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

std::string node_name = "tilt_tf_broadcaster";

std::string frame_name_parent = "odom";
std::string frame_name_child = "true_map";

std::string topic_name_head_imu = "/base_imu/global_position/local";
std::string topic_name_station = "/station_imu/global_position/local";

double boat_utm_x = 0.0;
double boat_utm_y = 0.0;
double boat_utm_z = 0.0;

double true_map_utm_x = 0.0;
double true_map_utm_y = 0.0;
double true_map_utm_z = 0.0;

double relative_t_x = 0.0;
double relative_t_y = 0.0;
double relative_t_z = 0.0;

double true_map_relative_rotation = 0.0;

bool differential_mode = false;

bool has_boat_utm_data = false;
int has_data_count = 0;

void boat_pose_cb(const nav_msgs::Odometry::ConstPtr _boat_utm_coor);

int main(int argc, char **argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param ("~");

	nh_param.param<std::string>("parent_frame", frame_name_parent, frame_name_parent);
	nh_param.param<std::string>("child_frame", frame_name_child, frame_name_child);

	nh_param.param<std::string>("true_map_utm_x", true_map_utm_x, true_map_utm_x);
	nh_param.param<std::string>("true_map_utm_y", true_map_utm_y, true_map_utm_y);
	nh_param.param<std::string>("true_map_utm_z", true_map_utm_z, true_map_utm_z);

	nh_param.param<std::string>("true_map_relative_rotation", true_map_relative_rotation, true_map_relative_rotation);

	ros::Subscriber boat_utm_coor_sub = nh.subscribe<nav_msgs::Odometry>
		(topic_name_head_imu, 10, boat_pose_cb);

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
		if (has_boat_utm_data){
			relative_t_x = 	
		} else {

		}
		q.setRPY(0.0, 0.0, yaw);
			transform.setOrigin( tf::Vector3(t_x_corrected, t_y_corrected, 0.0) );	
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, frame_name_child));	
			
			// transform.setOrigin( tf::Vector3(t_x, t_y, t_z) );	
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, "/imu_actual"));
				
			// transform.setOrigin( tf::Vector3(correction_t_x, correction_t_y, correction_t_z) );	
			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name_parent, "/station_correction"));
		ros::spinOnce();
		loop_rate.sleep();
	}	

	return 0;
}

void boat_pose_cb(const nav_msgs::Odometry::ConstPtr _boat_utm_coor){
	if (has_data_count < 30){
		double x, y, z, w;
		double m_roll, m_pitch, m_yaw;

		x = _boat_utm_coor->pose.pose.orientation.x;
		y = _boat_utm_coor->pose.pose.orientation.y;
		z = _boat_utm_coor->pose.pose.orientation.z;
		w = _boat_utm_coor->pose.pose.orientation.w;

		tf::Quaternion q = tf::Quaternion(x, y, z, w);
		tf::Matrix3x3 m(q);
		m.getRPY(m_roll, m_pitch, m_yaw);

		boat_utm_x = _boat_utm_coor->pose.pose.position.x;
		boat_utm_y = _boat_utm_coor->pose.pose.position.y;
		boat_utm_z = _boat_utm_coor->pose.pose.position.z;

		true_map_utm_x = boat_utm_x;
		true_map_utm_y = boat_utm_y;
		true_map_utm_z = boat_utm_z;	
	} else {
		has_boat_utm_data = true;
	}
	has_data_count += 1;
}