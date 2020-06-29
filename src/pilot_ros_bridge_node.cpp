/*
 * pilot_ros_bridge_node.cpp
 *
 *  Created on: May 29, 2020
 *      Author: mad
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vnx/Proxy.h>
#include <vnx/Config.h>
#include <vnx/Process.h>
#include <vnx/Terminal.h>

#include <pilot/ros/BridgeBase.hxx>
#include <pilot/VelocityCmd.hxx>
#include <pilot/GridMapData.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>

#include <algorithm>


inline
ros::Time pilot_to_ros_time(const int64_t& time_usec)
{
	ros::Time time;
	time.fromNSec(time_usec * 1000);
	return time;
}

template<typename T>
tf::Matrix3x3 pilot_to_ros_matrix_33(const automy::math::Matrix<T, 3, 3>& mat)
{
	tf::Matrix3x3 res;
	for(int j = 0; j < 3; ++j) {
		for(int i = 0; i < 3; ++i) {
			res[i][j] = mat(i, j);
		}
	}
	return res;
}


class Pilot_ROS_Bridge : public pilot::ros::BridgeBase {
public:
	Pilot_ROS_Bridge(const std::string& _vnx_name)
		:	BridgeBase(_vnx_name)
	{
	}

protected:
	void main() override
	{
		for(auto topic : export_tf) {
			subscribe(topic, max_queue_ms_vnx);
		}

		for(const auto& entry : import_map)
		{
			const auto& ros_topic = entry.first.first;
			const auto& ros_type = entry.first.second;
			const auto& pilot_topic = entry.second;

			log(INFO) << "Importing '" << ros_topic << "' type '" << ros_type << "' as '" << pilot_topic->get_name() << "'";

			if(!import_subscribers.count(ros_topic))
			{
				ros::Subscriber subs;
				if(ros_type == "geometry_msgs/Twist") {
					subs = nh.subscribe<geometry_msgs::Twist>(ros_topic, max_subscribe_queue_ros, boost::bind(&Pilot_ROS_Bridge::handle_twist, this, _1, ros_topic));
				} else {
					log(ERROR) << "Unsupported ROS type: " << ros_type;
					continue;
				}
				import_subscribers[ros_topic] = subs;
			}
			import_topic_map.emplace(ros_topic, pilot_topic);
		}

		for(const auto& entry : export_map)
		{
			const auto& ros_topic = entry.second.first;
			const auto& ros_type = entry.second.second;
			const auto& pilot_topic = entry.first;

			log(INFO) << "Exporting '" << pilot_topic->get_name() << "' as '" << ros_topic << "' type '" << ros_type << "'";

			subscribe(pilot_topic, max_queue_ms_vnx);

			if(!export_publishers.count(ros_topic))
			{
				ros::Publisher pub;
				if(ros_type == "sensor_msgs/LaserScan") {
					pub = nh.advertise<sensor_msgs::LaserScan>(ros_topic, max_publish_queue_ros);
				} else if(ros_type == "sensor_msgs/JointState") {
					pub = nh.advertise<sensor_msgs::JointState>(ros_topic, max_publish_queue_ros);
				} else if(ros_type == "nav_msgs/Odometry") {
					pub = nh.advertise<nav_msgs::Odometry>(ros_topic, max_publish_queue_ros);
				} else if(ros_type == "nav_msgs/OccupancyGrid") {
					pub = nh.advertise<nav_msgs::OccupancyGrid>(ros_topic, 1);
				} else {
					log(ERROR) << "Unsupported ROS type: " << ros_type;
					continue;
				}
				export_publishers[ros_topic] = pub;
			}
			export_topic_map.emplace(pilot_topic, ros_topic);
		}

		Super::main();

		ros::shutdown();
	}

	void handle(std::shared_ptr<const automy::basic::Transform3D> value)
	{
		geometry_msgs::TransformStamped out;
		out.header.stamp = pilot_to_ros_time(value->time);
		out.header.frame_id = value->parent;
		out.child_frame_id = value->frame;
		out.transform.translation.x = value->matrix(0, 3);
		out.transform.translation.y = value->matrix(1, 3);
		out.transform.translation.z = value->matrix(2, 3);
		tf::Quaternion q;
		pilot_to_ros_matrix_33(value->matrix.get<3, 3>()).getRotation(q);
		tf::quaternionTFToMsg(q, out.transform.rotation);
		broadcaster.sendTransform(out);
	}

	void handle(std::shared_ptr<const pilot::LaserScan> value)
	{
		auto out = boost::make_shared<sensor_msgs::LaserScan>();
		out->header.stamp = pilot_to_ros_time(value->time);
		out->header.frame_id = value->frame;
		out->angle_min = value->min_angle;
		out->angle_max = value->max_angle;
		out->range_min = value->min_range;
		out->range_max = value->max_range;
		if(value->points.size() >= 2) {
			out->scan_time = (value->points.back().time_offset - value->points.front().time_offset) * 1e-6f;
			out->time_increment = (value->points[1].time_offset - value->points[0].time_offset) * 1e-6f;
			out->angle_increment = (value->points[1].angle - value->points[0].angle);
		}
		out->ranges.resize(value->points.size());
		out->intensities.resize(value->points.size());
		for(size_t i = 0; i < value->points.size(); ++i) {
			out->ranges[i] = value->points[i].distance;
			out->intensities[i] = value->points[i].intensity;
		}
		export_publish(out);
	}

	void handle(std::shared_ptr<const pilot::Odometry> value)
	{
		if(std::count(export_tf.begin(), export_tf.end(), vnx_sample->topic)) {
			handle(std::shared_ptr<const automy::basic::Transform3D>(value));
		}

		auto out = boost::make_shared<nav_msgs::Odometry>();
		out->header.stamp = pilot_to_ros_time(value->time);
		out->header.frame_id = value->parent;
		out->child_frame_id = value->frame;
		out->pose.pose.position.x = value->position[0];
		out->pose.pose.position.y = value->position[1];
		out->pose.pose.position.z = value->position[2];
		tf::Quaternion q;
		pilot_to_ros_matrix_33(value->matrix.get<3, 3>()).getRotation(q);
		tf::quaternionTFToMsg(q, out->pose.pose.orientation);
		out->twist.twist.linear.x = value->linear_velocity[0];
		out->twist.twist.linear.y = value->linear_velocity[1];
		out->twist.twist.linear.z = value->linear_velocity[2];
		out->twist.twist.angular.x = value->angular_velocity[0];
		out->twist.twist.angular.y = value->angular_velocity[1];
		out->twist.twist.angular.z = value->angular_velocity[2];
		export_publish(out);
	}

	void handle(std::shared_ptr<const pilot::GridMapData> value)
	{
		auto out = boost::make_shared<nav_msgs::OccupancyGrid>();
		out->header.stamp = pilot_to_ros_time(value->time);
		out->info.resolution = value->scale;
		out->info.width = value->occupancy.width();
		out->info.height = value->occupancy.height();
		out->info.origin.position.x = value->origin.x();
		out->info.origin.position.y = value->origin.y();
		tf::quaternionTFToMsg(tf::createQuaternionFromYaw(value->orientation), out->info.origin.orientation);
		out->data.resize(value->occupancy.get_size());
		for(size_t i = 0; i < out->data.size(); ++i) {
			const auto pix = value->occupancy[i];
			if(pix == 255) {
				out->data[i] = -1;
			} else {
				out->data[i] = pix;
			}
		}
		export_publish(out);
	}

	void handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value)
	{
		auto out = boost::make_shared<sensor_msgs::JointState>();
		out->header.stamp = pilot_to_ros_time(value->time);
		out->name.resize(2);
		out->position.resize(2);
		out->velocity.resize(2);
		out->name[0] = "wheel_front_left_joint";
		out->name[1] = "wheel_front_right_joint";
		out->position[0] = value->position.left;
		out->position[1] = value->position.right;
		out->velocity[0] = value->velocity.left;
		out->velocity[1] = value->velocity.right;
		if(value->has_torque) {
			out->effort.resize(2);
			out->effort[0] = value->torque.left;
			out->effort[1] = value->torque.right;
		}
		export_publish(out);
	}

	void handle_twist(const geometry_msgs::Twist::ConstPtr& twist, const std::string& topic_name)
	{
		auto out = pilot::VelocityCmd::create();
		out->time = vnx::get_time_micros();
		out->linear.x() = twist->linear.x;
		out->linear.y() = twist->linear.y;
		out->linear.z() = twist->linear.z;
		out->angular.x() = twist->angular.x;
		out->angular.y() = twist->angular.y;
		out->angular.z() = twist->angular.z;
		import_publish(out, topic_name);
	}

private:
	void import_publish(std::shared_ptr<vnx::Value> sample, const std::string& ros_topic)
	{
		const auto range = import_topic_map.equal_range(ros_topic);
		for(auto entry = range.first; entry != range.second; ++entry) {
			publish(sample, entry->second);
		}
	}

	template<typename T>
	void export_publish(boost::shared_ptr<T> sample, vnx::TopicPtr pilot_topic)
	{
		const auto range = export_topic_map.equal_range(pilot_topic);
		for(auto entry = range.first; entry != range.second; ++entry) {
			const auto iter = export_publishers.find(entry->second);
			if(iter != export_publishers.end()) {
				iter->second.publish(sample);
			}
		}
	}

	template<typename T>
	void export_publish(boost::shared_ptr<T> sample)
	{
		if(!vnx_sample) {
			throw std::logic_error("!vnx_sample");
		}
		export_publish(sample, vnx_sample->topic);
	}

private:
	ros::NodeHandle nh;

	tf::TransformBroadcaster broadcaster;

	std::map<std::string, ros::Subscriber> import_subscribers;
	std::multimap<std::string, vnx::TopicPtr> import_topic_map;

	std::map<std::string, ros::Publisher> export_publishers;
	std::multimap<vnx::TopicPtr, std::string> export_topic_map;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "pilot_ros_bridge_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// initialize VNX
	vnx::init("pilot_ros_bridge_node", 0, 0);

	std::string pilot_node;
	std::string pilot_config;
	nh_private.param<std::string>("pilot_node", pilot_node, ".pilot_main.sock");
	nh_private.param<std::string>("pilot_config", pilot_config, "config/default/generic/");

	vnx::read_config_tree(pilot_config);

	{
		vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
		module.start_detached();
	}

	vnx::Handle<vnx::Proxy> proxy = new vnx::Proxy("Proxy", vnx::Endpoint::from_url(pilot_node));
	{
		vnx::Handle<Pilot_ROS_Bridge> module = new Pilot_ROS_Bridge("Pilot_ROS_Bridge");
		for(auto topic : module->export_tf) {
			proxy->import_list.push_back(topic->get_name());
		}
		for(const auto& entry : module->export_map) {
			proxy->import_list.push_back(entry.first->get_name());
		}
		for(const auto& entry : module->import_map) {
			proxy->export_list.push_back(entry.second->get_name());
		}
		module.start_detached();
	}
	proxy.start_detached();

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}

