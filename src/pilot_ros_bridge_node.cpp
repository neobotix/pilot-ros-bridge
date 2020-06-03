/*
 * pilot_ros_bridge_node.cpp
 *
 *  Created on: May 29, 2020
 *      Author: mad
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>

#include <vnx/Proxy.h>
#include <vnx/Config.h>
#include <vnx/Process.h>
#include <vnx/Terminal.h>
#include <pilot/ros/BridgeBase.hxx>

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
		for(const auto& entry : export_map) {
			subscribe(entry.first, max_queue_ms_vnx);
			publishers.emplace(entry.first, nh.advertise<sensor_msgs::LaserScan>(entry.second, max_publish_queue_ros));
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

private:
	template<typename T>
	void export_publish(boost::shared_ptr<T> sample, vnx::TopicPtr src_topic)
	{
		const auto range = publishers.equal_range(src_topic);
		for(auto iter = range.first; iter != range.second; ++iter) {
			iter->second.publish(sample);
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

	std::multimap<vnx::TopicPtr, ros::Publisher> publishers;

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
		module.start_detached();
	}
	proxy.start_detached();

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}

