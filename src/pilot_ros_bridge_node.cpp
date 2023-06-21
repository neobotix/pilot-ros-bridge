/*
 * pilot_ros_bridge_node.cpp
 *
 *  Created on: May 29, 2020
 *      Author: mad
 */

#include <pilot/ros_bridge/ROS_Bridge.h>

#include <vnx/Proxy.h>
#include <vnx/Config.h>
#include <vnx/Process.h>
#include <vnx/Terminal.h>

#include <ros/ros.h>


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "pilot_ros_bridge_node");

	// initialize VNX
	vnx::init("pilot_ros_bridge_node", 0, nullptr);

	std::string pilot_node;
	std::string pilot_config;
	{
		ros::NodeHandle nh_private("~");
		nh_private.param<std::string>("pilot_node", pilot_node, "localhost:5555");
		nh_private.param<std::string>("pilot_config", pilot_config, "config/default/generic/");
	}
	vnx::read_config_tree(pilot_config);

	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("pilot_ros_bridge_node");

	{
		vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
		module.start_detached();
	}

	vnx::Handle<vnx::Proxy> proxy = new vnx::Proxy("Proxy", vnx::Endpoint::from_url(pilot_node));
	proxy->time_sync = true;
	{
		vnx::Handle<pilot::ros_bridge::ROS_Bridge> module = new pilot::ros_bridge::ROS_Bridge("ROS_Bridge", nh);
		for(auto topic : module->export_tf) {
			proxy->import_list.push_back(topic->get_name());
		}
		for(const auto& entry : module->export_map) {
			proxy->import_list.push_back(entry.first->get_name());
		}
		for(const auto& entry : module->import_map) {
			proxy->export_list.push_back(entry.second->get_name());
		}
		proxy->forward_list.push_back(module->platform_interface_server);
		module.start_detached();
	}
	proxy.start_detached();

	ros::spin();		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}

