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

#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv)
{
	// initialize ROS
	rclcpp::init(argc, argv);

	// initialize VNX
	vnx::init("pilot_ros_bridge_node", 0, nullptr);


	std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("pilot_ros_bridge_node");
	nh->declare_parameter<std::string>("pilot_node", "localhost:5555");
	nh->declare_parameter<std::string>("pilot_config", "config/default/generic/");

	std::string pilot_node;
	std::string pilot_config;
	nh->get_parameter<std::string>("pilot_node", pilot_node);
	nh->get_parameter<std::string>("pilot_config", pilot_config);

	vnx::read_config_tree(pilot_config);

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

	rclcpp::spin(nh);		// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}

