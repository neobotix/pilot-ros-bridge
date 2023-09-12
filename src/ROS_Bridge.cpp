#include <pilot/ros_bridge/ROS_Bridge.h>

#include <pilot/VelocityCmd.hxx>

#include <neo_msgs/IOBoard.h>
#include <neo_msgs/USBoardV2.h>
#include <neo_msgs/RelayBoardV2.h>
#include <neo_msgs/EmergencyStopState.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace pilot{
namespace ros_bridge{


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

template<typename T>
geometry_msgs::Quaternion matrix_to_orientation(const automy::math::Matrix<T, 3, 3> &mat){
	tf::Quaternion q;
	const auto ros_matrix = pilot_to_ros_matrix_33(mat);
	ros_matrix.getRotation(q);

	geometry_msgs::Quaternion result;
	tf::quaternionTFToMsg(q, result);
	return result;
}

geometry_msgs::Quaternion orientation_from_yaw(double yaw){
	geometry_msgs::Quaternion result;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), result);
	return result;
}


ROS_Bridge::ROS_Bridge(const std::string& _vnx_name, std::shared_ptr<ros::NodeHandle> node_handle)
		:	ROS_BridgeBase(_vnx_name), platform_interface(platform_interface_server)
{
	nh = node_handle;
}


void ROS_Bridge::main(){
	for(auto topic : export_tf) {
		subscribe(topic, max_queue_ms_vnx);
	}

	for(const auto& entry : import_map){
		const auto& ros_topic = entry.first.first;
		const auto& ros_type = entry.first.second;
		const auto& pilot_topic = entry.second;

		log(INFO) << "Importing '" << ros_topic << "' type '" << ros_type << "' as '" << pilot_topic->get_name() << "'";

		if(!import_subscribers.count(ros_topic)){
			ros::Subscriber subs;
			if(ros_type == "geometry_msgs/Twist") {
				subs = nh->subscribe<geometry_msgs::Twist>(ros_topic, max_subscribe_queue_ros, boost::bind(&ROS_Bridge::handle_twist, this, _1, ros_topic));
			} else if(ros_type == "geometry_msgs/PoseStamped") {
				subs = nh->subscribe<geometry_msgs::PoseStamped>(ros_topic, max_subscribe_queue_ros, boost::bind(&ROS_Bridge::handle_pose, this, _1, ros_topic));
			} else if(ros_type == "geometry_msgs/PoseWithCovarianceStamped") {
				subs = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(ros_topic, max_subscribe_queue_ros, boost::bind(&ROS_Bridge::handle_pose_cov, this, _1, ros_topic));
			} else {
				log(WARN) << "Unsupported ROS type: " << ros_type;
				continue;
			}
			import_subscribers[ros_topic] = subs;
		}
		import_topic_map.emplace(ros_topic, pilot_topic);
	}

	for(const auto &topic : pilot_topics){
		subscribe(topic, max_queue_ms_vnx);
	}

	for(const auto& entry : export_map){
		const auto& ros_topic = entry.second;
		const auto& pilot_topic = entry.first;

		subscribe(pilot_topic, max_queue_ms_vnx);
		export_topic_map.emplace(pilot_topic, ros_topic);

		log(INFO) << "Exporting '" << pilot_topic->get_name() << "' as '" << ros_topic << "'";
	}

	srv_SetRelay = nh->advertiseService("set_relay", &ROS_Bridge::service_set_relay, this);
	srv_StartCharging = nh->advertiseService("start_charging", &ROS_Bridge::service_start_charging, this);
	srv_StopCharging = nh->advertiseService("stop_charging", &ROS_Bridge::service_stop_charging, this);
	srv_SetLCDMsg = nh->advertiseService("set_LCD_msg", &ROS_Bridge::service_set_LCD_msg, this);

	Super::main();

	ros::shutdown();
}


void ROS_Bridge::handle_resend(std::shared_ptr<const vnx::Sample> value){
	Super::handle(value);
}


void ROS_Bridge::handle(std::shared_ptr<const automy::basic::Transform3D> value){
	geometry_msgs::TransformStamped out;
	out.header.stamp = pilot_to_ros_time(value->time);
	out.header.frame_id = value->parent;
	out.child_frame_id = value->frame;
	out.transform.translation.x = value->matrix(0, 3);
	out.transform.translation.y = value->matrix(1, 3);
	out.transform.translation.z = value->matrix(2, 3);
	out.transform.rotation = matrix_to_orientation(value->matrix.get<3, 3>());
	broadcaster.sendTransform(out);
}


void ROS_Bridge::handle(std::shared_ptr<const LaserScan> value){
	auto out = boost::make_shared<sensor_msgs::LaserScan>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->header.frame_id = value->frame;
	out->angle_min = value->field.min_angle;
	out->angle_max = value->field.max_angle;
	out->range_min = value->field.min_range;
	out->range_max = value->field.max_range;
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


void ROS_Bridge::handle(std::shared_ptr<const Pose2D> value){
	if(std::count(export_tf.begin(), export_tf.end(), vnx_sample->topic)) {
		handle(std::shared_ptr<const automy::basic::Transform3D>(value));
	}

	auto out = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->header.frame_id = value->parent;
	out->pose.pose.position.x = value->pose.x();
	out->pose.pose.position.y = value->pose.y();
	out->pose.pose.orientation = orientation_from_yaw(value->pose.z());
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const PoseArray2D> value){
	auto out = boost::make_shared<geometry_msgs::PoseArray>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->header.frame_id = value->frame;
	for(const auto& pose : value->poses) {
		geometry_msgs::Pose tmp;
		tmp.position.x = pose.x();
		tmp.position.y = pose.y();
		tmp.orientation = orientation_from_yaw(pose.z());
		out->poses.push_back(tmp);
	}
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const Odometry> value){
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
	out->pose.pose.orientation = matrix_to_orientation(value->matrix.get<3, 3>());
	out->twist.twist.linear.x = value->linear_velocity[0];
	out->twist.twist.linear.y = value->linear_velocity[1];
	out->twist.twist.linear.z = value->linear_velocity[2];
	out->twist.twist.angular.x = value->angular_velocity[0];
	out->twist.twist.angular.y = value->angular_velocity[1];
	out->twist.twist.angular.z = value->angular_velocity[2];
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const Path2D> value){
	auto out = boost::make_shared<nav_msgs::Path>();
	out->header.frame_id = value->frame;
	out->header.stamp = pilot_to_ros_time(value->time);
	for(auto point : value->points) {
		geometry_msgs::PoseStamped tmp;
		tmp.header = out->header;
		tmp.pose.position.x = point->pose.x();
		tmp.pose.position.y = point->pose.y();
		tmp.pose.orientation = orientation_from_yaw(point->pose.z());
		out->poses.push_back(tmp);
	}
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const SystemState> value){
	system_state = value;
}


void ROS_Bridge::handle(std::shared_ptr<const BatteryState> value){
	battery_state = value;

	auto out = boost::make_shared<sensor_msgs::BatteryState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	out->voltage = value->voltage;
	out->current = value->current;
	out->charge = NAN;
	out->capacity = NAN;
	out->design_capacity = NAN;
	out->percentage = value->remaining;
	out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	if(power_state){
		const auto &chrg = power_state->charging_state;
		if(chrg == charging_state_e::IS_CHARGING){
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		}else if(chrg == charging_state_e::FINISHED){
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
		}else{
			out->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	out->power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
	out->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	if(value->type == battery_type_e::AGM){
		// lead
	}else if(value->type == battery_type_e::LFP){
		out->power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
	}
	out->present = true;

	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const PowerState> value){
	power_state = value;
}


void ROS_Bridge::handle(std::shared_ptr<const EmergencyState> value){
	auto out = boost::make_shared<neo_msgs::EmergencyStopState>();
	out->header.stamp = pilot_to_ros_time(value->time);

	// assign input (laser, button) specific EM state
	out->emergency_button_stop = false;
	out->scanner_stop = false;

	// Scanner stop or EMStop
	switch (value->code) {
		case safety_code_e::SCANNER_STOP: out->scanner_stop = true; break;
		case safety_code_e::EMERGENCY_STOP: out->emergency_button_stop = true; break;
	}

	// State of the EMStop
	switch (value->state) {
		case em_stop_state_e::FREE: out->emergency_state = 0; break;
		case em_stop_state_e::STOPPED: out->emergency_state = 1; break;
		case em_stop_state_e::CONFIRMED: out->emergency_state = 2; break;
	}

	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const RelayBoardData> value){
	auto out = boost::make_shared<neo_msgs::RelayBoardV2>();

	out->header.stamp = pilot_to_ros_time(value->time);


	out->temperature = value->ambient_temperature;

	for(int i = 0; i < value->relay_states.size(); ++i) {
		out->relay_states[i] = value->relay_states[i];
	}

	out->keypad[0] = value->keypad_state.info_button;
	out->keypad[1] = value->keypad_state.home_button;
	out->keypad[2] = value->keypad_state.start_button;
	out->keypad[3] = value->keypad_state.stop_button;
	out->keypad[4] = value->keypad_state.brake_release_button;
	out->keypad[5] = value->keypad_state.digital_input[0];
	out->keypad[6] = value->keypad_state.digital_input[1];
	out->keypad[7] = value->keypad_state.digital_input[2];


	if(battery_state){
		out->battery_voltage = battery_state->voltage;
		out->charging_current = battery_state->current;
	}

	if(power_state){
		switch(power_state->charging_state) {
		case charging_state_e::NOT_CHARGING: out->charging_state = 0; break;
		case charging_state_e::IS_CHARGING: out->charging_state = 1; break;
		case charging_state_e::NO_CHARGER: out->charging_state = 2; break;
		case charging_state_e::BRAKES_OPEN: out->charging_state = 3; break;
		case charging_state_e::EM_STOP: out->charging_state = 3; break;
		case charging_state_e::ABORTED: out->charging_state = 4; break;
		case charging_state_e::FINISHED: out->charging_state = 5; break;
		}
	}

	out->relayboardv2_state = {false};
	if(system_state){
		for (auto it : system_state->system_errors) {
			switch(it)
			{
				case system_error_e::CHARGING_RELAY_ERROR: out->relayboardv2_state[0] = true; break;
				case system_error_e::BRAKE_RELEASE_BUTTON_ERROR: out->relayboardv2_state[1] = true; break;
				case system_error_e::MOTOR_ERROR: out->relayboardv2_state[2] = true; break;
				case system_error_e::SAFETY_RELAY_ERROR: out->relayboardv2_state[3] = true; break;
				case system_error_e::POWER_RELAY_ERROR: out->relayboardv2_state[4] = true; break;
				case system_error_e::EM_STOP_SYSTEM_ERROR: out->relayboardv2_state[5] = true; break;
			}
		}

		// Shutdown
		out->shutdown = system_state->is_shutdown; // relayboard is powering of in < 30s
	}

	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const IOBoardData> value){
	auto out = boost::make_shared<neo_msgs::IOBoard>();

	// Assigning digital inputs and outputs
	for (int iIOCnt = 0; iIOCnt < 16; iIOCnt++)
	{
		out->digital_inputs[iIOCnt] = value->digital_input[iIOCnt];
		out->digital_outputs[iIOCnt] = value->digital_output[iIOCnt];
	}

	// Assigning analog inputs
	for (int i = 0; i < 8; i++)
	{
		out->analog_inputs[i] = value->analog_input[i];
	}

	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const USBoardData> value){
	auto out = boost::make_shared<neo_msgs::USBoardV2>();
	out->header.stamp = pilot_to_ros_time(value->time);

	for (int i = 0; i < 16; i++)
	{
		out->sensor[i] = value->sensor[i];
	}

	for (int i = 0; i < 4; i++)
	{
		out->analog[i] = value->analog_input[i];
	}

	// Publish raw data in neo_msgs::USBoardV2 format
	export_publish(out);

	// TODO: Additionally publish data in ROS sensor_msgs::Range format
}


void ROS_Bridge::handle(std::shared_ptr<const CostMapData> value){
	auto out = boost::make_shared<nav_msgs::OccupancyGrid>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->header.frame_id = value->frame;
	out->info.resolution = value->scale;
	out->info.width = value->cost.width();
	out->info.height = value->cost.height();
	out->info.origin.position.x = value->origin.x();
	out->info.origin.position.y = value->origin.y();
	out->info.origin.orientation = orientation_from_yaw(value->orientation);
	out->data.resize(value->cost.get_size());
	for(size_t i = 0; i < out->data.size(); ++i) {
		const auto pix = value->cost[i];
		if(pix <= 200) {
			out->data[i] = pix / 2;
		} else {
			out->data[i] = 0;
		}
	}
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const OccupancyMapData> value){
	auto out = boost::make_shared<nav_msgs::OccupancyGrid>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->header.frame_id = value->frame;
	out->info.resolution = value->scale;
	out->info.width = value->occupancy.width();
	out->info.height = value->occupancy.height();
	out->info.origin.position.x = value->origin.x();
	out->info.origin.position.y = value->origin.y();
	out->info.origin.orientation = orientation_from_yaw(value->orientation);
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


void ROS_Bridge::handle(std::shared_ptr<const RoadMapData> value){
	auto nodes = boost::make_shared<visualization_msgs::Marker>();
	nodes->header.frame_id = map_frame;
	nodes->ns = vnx_sample->topic->get_name() + ".nodes";
	nodes->type = visualization_msgs::Marker::SPHERE_LIST;
	nodes->scale.x = 0.1; nodes->scale.y = 0.1; nodes->scale.z = 0.1;
	nodes->color.r = 0; nodes->color.g = 0; nodes->color.b = 1; nodes->color.a = 1;

	auto segments = boost::make_shared<visualization_msgs::Marker>();
	segments->header.frame_id = map_frame;
	segments->ns = vnx_sample->topic->get_name() + ".segments";
	segments->type = visualization_msgs::Marker::LINE_LIST;
	segments->scale.x = 0.03;
	segments->color.r = 1; segments->color.g = 0; segments->color.b = 1; segments->color.a = 0.3;

	auto stations = boost::make_shared<geometry_msgs::PoseArray>();
	stations->header.frame_id = map_frame;
	auto markers = boost::make_shared<visualization_msgs::MarkerArray>();

	std::map<int, std::shared_ptr<const MapNode>> node_map;

	for(auto node : value->nodes) {
		auto station = std::dynamic_pointer_cast<const MapStation>(node);
		if(station) {
			geometry_msgs::Pose tmp;
			tmp.position.x = station->position.x();
			tmp.position.y = station->position.y();
			tmp.orientation = orientation_from_yaw(station->orientation);
			stations->poses.push_back(tmp);
			{
				visualization_msgs::Marker marker;
				marker.header.frame_id = map_frame;
				marker.ns = vnx_sample->topic->get_name() + ".stations";
				marker.id = node->id;
				marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				marker.pose = tmp;
				marker.text = station->name;
				marker.scale.z = 0.25;
				marker.color.r = 0; marker.color.g = 0; marker.color.b = 0; marker.color.a = 1;
				markers->markers.push_back(marker);
			}
		}
		geometry_msgs::Point point;
		point.x = node->position.x();
		point.y = node->position.y();
		nodes->points.push_back(point);
		node_map[node->id] = node;
	}
	for(auto segment : value->segments) {
		{
			auto node = node_map[segment->from_node];
			geometry_msgs::Point point;
			if(node) {
				point.x = node->position.x();
				point.y = node->position.y();
			}
			segments->points.push_back(point);
		}
		{
			auto node = node_map[segment->to_node];
			geometry_msgs::Point point;
			if(node) {
				point.x = node->position.x();
				point.y = node->position.y();
			}
			segments->points.push_back(point);
		}
	}
	export_publish(nodes, "nodes");
	export_publish(segments, "segments");
	export_publish(stations, "stations");
	export_publish(markers, "markers");
}


void ROS_Bridge::handle(std::shared_ptr<const kinematics::bicycle::DriveState> value){
	// TODO
}


void ROS_Bridge::handle(std::shared_ptr<const kinematics::differential::DriveState> value){
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


void ROS_Bridge::handle(std::shared_ptr<const kinematics::mecanum::DriveState> value){
	auto out = boost::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(4);
	out->position.resize(4);
	out->velocity.resize(4);
	out->name[0] = "wheel_front_left_base_link";
	out->name[1] = "wheel_front_right_base_link";
	out->name[2] = "wheel_back_left_base_link";
	out->name[3] = "wheel_back_right_base_link";
	out->position[0] = value->position.front_left;
	out->position[1] = value->position.front_right;
	out->position[2] = value->position.back_left;
	out->position[3] = value->position.back_right;
	out->velocity[0] = value->velocity.front_left;
	out->velocity[1] = value->velocity.front_right;
	out->velocity[2] = value->velocity.back_left;
	out->velocity[3] = value->velocity.back_right;
	if(value->has_torque) {
		out->effort.resize(4);
		out->effort[0] = value->torque.front_left;
		out->effort[1] = value->torque.front_right;
		out->effort[2] = value->torque.back_left;
		out->effort[3] = value->torque.back_right;
	}
	export_publish(out);
}


void ROS_Bridge::handle(std::shared_ptr<const kinematics::omnidrive::DriveState> value){
	auto out = boost::make_shared<sensor_msgs::JointState>();
	out->header.stamp = pilot_to_ros_time(value->time);
	out->name.resize(8);
	out->position.resize(8);
	out->velocity.resize(8);
	out->name[0] = "mpo_700_wheel_front_left_joint";
	out->name[1] = "mpo_700_caster_front_left_joint";
	out->name[2] = "mpo_700_wheel_back_left_joint";
	out->name[3] = "mpo_700_caster_back_left_joint";
	out->name[4] = "mpo_700_wheel_back_right_joint";
	out->name[5] = "mpo_700_caster_back_right_joint";
	out->name[6] = "mpo_700_wheel_front_right_joint";
	out->name[7] = "mpo_700_caster_front_right_joint";
	out->position[0] = value->drive_pos.get(kinematics::position_code_e::FRONT_LEFT);
	out->position[1] = value->steer_pos.get(kinematics::position_code_e::FRONT_LEFT);
	out->position[2] = value->drive_pos.get(kinematics::position_code_e::BACK_LEFT);
	out->position[3] = value->steer_pos.get(kinematics::position_code_e::BACK_LEFT);
	out->position[4] = value->drive_pos.get(kinematics::position_code_e::BACK_RIGHT);
	out->position[5] = value->steer_pos.get(kinematics::position_code_e::BACK_RIGHT);
	out->position[6] = value->drive_pos.get(kinematics::position_code_e::FRONT_RIGHT);
	out->position[7] = value->steer_pos.get(kinematics::position_code_e::FRONT_RIGHT);
	out->velocity[0] = value->drive_vel.get(kinematics::position_code_e::FRONT_LEFT);
	out->velocity[1] = value->steer_vel.get(kinematics::position_code_e::FRONT_LEFT);
	out->velocity[2] = value->drive_vel.get(kinematics::position_code_e::BACK_LEFT);
	out->velocity[3] = value->steer_vel.get(kinematics::position_code_e::BACK_LEFT);
	out->velocity[4] = value->drive_vel.get(kinematics::position_code_e::BACK_RIGHT);
	out->velocity[5] = value->steer_vel.get(kinematics::position_code_e::BACK_RIGHT);
	out->velocity[6] = value->drive_vel.get(kinematics::position_code_e::FRONT_RIGHT);
	out->velocity[7] = value->steer_vel.get(kinematics::position_code_e::FRONT_RIGHT);
	if(value->has_torque) {
		out->effort.resize(8);
		out->effort[0] = value->drive_torque.get(kinematics::position_code_e::FRONT_LEFT);
		out->effort[1] = value->steer_torque.get(kinematics::position_code_e::FRONT_LEFT);
		out->effort[2] = value->drive_torque.get(kinematics::position_code_e::BACK_LEFT);
		out->effort[3] = value->steer_torque.get(kinematics::position_code_e::BACK_LEFT);
		out->effort[4] = value->drive_torque.get(kinematics::position_code_e::BACK_RIGHT);
		out->effort[5] = value->steer_torque.get(kinematics::position_code_e::BACK_RIGHT);
		out->effort[6] = value->drive_torque.get(kinematics::position_code_e::FRONT_RIGHT);
		out->effort[7] = value->steer_torque.get(kinematics::position_code_e::FRONT_RIGHT);
	}
	export_publish(out);
}


void ROS_Bridge::handle_twist(const geometry_msgs::Twist::ConstPtr& twist, const std::string& topic_name){
	auto out = VelocityCmd::create();
	out->time = vnx::get_time_micros();
	out->linear.x() = twist->linear.x;
	out->linear.y() = twist->linear.y;
	out->linear.z() = twist->linear.z;
	out->angular.x() = twist->angular.x;
	out->angular.y() = twist->angular.y;
	out->angular.z() = twist->angular.z;
	import_publish(out, topic_name);
}


void ROS_Bridge::handle_pose(const geometry_msgs::PoseStamped::ConstPtr& pose, const std::string& topic_name){
	auto out = Pose2D::create();
	out->time = vnx::get_time_micros();
	out->frame = base_frame;
	out->parent = pose->header.frame_id;
	out->pose.x() = pose->pose.position.x;
	out->pose.y() = pose->pose.position.y;
	out->pose.z() = tf::getYaw(pose->pose.orientation);
	out->update();
	import_publish(out, topic_name);
}


void ROS_Bridge::handle_pose_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const std::string& topic_name){
	auto out = Pose2D::create();
	out->time = vnx::get_time_micros();
	out->frame = base_frame;
	out->parent = pose->header.frame_id;
	out->pose.x() = pose->pose.pose.position.x;
	out->pose.y() = pose->pose.pose.position.y;
	out->pose.z() = tf::getYaw(pose->pose.pose.orientation);
	out->update();
	import_publish(out, topic_name);
}


bool ROS_Bridge::service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res){
	try {
		platform_interface.set_relay(req.id, req.state);
	}catch(const std::exception& ex){
		const std::string error = "Service call failed with: " + std::string(ex.what());
		ROS_ERROR_STREAM(error);
		res.success = false;
		return false;
	}
	res.success = true;
	return true;
}


bool ROS_Bridge::service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	try {
		platform_interface.start_charging();
	}catch(const std::exception& ex){
		const std::string error = "Service call failed with: " + std::string(ex.what());
		ROS_ERROR_STREAM(error);
		return false;
	}
	return true;
}


bool ROS_Bridge::service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	try {
		platform_interface.stop_charging();
	}catch(const std::exception& ex){
		const std::string error = "Service call failed with: " + std::string(ex.what());
		ROS_ERROR_STREAM(error);
		return false;
	}
	return true;
}


bool ROS_Bridge::service_set_LCD_msg(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res){
	try {
		platform_interface.set_display_text(req.message);
	}catch(std::exception& ex){
		const std::string error = "Service call failed with: " + std::string(ex.what());
		ROS_ERROR_STREAM(error);
		res.success = false;
		return false;
	}
	res.success = true;
	return true;
}


void ROS_Bridge::import_publish(std::shared_ptr<vnx::Value> sample, const std::string& ros_topic){
	const auto range = import_topic_map.equal_range(ros_topic);
	for(auto entry = range.first; entry != range.second; ++entry) {
		publish(sample, entry->second);
	}
}


template<typename T>
void ROS_Bridge::export_publish(boost::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const std::string& sub_topic){
	const auto range = export_topic_map.equal_range(pilot_topic);
	for(auto entry = range.first; entry != range.second; ++entry){
		const std::string ros_topic = entry->second + (sub_topic.empty() ? "" : "/") + sub_topic;
		if(!export_publishers.count(ros_topic)) {
			export_publishers[ros_topic] = nh->advertise<T>(ros_topic, max_publish_queue_ros);
		}
		export_publishers[ros_topic].publish(sample);
	}
}


template<typename T>
void ROS_Bridge::export_publish(boost::shared_ptr<T> sample, const std::string& sub_topic){
	if(!vnx_sample) {
		throw std::logic_error("!vnx_sample");
	}
	export_publish(sample, vnx_sample->topic, sub_topic);
}



}
}



