#include <pilot/ros_bridge/ROS_BridgeBase.hxx>

#include <pilot/PlatformInterfaceClient.hxx>

#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <neo_srvs2/srv/relay_board_set_lcd_msg.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>


namespace pilot{
namespace ros_bridge{


class ROS_Bridge : public ROS_BridgeBase {
public:
	ROS_Bridge(const std::string& _vnx_name, std::shared_ptr<rclcpp::Node> node_handle);

protected:
	void main() override;

	void handle_resend(std::shared_ptr<const vnx::Sample> value) override;
	void handle(std::shared_ptr<const automy::basic::Transform3D> value) override;

	void handle(std::shared_ptr<const LaserScan> value) override;
	void handle(std::shared_ptr<const Pose2D> value) override;
	void handle(std::shared_ptr<const PoseArray2D> value) override;
	void handle(std::shared_ptr<const Odometry> value) override;
	void handle(std::shared_ptr<const Path2D> value) override;
	void handle(std::shared_ptr<const SystemState> value) override;
	void handle(std::shared_ptr<const BatteryState> value) override;
	void handle(std::shared_ptr<const PowerState> value) override;
	void handle(std::shared_ptr<const EmergencyState> value) override;
	void handle(std::shared_ptr<const RelayBoardData> value) override;
	void handle(std::shared_ptr<const IOBoardData> value) override;
	void handle(std::shared_ptr<const USBoardData> value) override;
	void handle(std::shared_ptr<const CostMapData> value) override;
	void handle(std::shared_ptr<const OccupancyMapData> value) override;
	void handle(std::shared_ptr<const RoadMapData> value) override;
	void handle(std::shared_ptr<const kinematics::bicycle::DriveState> value) override;
	void handle(std::shared_ptr<const kinematics::differential::DriveState> value) override;
	void handle(std::shared_ptr<const kinematics::mecanum::DriveState> value) override;
	void handle(std::shared_ptr<const kinematics::omnidrive::DriveState> value) override;

	void handle_twist(std::shared_ptr<const geometry_msgs::msg::Twist> twist, const std::string& topic_name);
	void handle_pose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose, const std::string& topic_name);
	void handle_pose_cov(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> pose, const std::string& topic_name);

private:
	bool service_set_relay(std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Response> res);
	bool service_start_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool service_stop_charging(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool service_set_LCD_msg(std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Request> req, std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Response> res);

	void import_publish(std::shared_ptr<vnx::Value> sample, const std::string& ros_topic);
	template<typename T>
	void export_publish(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const std::string& sub_topic);
	template<typename T>
	void export_publish(std::shared_ptr<T> sample, const std::string& sub_topic = std::string());

	std::shared_ptr<rclcpp::Node> nh;
	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

	std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> import_subscribers;
	std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> export_publishers;
	std::multimap<std::string, vnx::TopicPtr> import_topic_map;
	std::multimap<vnx::TopicPtr, std::string> export_topic_map;

	std::shared_ptr<const BatteryState> battery_state;
	std::shared_ptr<const PowerState> power_state;
	std::shared_ptr<const SystemState> system_state;

	rclcpp::Service<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr srv_SetRelay;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_StartCharging;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_StopCharging;
	rclcpp::Service<neo_srvs2::srv::RelayBoardSetLCDMsg>::SharedPtr srv_SetLCDMsg;

	pilot::PlatformInterfaceClient platform_interface;
};


}
}


