#include <pilot/ros_bridge/ROS_BridgeBase.hxx>

#include <pilot/PlatformInterfaceClient.hxx>

#include <neo_srvs/RelayBoardSetRelay.h>
#include <neo_srvs/RelayBoardSetLCDMsg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


namespace pilot{
namespace ros_bridge{


class ROS_Bridge : public ROS_BridgeBase {
public:
	ROS_Bridge(const std::string& _vnx_name, std::shared_ptr<ros::NodeHandle> node_handle);

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

	void handle_twist(const geometry_msgs::Twist::ConstPtr& twist, const std::string& topic_name);
	void handle_pose(const geometry_msgs::PoseStamped::ConstPtr& pose, const std::string& topic_name);
	void handle_pose_cov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const std::string& topic_name);

private:
	bool service_set_relay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res);
	bool service_start_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool service_stop_charging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool service_set_LCD_msg(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res);

	void import_publish(std::shared_ptr<vnx::Value> sample, const std::string& ros_topic);
	template<typename T>
	void export_publish(boost::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const std::string& sub_topic);
	template<typename T>
	void export_publish(boost::shared_ptr<T> sample, const std::string& sub_topic = std::string());

	std::shared_ptr<ros::NodeHandle> nh;
	tf::TransformBroadcaster broadcaster;

	std::map<std::string, ros::Subscriber> import_subscribers;
	std::map<std::string, ros::Publisher> export_publishers;
	std::multimap<std::string, vnx::TopicPtr> import_topic_map;
	std::multimap<vnx::TopicPtr, std::string> export_topic_map;

	std::shared_ptr<const BatteryState> battery_state;
	std::shared_ptr<const PowerState> power_state;
	std::shared_ptr<const SystemState> system_state;

	ros::ServiceServer srv_SetRelay;
	ros::ServiceServer srv_StartCharging;
	ros::ServiceServer srv_StopCharging;
	ros::ServiceServer srv_SetLCDMsg;

	pilot::PlatformInterfaceClient platform_interface;
};


}
}


