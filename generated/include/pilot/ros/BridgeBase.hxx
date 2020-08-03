
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_pilot_ros_BridgeBase_HXX_
#define INCLUDE_pilot_ros_BridgeBase_HXX_

#include <pilot/ros/package.hxx>
#include <automy/basic/Transform3D.hxx>
#include <pilot/CostMapData.hxx>
#include <pilot/LaserScan.hxx>
#include <pilot/OccupancyMapData.hxx>
#include <pilot/Odometry.hxx>
#include <pilot/Path2D.hxx>
#include <pilot/Pose2D.hxx>
#include <pilot/PoseArray2D.hxx>
#include <pilot/RoadMapData.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>
#include <vnx/Module.h>
#include <vnx/TopicPtr.hpp>


namespace pilot {
namespace ros {

class BridgeBase : public ::vnx::Module {
public:
	
	std::vector<::vnx::TopicPtr> export_tf;
	std::vector<std::pair<std::pair<std::string, std::string>, ::vnx::TopicPtr>> import_map;
	std::vector<std::pair<::vnx::TopicPtr, std::string>> export_map;
	std::string base_frame = "base_link";
	std::string odom_frame = "odom";
	std::string map_frame = "map";
	int32_t max_queue_ms_vnx = 100;
	int32_t max_publish_queue_ros = 1;
	int32_t max_subscribe_queue_ros = 1;
	
	typedef ::vnx::Module Super;
	
	static const vnx::Hash64 VNX_TYPE_HASH;
	static const vnx::Hash64 VNX_CODE_HASH;
	
	BridgeBase(const std::string& _vnx_name);
	
	vnx::Hash64 get_type_hash() const;
	const char* get_type_name() const;
	const vnx::TypeCode* get_type_code() const;
	
	void read(std::istream& _in);
	void write(std::ostream& _out) const;
	
	void accept(vnx::Visitor& _visitor) const;
	
	vnx::Object to_object() const;
	void from_object(const vnx::Object& object);
	
	friend std::ostream& operator<<(std::ostream& _out, const BridgeBase& _value);
	friend std::istream& operator>>(std::istream& _in, BridgeBase& _value);
	
	static const vnx::TypeCode* static_get_type_code();
	static std::shared_ptr<vnx::TypeCode> static_create_type_code();
	
protected:
	virtual void handle(std::shared_ptr<const ::automy::basic::Transform3D> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::automy::basic::Transform3D> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::Path2D> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::Path2D> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::Pose2D> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::Pose2D> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::PoseArray2D> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::PoseArray2D> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::Odometry> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::Odometry> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::LaserScan> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::LaserScan> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::CostMapData> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::CostMapData> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::OccupancyMapData> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::OccupancyMapData> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::RoadMapData> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::RoadMapData> _value) {}
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::differential::DriveState> _value, std::shared_ptr<const vnx::Sample> _sample) { handle(_value); }
	virtual void handle(std::shared_ptr<const ::pilot::kinematics::differential::DriveState> _value) {}
	
	void vnx_handle_switch(std::shared_ptr<const vnx::Sample> _sample) override;
	std::shared_ptr<vnx::Value> vnx_call_switch(std::shared_ptr<const vnx::Value> _method, const vnx::request_id_t& _request_id) override;
	
};


} // namespace pilot
} // namespace ros

#endif // INCLUDE_pilot_ros_BridgeBase_HXX_
