

#include <any>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// ROS message includes
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "neo_msgs2/msg/emergency_stop_state.hpp"
#include "neo_msgs2/msg/us_board.hpp"
#include "neo_msgs2/msg/relay_board_v2.hpp"
#include "neo_msgs2/msg/io_board.hpp"
#include "neo_msgs2/msg/us_board_v2.hpp"

// ROS service includes
#include <neo_srvs2/srv/io_board_set_dig_out.hpp>
#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <neo_srvs2/srv/relay_board_set_lcd_msg.hpp>
#include <neo_srvs2/srv/relay_board_set_em_stop.hpp>
#include <neo_srvs2/srv/relay_board_un_set_em_stop.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

// Visualization msgs
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <pilot/PlatformInterfaceClient.hxx>

#include <vnx/Proxy.h>
#include <vnx/Config.h>
#include <vnx/Process.h>
#include <vnx/Terminal.h>

#include <pilot/ros_bridge/BridgeBase.hxx>
#include <pilot/Pose2D.hxx>
#include <pilot/VelocityCmd.hxx>
#include <pilot/GridMapData.hxx>
#include <pilot/RoadMapData.hxx>
#include <pilot/SystemState.hxx>
#include <pilot/MapStation.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>

#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

inline
rclcpp::Time pilot_to_ros_time(const int64_t& time_usec)
{
 rclcpp::Time time(time_usec);
 return time;
}

template<typename T>
tf2::Matrix3x3 pilot_to_ros_matrix_33(const automy::math::Matrix<T, 3, 3>& mat)
{
 tf2::Matrix3x3 res;
 for(int j = 0; j < 3; ++j) {
   for(int i = 0; i < 3; ++i) {
     res[i][j] = mat(i, j);
   }
 }
 return res;
}


class Pilot_ROS_Bridge : public rclcpp::Node, public pilot::ros_bridge::BridgeBase {
public:
  Pilot_ROS_Bridge(const std::string& _vnx_name)
    : rclcpp::Node::Node(vnx::get_process_name()), BridgeBase(_vnx_name), platform_interface("PlatformInterface")
  {}

protected:
  void main() override
  {
    for(auto topic : export_tf) {
      subscribe(topic, max_queue_ms_vnx);
    }

   for(const auto& entry : import_map) {
     const auto& ros_topic = entry.first.first;
     const auto& ros_type = entry.first.second;
     const auto& pilot_topic = entry.second;

     log(INFO) << "Importing '" << ros_topic << "' type '" << ros_type << "' as '" << pilot_topic->get_name() << "'";

     if(!import_subscribers.count(ros_topic))
     {
       if(ros_type == "geometry_msgs/Twist") {
        std::function<void(const geometry_msgs::msg::Twist::SharedPtr msg)> cb = 
          std::bind(&Pilot_ROS_Bridge::handle_twist, this, _1, ros_topic);
        auto subs = this->create_subscription<geometry_msgs::msg::Twist>(
          ros_topic, 1,
          cb);
        import_subscribers[ros_topic] = subs;
       } else if(ros_type == "geometry_msgs/PoseStamped") {
         std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr msg)> cb = 
          std::bind(&Pilot_ROS_Bridge::handle_pose, this, _1, ros_topic);
         auto subs = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          ros_topic, 1,
          cb);
        import_subscribers[ros_topic] = subs;
       } else if(ros_type == "geometry_msgs/PoseWithCovarianceStamped") {
         std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)> cb = 
          std::bind(&Pilot_ROS_Bridge::handle_pose_cov, this, _1, ros_topic);
         auto subs = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          ros_topic, 1,
          cb);
        import_subscribers[ros_topic] = subs;
       } else {
         log(ERROR) << "Unsupported ROS type: " << ros_type;
         continue;
       }
     }
     import_topic_map.emplace(ros_topic, pilot_topic);
   }

   for(const auto& entry : export_map)
   {
     const auto& ros_topic = entry.second;
     const auto& pilot_topic = entry.first;

     subscribe(pilot_topic, max_queue_ms_vnx);
     export_topic_map.emplace(pilot_topic, ros_topic);

     log(INFO) << "Exporting '" << pilot_topic->get_name() << "' as '" << ros_topic << "'";
   }

   Super::main();

   rclcpp::shutdown();
 }

 void handle_resend(std::shared_ptr<const vnx::Sample> value) override
 {
   Super::handle(value);
 }

 // void handle(std::shared_ptr<const automy::basic::Transform3D> value) override
 // {
 //   geometry_msgs::msg::TransformStamped out;
 //   out.header.stamp = pilot_to_ros_time(value->time);
 //   out.header.frame_id = value->parent;
 //   out.child_frame_id = value->frame;
 //   out.transform.translation.x = value->matrix(0, 3);
 //   out.transform.translation.y = value->matrix(1, 3);
 //   out.transform.translation.z = value->matrix(2, 3);
 //   tf2::Quaternion q;
 //   q.setRPY(0, 0, pilot_to_ros_matrix_33(value->matrix.get<3, 3>()));
 //   out.transform.rotation = tf2::toMsg(q);
 //   broadcaster.sendTransform(out);
 // }

 // void handle(std::shared_ptr<const pilot::LaserScan> value) override
 // {
 //   auto out = std::make_shared<sensor_msgs::msg::LaserScan>();
 //   out->header.stamp = pilot_to_ros_time(value->time);
 //   out->header.frame_id = value->frame;
 //   out->angle_min = value->field.min_angle;
 //   out->angle_max = value->field.max_angle;
 //   out->range_min = value->field.min_range;
 //   out->range_max = value->field.max_range;
 //   if(value->points.size() >= 2) {
 //     out->scan_time = (value->points.back().time_offset - value->points.front().time_offset) * 1e-6f;
 //     out->time_increment = (value->points[1].time_offset - value->points[0].time_offset) * 1e-6f;
 //     out->angle_increment = (value->points[1].angle - value->points[0].angle);
 //   }
 //   out->ranges.resize(value->points.size());
 //   out->intensities.resize(value->points.size());
 //   for(size_t i = 0; i < value->points.size(); ++i) {
 //     out->ranges[i] = value->points[i].distance;
 //     out->intensities[i] = value->points[i].intensity;
 //   }
 //   export_publish(out);
 // }

 // void handle(std::shared_ptr<const pilot::Pose2D> value) override
 // {
 //   if(std::count(export_tf.begin(), export_tf.end(), vnx_sample->topic)) {
 //     handle(std::shared_ptr<const automy::basic::Transform3D>(value));
 //   }
 //   tf2::Quaternion q;
 //   auto out = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
 //   out->header.stamp = pilot_to_ros_time(value->time);
 //   out->header.frame_id = value->parent;
 //   out->pose.pose.position.x = value->pose.x();
 //   out->pose.pose.position.y = value->pose.y();
 //   q.setRPY(0, 0, value->pose.z());
 //   out->pose.pose.orientation = tf2::toMsg(q);
 //   export_publish(out);
 // }

 void handle(std::shared_ptr<const pilot::PoseArray2D> value) override
 {
   tf2::Quaternion q;
   auto out = std::make_shared<geometry_msgs::msg::PoseArray>();
   for(const auto& pose : value->poses) {
     geometry_msgs::msg::Pose tmp;
     tmp.position.x = pose.x();
     tmp.position.y = pose.y();
     q.setRPY(0, 0, pose.z());
     tmp.orientation = tf2::toMsg(q);
     out->poses.push_back(tmp);
   }
   export_publish(out);
 }

 // void handle(std::shared_ptr<const pilot::Odometry> value) override
 // {
 //   if(std::count(export_tf.begin(), export_tf.end(), vnx_sample->topic)) {
 //     handle(std::shared_ptr<const automy::basic::Transform3D>(value));
 //   }

 //   auto out = std::make_shared<nav_msgs::msg::Odometry>();
 //   out->header.stamp = pilot_to_ros_time(value->time);
 //   out->header.frame_id = value->parent;
 //   out->child_frame_id = value->frame;
 //   out->pose.pose.position.x = value->position[0];
 //   out->pose.pose.position.y = value->position[1];
 //   out->pose.pose.position.z = value->position[2];
 //   tf2::Quaternion q;
 //   q.setRPY(0, 0, pilot_to_ros_matrix_33(value->matrix.get<3, 3>()));
 //   out->info.origin.orientation = tf2::toMsg(q);
 //   out->twist.twist.linear.x = value->linear_velocity[0];
 //   out->twist.twist.linear.y = value->linear_velocity[1];
 //   out->twist.twist.linear.z = value->linear_velocity[2];
 //   out->twist.twist.angular.x = value->angular_velocity[0];
 //   out->twist.twist.angular.y = value->angular_velocity[1];
 //   out->twist.twist.angular.z = value->angular_velocity[2];
 //   export_publish(out);
 // }

 void handle(std::shared_ptr<const pilot::Path2D> value) override
 {
   tf2::Quaternion q;
   auto out = std::make_shared<nav_msgs::msg::Path>();
   out->header.frame_id = value->frame;
   out->header.stamp = pilot_to_ros_time(value->time);
   for(auto point : value->points) {
     geometry_msgs::msg::PoseStamped tmp;
     tmp.header = out->header;
     tmp.pose.position.x = point->pose.x();
     tmp.pose.position.y = point->pose.y();
     q.setRPY(0, 0, point->pose.z());
     tmp.pose.orientation = tf2::toMsg(q);
     out->poses.push_back(tmp);
   }
   export_publish(out);
 }

// Relayboard SystemState showing the errors
 void handle(std::shared_ptr<const pilot::SystemState> value) override
 {
   auto out = std::make_shared<neo_msgs2::msg::RelayBoardV2>();

   // time
   out->header.stamp = pilot_to_ros_time(value->time);

   for (auto it : value->system_errors) {
     switch(it)
     {
       case pilot::system_error_e::CHARGING_RELAY_ERROR: out->relayboardv2_state[0] = true; break;
       case pilot::system_error_e::BRAKE_RELEASE_BUTTON_ERROR: out->relayboardv2_state[1] = true; break;
       case pilot::system_error_e::MOTOR_ERROR: out->relayboardv2_state[2] = true; break;
       case pilot::system_error_e::SAFETY_RELAY_ERROR: out->relayboardv2_state[3] = true; break;
       case pilot::system_error_e::POWER_RELAY_ERROR: out->relayboardv2_state[4] = true; break;
       case pilot::system_error_e::EM_STOP_SYSTEM_ERROR: out->relayboardv2_state[5] = true; break;   
     } 
   } 
    
   std::shared_ptr<const pilot::BatteryState> bat_value;

   // Shutdown
   out->shutdown = value->is_shutdown; // relayboard is powering of in < 30s

   export_publish(out);
 }

// // Relayboard BatteryState
 void handle(std::shared_ptr<const pilot::BatteryState> value) override
 {
   battery = value;
   auto out = std::make_shared<sensor_msgs::msg::BatteryState>();
   // out->header.stamp = pilot_to_ros_time(value->time);

   out->voltage = value->voltage;        // float32 Voltage in Volts (Mandatory)
   out->current = value->current;        // float32 Negative when discharging (A)  (If unmeasured NaN)
   out->charge = NAN;              // float32 Current charge in Ah  (If unmeasured NaN)
   out->capacity = NAN;            // float32 Capacity in Ah (last full capacity)  (If unmeasured NaN)
   out->design_capacity = NAN;         // float32 Capacity in Ah (design capacity)  (If unmeasured NaN)
   out->percentage = value->remaining;     // float32 Charge percentage on 0 to 1 range  (If unmeasured NaN)
   out->power_supply_health = 0;           // uint8   The battery health metric.
   out->power_supply_technology = 0;     // uint8   The battery chemistry.
   out->present = true;            // bool    True if the battery is present
   out->location = '\000' ;            // The location into which the battery is inserted. (slot number or plug)
   out->serial_number = '\000' ;           // The best approximation of the battery serial number

   export_publish(out);
 }

// Relayboard EmergencyState
 void handle(std::shared_ptr<const pilot::EmergencyState> value) override
 {
   auto out = std::make_shared<neo_msgs2::msg::EmergencyStopState>();
   out->header.stamp = pilot_to_ros_time(value->time);

   // assign input (laser, button) specific EM state
   out->emergency_button_stop = false;
   out->scanner_stop = false;

   // Scanner stop or EMStop
   switch (value->code) {
     case pilot::safety_code_e::SCANNER_STOP: out->scanner_stop = true; break;
     case pilot::safety_code_e::EMERGENCY_STOP: out->emergency_button_stop = true; break;
   }

   // State of the EMStop
   switch (value->state) {
     case pilot::em_stop_state_e::FREE: out->emergency_state = 0; break;
     case pilot::em_stop_state_e::STOPPED: out->emergency_state = 1; break;
     case pilot::em_stop_state_e::CONFIRMED: out->emergency_state = 2; break;
   }

   export_publish(out);
 }

// // Relayboard IOBoardData
//  void handle(std::shared_ptr<const pilot::IOBoardData> value) override
//  {
//    neo_msgs::IOBoard::Ptr out = boost::make_shared<neo_msgs::IOBoard>();

//    // Assigning digital inputs and outputs
//    for (int iIOCnt = 0; iIOCnt < 16; iIOCnt++)
//    {
//      out->digital_inputs[iIOCnt] = value->digital_input[iIOCnt];
//      out->digital_outputs[iIOCnt] = value->digital_output[iIOCnt];
//    }

//    // Assigning analog inputs
//    for (int i = 0; i < 8; i++)
//    {
//      out->analog_inputs[i] = value->analog_input[i];
//    }

//    export_publish(out);
//  }

// // Relayboard USBoardData
//  void handle(std::shared_ptr<const pilot::USBoardData> value) override
//  {
//    neo_msgs::USBoardV2::Ptr out = boost::make_shared<neo_msgs::USBoardV2>();
//    out->header.stamp = pilot_to_ros_time(value->time);
    
//    for (int i = 0; i < 16; i++)
//    {
//      out->sensor[i] = value->sensor[i];
//    }
    
//    for (int i = 0; i < 4; i++)
//    {
//      out->analog[i] = value->analog_input[i];
//    }

//    // Publish raw data in neo_msgs::USBoardV2 format
//    export_publish(out);

//    // ToDo: Additionally publish data in ROS sensor_msgs::Range format
//    // for (int i = 0; i < 16; ++i)
//    // {
//    //  if(!m_bUSBoardSensorActive[i]) {
//    //    continue;
//    //  }
//    //  std_msgs::Header header;
//    //  header.stamp = m_tCurrentTimeStamp;              // time
//    //  header.frame_id = "us_" + std::to_string(i + 1) + "_link";  // string

//    //  sensor_msgs::Range us_range_msg;
//    //  us_range_msg.header = header;
//    //  us_range_msg.radiation_type = 0;        // uint8   => Enum ULTRASOUND=0; INFRARED=1
//    //  us_range_msg.field_of_view = 1.05;        // float32 [rad]
//    //  us_range_msg.min_range = 0.1;         // float32 [m]
//    //  us_range_msg.max_range = 1.2;         // float32 [m]
//    //  us_range_msg.range = out->sensor[i] / 100.f; // float32 [cm] => [m]

//    //  topicPub_USRangeSensor[i].publish(us_range_msg);
//    // }
//  }


 void handle(std::shared_ptr<const pilot::CostMapData> value) override
 {
   tf2::Quaternion q;
   auto out = std::make_shared<nav_msgs::msg::OccupancyGrid>();
   out->header.stamp = pilot_to_ros_time(value->time);
   out->header.frame_id = value->frame;
   out->info.resolution = value->scale;
   out->info.width = value->cost.width();
   out->info.height = value->cost.height();
   q.setRPY(0, 0, value->orientation);
   out->info.origin.position.x = value->origin.x();
   out->info.origin.position.y = value->origin.y();
   out->info.origin.orientation = tf2::toMsg(q);
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

 void handle(std::shared_ptr<const pilot::OccupancyMapData> value) override
 {
  tf2::Quaternion q;
  auto out = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  out->header.stamp = pilot_to_ros_time(value->time);
  out->header.frame_id = value->frame;
  out->info.resolution = value->scale;
  out->info.width = value->occupancy.width();
  out->info.height = value->occupancy.height();
  out->info.origin.position.x = value->origin.x();
  out->info.origin.position.y = value->origin.y();
  q.setRPY(0, 0, value->orientation);
  out->info.origin.orientation = tf2::toMsg(q);
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

//  void handle(std::shared_ptr<const pilot::RoadMapData> value) override
//  {
//    auto nodes = boost::make_shared<visualization_msgs::Marker>();
//    nodes->header.frame_id = map_frame;
//    nodes->ns = vnx_sample->topic->get_name() + ".nodes";
//    nodes->type = visualization_msgs::Marker::SPHERE_LIST;
//    nodes->scale.x = 0.1; nodes->scale.y = 0.1; nodes->scale.z = 0.1;
//    nodes->color.r = 0; nodes->color.g = 0; nodes->color.b = 1; nodes->color.a = 1;

//    auto segments = boost::make_shared<visualization_msgs::Marker>();
//    segments->header.frame_id = map_frame;
//    segments->ns = vnx_sample->topic->get_name() + ".segments";
//    segments->type = visualization_msgs::Marker::LINE_LIST;
//    segments->scale.x = 0.03;
//    segments->color.r = 1; segments->color.g = 0; segments->color.b = 1; segments->color.a = 0.3;

//    auto stations = boost::make_shared<geometry_msgs::PoseArray>();
//    stations->header.frame_id = map_frame;
//    auto markers = boost::make_shared<visualization_msgs::MarkerArray>();

//    std::map<int, std::shared_ptr<const pilot::MapNode>> node_map;

//    for(auto node : value->nodes) {
//      auto station = std::dynamic_pointer_cast<const pilot::MapStation>(node);
//      if(station) {
//        geometry_msgs::Pose tmp;
//        tmp.position.x = station->position.x();
//        tmp.position.y = station->position.y();
//        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(station->orientation), tmp.orientation);
//        stations->poses.push_back(tmp);
//        {
//          visualization_msgs::Marker marker;
//          marker.header.frame_id = map_frame;
//          marker.ns = vnx_sample->topic->get_name() + ".stations";
//          marker.id = node->id;
//          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//          marker.pose = tmp;
//          marker.text = station->name;
//          marker.scale.z = 0.25;
//          marker.color.r = 0; marker.color.g = 0; marker.color.b = 0; marker.color.a = 1;
//          markers->markers.push_back(marker);
//        }
//      }
//      geometry_msgs::Point point;
//      point.x = node->position.x();
//      point.y = node->position.y();
//      nodes->points.push_back(point);
//      node_map[node->id] = node;
//    }
//    for(auto segment : value->segments) {
//      {
//        auto node = node_map[segment->from_node];
//        geometry_msgs::Point point;
//        if(node) {
//          point.x = node->position.x();
//          point.y = node->position.y();
//        }
//        segments->points.push_back(point);
//      }
//      {
//        auto node = node_map[segment->to_node];
//        geometry_msgs::Point point;
//        if(node) {
//          point.x = node->position.x();
//          point.y = node->position.y();
//        }
//        segments->points.push_back(point);
//      }
//    }
//    export_publish(nodes, "nodes");
//    export_publish(segments, "segments");
//    export_publish(stations, "stations");
//    export_publish(markers, "markers");
//  }

 void handle(std::shared_ptr<const pilot::kinematics::differential::DriveState> value) override
 {
   auto out = std::make_shared<sensor_msgs::msg::JointState>();
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

 void handle(std::shared_ptr<const pilot::kinematics::mecanum::DriveState> value) override
 {
   auto out = std::make_shared<sensor_msgs::msg::JointState>();
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

 void handle(std::shared_ptr<const pilot::kinematics::omnidrive::DriveState> value) override
 {
  auto out = std::make_shared<sensor_msgs::msg::JointState>();
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
  out->position[0] = value->drive_pos.get(pilot::kinematics::position_code_e::FRONT_LEFT);
  out->position[1] = value->steer_pos.get(pilot::kinematics::position_code_e::FRONT_LEFT);
  out->position[2] = value->drive_pos.get(pilot::kinematics::position_code_e::BACK_LEFT);
  out->position[3] = value->steer_pos.get(pilot::kinematics::position_code_e::BACK_LEFT);
  out->position[4] = value->drive_pos.get(pilot::kinematics::position_code_e::BACK_RIGHT);
  out->position[5] = value->steer_pos.get(pilot::kinematics::position_code_e::BACK_RIGHT);
  out->position[6] = value->drive_pos.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
  out->position[7] = value->steer_pos.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
  out->velocity[0] = value->drive_vel.get(pilot::kinematics::position_code_e::FRONT_LEFT);
  out->velocity[1] = value->steer_vel.get(pilot::kinematics::position_code_e::FRONT_LEFT);
  out->velocity[2] = value->drive_vel.get(pilot::kinematics::position_code_e::BACK_LEFT);
  out->velocity[3] = value->steer_vel.get(pilot::kinematics::position_code_e::BACK_LEFT);
  out->velocity[4] = value->drive_vel.get(pilot::kinematics::position_code_e::BACK_RIGHT);
  out->velocity[5] = value->steer_vel.get(pilot::kinematics::position_code_e::BACK_RIGHT);
  out->velocity[6] = value->drive_vel.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
  out->velocity[7] = value->steer_vel.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
  if(value->has_torque) {
    out->effort.resize(8);
    out->effort[0] = value->drive_torque.get(pilot::kinematics::position_code_e::FRONT_LEFT);
    out->effort[1] = value->steer_torque.get(pilot::kinematics::position_code_e::FRONT_LEFT);
    out->effort[2] = value->drive_torque.get(pilot::kinematics::position_code_e::BACK_LEFT);
    out->effort[3] = value->steer_torque.get(pilot::kinematics::position_code_e::BACK_LEFT);
    out->effort[4] = value->drive_torque.get(pilot::kinematics::position_code_e::BACK_RIGHT);
    out->effort[5] = value->steer_torque.get(pilot::kinematics::position_code_e::BACK_RIGHT);
    out->effort[6] = value->drive_torque.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
    out->effort[7] = value->steer_torque.get(pilot::kinematics::position_code_e::FRONT_RIGHT);
  }

   export_publish(out);
 }

 void handle_twist(const geometry_msgs::msg::Twist::SharedPtr twist, const std::string& topic_name)
 {
   auto out = pilot::VelocityCmd::create();
   out->time = vnx::get_time_micros();
   out->linear.x() = twist->linear.x;
   out->linear.y() = twist->linear.y;
   out->linear.z() = twist->linear.z;
   out->angular.x() = twist->angular.x;
   out->angular.y() = twist->angular.y;
   out->angular.z() = twist->angular.z;
   import_publish(out, "cmd_vel");
 }

 void handle_pose(const geometry_msgs::msg::PoseStamped::SharedPtr pose, const std::string& topic_name)
 {
   tf2::Transform tmp;
   tf2::fromMsg(pose->pose, tmp);

   auto out = pilot::Pose2D::create();
   out->time = vnx::get_time_micros();
   out->frame = base_frame;
   out->parent = pose->header.frame_id;
   out->pose.x() = pose->pose.position.x;
   out->pose.y() = pose->pose.position.y;
   out->pose.z() = tf2::getYaw(pose->pose.orientation);
   out->update();
   import_publish(out, topic_name);
 }

 void handle_pose_cov(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, const std::string& topic_name)
 {
   tf2::Transform tmp;
   tf2::fromMsg(pose->pose.pose, tmp);

   auto out = pilot::Pose2D::create();
   out->time = vnx::get_time_micros();
   out->frame = base_frame;
   out->parent = pose->header.frame_id;
   out->pose.x() = pose->pose.pose.position.x;
   out->pose.y() = pose->pose.pose.position.y;
   out->pose.z() = tf2::getYaw(pose->pose.pose.orientation);
   out->update();
   import_publish(out, topic_name);
 }

// private:
//  bool serviceRelayBoardSetRelay(neo_srvs::RelayBoardSetRelay::Request &req,
//                          neo_srvs::RelayBoardSetRelay::Response &res)
//  {
//    try {
//      platform_interface.set_relay(req.id, req.state);
//    }
//    catch(std::exception& ex)
//    {
//      ROS_ERROR("Exception: Did not recieve message");
//      res.success = false;
//      return false;
//    }
//    res.success = true;
//    return true;
//  }

//  bool serviceStartCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
//  {
//    try {
//      platform_interface.start_charging();
//    }
//    catch(std::exception& ex)
//    {
//      ROS_ERROR("Exception: Did not recieve message");
//      return false;
//    }
//    return true;
//  }

//  bool serviceStopCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
//  {
//    try {
//      platform_interface.stop_charging();
//    }
//    catch(std::exception& ex)
//    {
//      ROS_ERROR("Exception: Did not recieve message");
//      return false;
//    }
//    return true;
//  }

//  bool serviceRelayBoardSetLCDMsg(neo_srvs::RelayBoardSetLCDMsg::Request &req,
//                             neo_srvs::RelayBoardSetLCDMsg::Response &res)
//  {
//    try {
//      platform_interface.set_display_text(req.message.c_str());
//    }
//    catch(std::exception& ex)
//    {
//      ROS_ERROR("Exception: Did not recieve message");
//      res.success = false;
//      return false;
//    }
//    res.success = true;
//    return true;
//  }


 void import_publish(std::shared_ptr<vnx::Value> sample, const std::string& ros_topic)
 {
   const auto range = import_topic_map.equal_range(ros_topic);
   for(auto entry = range.first; entry != range.second; ++entry) {
     publish(sample, entry->second);
   }
 }

 template<typename T>
 void export_publish(std::shared_ptr<T> sample, vnx::TopicPtr pilot_topic, const std::string& sub_topic)
 {
   const auto range = export_topic_map.equal_range(pilot_topic);
   for(auto entry = range.first; entry != range.second; ++entry)
   {
     const std::string ros_topic = entry->second + (sub_topic.empty() ? "" : "/") + sub_topic;
     if(!export_publishers.count(ros_topic)) {
       export_publishers[ros_topic] = this->create_publisher<T>(ros_topic, max_publish_queue_ros);
     }
     std::any_cast<typename rclcpp::Publisher<T>::SharedPtr>(export_publishers[ros_topic])->publish(*sample);
   }
 }

 template<typename T>
 void export_publish(std::shared_ptr<T> sample, const std::string& sub_topic = std::string())
 {
   if(!vnx_sample) {
     throw std::logic_error("!vnx_sample");
   }
   export_publish(sample, vnx_sample->topic, sub_topic);
 }

private:

//  tf::TransformBroadcaster broadcaster;

 std::map<std::string, std::any> import_subscribers;
 std::multimap<std::string, vnx::TopicPtr> import_topic_map;

 std::map<std::string, std::any> export_publishers;
 std::multimap<vnx::TopicPtr, std::string> export_topic_map;
 std::shared_ptr<const pilot::BatteryState> battery;

//  ros::ServiceServer srv_SetRelay = nh.advertiseService("set_relay", &Pilot_ROS_Bridge::serviceRelayBoardSetRelay, this);
//  ros::ServiceServer srv_StartCharging = nh.advertiseService("start_charging", &Pilot_ROS_Bridge::serviceStartCharging, this);
//  ros::ServiceServer srv_StopCharging = nh.advertiseService("stop_charging", &Pilot_ROS_Bridge::serviceStopCharging, this);
//  ros::ServiceServer srv_SetLCDMsg = nh.advertiseService("set_LCD_msg", &Pilot_ROS_Bridge::serviceRelayBoardSetLCDMsg, this);

  pilot::PlatformInterfaceClient platform_interface;
};


int main(int argc, char** argv)
{
  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize VNX
  vnx::init("pilot_ros_bridge_node", 0, nullptr);
  auto nh = std::make_shared<Pilot_ROS_Bridge>("ROS_Node");

  std::string pilot_node;
  std::string pilot_config;

  nh->declare_parameter<std::string>("pilot_node", "localhost:5555");
  nh->declare_parameter<std::string>("pilot_config", "config/default/generic/");

  nh->get_parameter<std::string>("pilot_node", pilot_node);
  nh->get_parameter<std::string>("pilot_config", pilot_config);

  vnx::read_config_tree(pilot_config);

  {
    vnx::Handle<vnx::Terminal> module = new vnx::Terminal("Terminal");
    module.start_detached();
  }

  vnx::Handle<vnx::Proxy> proxy = new vnx::Proxy("Proxy", vnx::Endpoint::from_url(pilot_node));
  proxy->time_sync = true;
  proxy->forward_list.push_back("PlatformInterface");
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

  rclcpp::spin(nh); // process incoming ROS messages

  vnx::close(); // trigger VNX shutdown and wait for all modules to exit

  return 0;
}

