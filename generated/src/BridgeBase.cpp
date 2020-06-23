
// AUTO GENERATED by vnxcppcodegen

#include <vnx/vnx.h>
#include <pilot/ros/package.hxx>
#include <pilot/ros/BridgeBase.hxx>
#include <vnx/NoSuchMethod.hxx>
#include <automy/basic/Transform3D.hxx>
#include <pilot/GridMapData.hxx>
#include <pilot/LaserScan.hxx>
#include <pilot/Odometry.hxx>
#include <pilot/Pose2D.hxx>
#include <pilot/kinematics/differential/DriveState.hxx>
#include <vnx/Module.h>
#include <vnx/TopicPtr.hpp>



namespace pilot {
namespace ros {


const vnx::Hash64 BridgeBase::VNX_TYPE_HASH(0x4deabea977d4c59bull);
const vnx::Hash64 BridgeBase::VNX_CODE_HASH(0xd1e282d5c8bbc39bull);

BridgeBase::BridgeBase(const std::string& _vnx_name)
	:	Module::Module(_vnx_name)
{
	vnx::read_config(vnx_name + ".export_map", export_map);
	vnx::read_config(vnx_name + ".export_tf", export_tf);
	vnx::read_config(vnx_name + ".import_map", import_map);
	vnx::read_config(vnx_name + ".max_publish_queue_ros", max_publish_queue_ros);
	vnx::read_config(vnx_name + ".max_queue_ms_vnx", max_queue_ms_vnx);
	vnx::read_config(vnx_name + ".max_subscribe_queue_ros", max_subscribe_queue_ros);
}

vnx::Hash64 BridgeBase::get_type_hash() const {
	return VNX_TYPE_HASH;
}

const char* BridgeBase::get_type_name() const {
	return "pilot.ros.Bridge";
}
const vnx::TypeCode* BridgeBase::get_type_code() const {
	return pilot::ros::vnx_native_type_code_BridgeBase;
}

void BridgeBase::accept(vnx::Visitor& _visitor) const {
	const vnx::TypeCode* _type_code = pilot::ros::vnx_native_type_code_BridgeBase;
	_visitor.type_begin(*_type_code);
	_visitor.type_field(_type_code->fields[0], 0); vnx::accept(_visitor, export_tf);
	_visitor.type_field(_type_code->fields[1], 1); vnx::accept(_visitor, import_map);
	_visitor.type_field(_type_code->fields[2], 2); vnx::accept(_visitor, export_map);
	_visitor.type_field(_type_code->fields[3], 3); vnx::accept(_visitor, max_queue_ms_vnx);
	_visitor.type_field(_type_code->fields[4], 4); vnx::accept(_visitor, max_publish_queue_ros);
	_visitor.type_field(_type_code->fields[5], 5); vnx::accept(_visitor, max_subscribe_queue_ros);
	_visitor.type_end(*_type_code);
}

void BridgeBase::write(std::ostream& _out) const {
	_out << "{\"__type\": \"pilot.ros.Bridge\"";
	_out << ", \"export_tf\": "; vnx::write(_out, export_tf);
	_out << ", \"import_map\": "; vnx::write(_out, import_map);
	_out << ", \"export_map\": "; vnx::write(_out, export_map);
	_out << ", \"max_queue_ms_vnx\": "; vnx::write(_out, max_queue_ms_vnx);
	_out << ", \"max_publish_queue_ros\": "; vnx::write(_out, max_publish_queue_ros);
	_out << ", \"max_subscribe_queue_ros\": "; vnx::write(_out, max_subscribe_queue_ros);
	_out << "}";
}

void BridgeBase::read(std::istream& _in) {
	std::map<std::string, std::string> _object;
	vnx::read_object(_in, _object);
	for(const auto& _entry : _object) {
		if(_entry.first == "export_map") {
			vnx::from_string(_entry.second, export_map);
		} else if(_entry.first == "export_tf") {
			vnx::from_string(_entry.second, export_tf);
		} else if(_entry.first == "import_map") {
			vnx::from_string(_entry.second, import_map);
		} else if(_entry.first == "max_publish_queue_ros") {
			vnx::from_string(_entry.second, max_publish_queue_ros);
		} else if(_entry.first == "max_queue_ms_vnx") {
			vnx::from_string(_entry.second, max_queue_ms_vnx);
		} else if(_entry.first == "max_subscribe_queue_ros") {
			vnx::from_string(_entry.second, max_subscribe_queue_ros);
		}
	}
}

vnx::Object BridgeBase::to_object() const {
	vnx::Object _object;
	_object["__type"] = "pilot.ros.Bridge";
	_object["export_tf"] = export_tf;
	_object["import_map"] = import_map;
	_object["export_map"] = export_map;
	_object["max_queue_ms_vnx"] = max_queue_ms_vnx;
	_object["max_publish_queue_ros"] = max_publish_queue_ros;
	_object["max_subscribe_queue_ros"] = max_subscribe_queue_ros;
	return _object;
}

void BridgeBase::from_object(const vnx::Object& _object) {
	for(const auto& _entry : _object.field) {
		if(_entry.first == "export_map") {
			_entry.second.to(export_map);
		} else if(_entry.first == "export_tf") {
			_entry.second.to(export_tf);
		} else if(_entry.first == "import_map") {
			_entry.second.to(import_map);
		} else if(_entry.first == "max_publish_queue_ros") {
			_entry.second.to(max_publish_queue_ros);
		} else if(_entry.first == "max_queue_ms_vnx") {
			_entry.second.to(max_queue_ms_vnx);
		} else if(_entry.first == "max_subscribe_queue_ros") {
			_entry.second.to(max_subscribe_queue_ros);
		}
	}
}

/// \private
std::ostream& operator<<(std::ostream& _out, const BridgeBase& _value) {
	_value.write(_out);
	return _out;
}

/// \private
std::istream& operator>>(std::istream& _in, BridgeBase& _value) {
	_value.read(_in);
	return _in;
}

const vnx::TypeCode* BridgeBase::static_get_type_code() {
	const vnx::TypeCode* type_code = vnx::get_type_code(VNX_TYPE_HASH);
	if(!type_code) {
		type_code = vnx::register_type_code(static_create_type_code());
	}
	return type_code;
}

std::shared_ptr<vnx::TypeCode> BridgeBase::static_create_type_code() {
	std::shared_ptr<vnx::TypeCode> type_code = std::make_shared<vnx::TypeCode>();
	type_code->name = "pilot.ros.Bridge";
	type_code->type_hash = vnx::Hash64(0x4deabea977d4c59bull);
	type_code->code_hash = vnx::Hash64(0xd1e282d5c8bbc39bull);
	type_code->is_native = true;
	type_code->methods.resize(0);
	type_code->fields.resize(6);
	{
		vnx::TypeField& field = type_code->fields[0];
		field.is_extended = true;
		field.name = "export_tf";
		field.code = {12, 12, 5};
	}
	{
		vnx::TypeField& field = type_code->fields[1];
		field.is_extended = true;
		field.name = "import_map";
		field.code = {12, 23, 2, 4, 12, 23, 2, 4, 6, 12, 5, 12, 5, 12, 5};
	}
	{
		vnx::TypeField& field = type_code->fields[2];
		field.is_extended = true;
		field.name = "export_map";
		field.code = {12, 23, 2, 4, 6, 12, 5, 23, 2, 4, 6, 12, 5, 12, 5};
	}
	{
		vnx::TypeField& field = type_code->fields[3];
		field.name = "max_queue_ms_vnx";
		field.value = vnx::to_string(100);
		field.code = {7};
	}
	{
		vnx::TypeField& field = type_code->fields[4];
		field.name = "max_publish_queue_ros";
		field.value = vnx::to_string(1);
		field.code = {7};
	}
	{
		vnx::TypeField& field = type_code->fields[5];
		field.name = "max_subscribe_queue_ros";
		field.value = vnx::to_string(1);
		field.code = {7};
	}
	type_code->build();
	return type_code;
}

void BridgeBase::vnx_handle_switch(std::shared_ptr<const vnx::Sample> _sample) {
	const auto _type_hash = _sample->value->get_type_hash();
	if(_type_hash == 0xe762feb1b334b36dull) {
		auto _value = std::dynamic_pointer_cast<const ::automy::basic::Transform3D>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	} else if(_type_hash == 0xf18f0311672bb286ull) {
		auto _value = std::dynamic_pointer_cast<const ::pilot::GridMapData>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	} else if(_type_hash == 0x865aafd7c578368ull) {
		auto _value = std::dynamic_pointer_cast<const ::pilot::LaserScan>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	} else if(_type_hash == 0xcecf75b564f86511ull) {
		auto _value = std::dynamic_pointer_cast<const ::pilot::Odometry>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	} else if(_type_hash == 0x582f1fd83769573full) {
		auto _value = std::dynamic_pointer_cast<const ::pilot::Pose2D>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	} else if(_type_hash == 0x954b1e7cfbb6b85ull) {
		auto _value = std::dynamic_pointer_cast<const ::pilot::kinematics::differential::DriveState>(_sample->value);
		if(_value) {
			handle(_value, _sample);
		}
	}
}

std::shared_ptr<vnx::Value> BridgeBase::vnx_call_switch(std::shared_ptr<const vnx::Value> _method, const vnx::request_id_t& _request_id) {
	auto _ex = vnx::NoSuchMethod::create();
	_ex->dst_mac = vnx_request ? vnx_request->dst_mac : 0;
	_ex->method = _method->get_type_name();
	return _ex;
}


} // namespace pilot
} // namespace ros


namespace vnx {

void read(TypeInput& in, ::pilot::ros::BridgeBase& value, const TypeCode* type_code, const uint16_t* code) {
	if(!type_code) {
		throw std::logic_error("read(): type_code == 0");
	}
	if(code) {
		switch(code[0]) {
			case CODE_STRUCT: type_code = type_code->depends[code[1]]; break;
			case CODE_ALT_STRUCT: type_code = type_code->depends[vnx::flip_bytes(code[1])]; break;
			default: vnx::skip(in, type_code, code); return;
		}
	}
	const char* const _buf = in.read(type_code->total_field_size);
	if(type_code->is_matched) {
		{
			const vnx::TypeField* const _field = type_code->field_map[3];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.max_queue_ms_vnx, _field->code.data());
			}
		}
		{
			const vnx::TypeField* const _field = type_code->field_map[4];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.max_publish_queue_ros, _field->code.data());
			}
		}
		{
			const vnx::TypeField* const _field = type_code->field_map[5];
			if(_field) {
				vnx::read_value(_buf + _field->offset, value.max_subscribe_queue_ros, _field->code.data());
			}
		}
	}
	for(const vnx::TypeField* _field : type_code->ext_fields) {
		switch(_field->native_index) {
			case 0: vnx::read(in, value.export_tf, type_code, _field->code.data()); break;
			case 1: vnx::read(in, value.import_map, type_code, _field->code.data()); break;
			case 2: vnx::read(in, value.export_map, type_code, _field->code.data()); break;
			default: vnx::skip(in, type_code, _field->code.data());
		}
	}
}

void write(TypeOutput& out, const ::pilot::ros::BridgeBase& value, const TypeCode* type_code, const uint16_t* code) {
	if(!type_code || (code && code[0] == CODE_ANY)) {
		type_code = pilot::ros::vnx_native_type_code_BridgeBase;
		out.write_type_code(type_code);
		vnx::write_class_header<::pilot::ros::BridgeBase>(out);
	}
	if(code && code[0] == CODE_STRUCT) {
		type_code = type_code->depends[code[1]];
	}
	char* const _buf = out.write(12);
	vnx::write_value(_buf + 0, value.max_queue_ms_vnx);
	vnx::write_value(_buf + 4, value.max_publish_queue_ros);
	vnx::write_value(_buf + 8, value.max_subscribe_queue_ros);
	vnx::write(out, value.export_tf, type_code, type_code->fields[0].code.data());
	vnx::write(out, value.import_map, type_code, type_code->fields[1].code.data());
	vnx::write(out, value.export_map, type_code, type_code->fields[2].code.data());
}

void read(std::istream& in, ::pilot::ros::BridgeBase& value) {
	value.read(in);
}

void write(std::ostream& out, const ::pilot::ros::BridgeBase& value) {
	value.write(out);
}

void accept(Visitor& visitor, const ::pilot::ros::BridgeBase& value) {
	value.accept(visitor);
}

} // vnx