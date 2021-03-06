
// AUTO GENERATED by vnxcppcodegen

#ifndef INCLUDE_pilot_ros_bridge_PACKAGE_HXX_
#define INCLUDE_pilot_ros_bridge_PACKAGE_HXX_

#include <vnx/Type.h>

#include <vnx/package.hxx>


namespace pilot {
namespace ros_bridge {


class BridgeBase;

extern const vnx::TypeCode* const vnx_native_type_code_BridgeBase; ///< \private

} // namespace pilot
} // namespace ros_bridge


namespace vnx {

void read(TypeInput& in, ::pilot::ros_bridge::BridgeBase& value, const TypeCode* type_code, const uint16_t* code); ///< \private

void write(TypeOutput& out, const ::pilot::ros_bridge::BridgeBase& value, const TypeCode* type_code, const uint16_t* code); ///< \private

void read(std::istream& in, ::pilot::ros_bridge::BridgeBase& value); ///< \private

void write(std::ostream& out, const ::pilot::ros_bridge::BridgeBase& value); ///< \private

void accept(Visitor& visitor, const ::pilot::ros_bridge::BridgeBase& value); ///< \private

/// \private
template<>
struct type<::pilot::ros_bridge::BridgeBase> {
	void read(TypeInput& in, ::pilot::ros_bridge::BridgeBase& value, const TypeCode* type_code, const uint16_t* code) {
		vnx::read(in, value, type_code, code);
	}
	void write(TypeOutput& out, const ::pilot::ros_bridge::BridgeBase& value, const TypeCode* type_code, const uint16_t* code) {
		vnx::write(out, value, type_code, code);
	}
	void read(std::istream& in, ::pilot::ros_bridge::BridgeBase& value) {
		vnx::read(in, value);
	}
	void write(std::ostream& out, const ::pilot::ros_bridge::BridgeBase& value) {
		vnx::write(out, value);
	}
	void accept(Visitor& visitor, const ::pilot::ros_bridge::BridgeBase& value) {
		vnx::accept(visitor, value);
	}
};


} // namespace vnx

#endif // INCLUDE_pilot_ros_bridge_PACKAGE_HXX_
