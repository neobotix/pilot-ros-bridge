#!/bin/bash

VNX_INTERFACE_DIR=${VNX_INTERFACE_DIR:-/opt/neobotix/pilot-core/interface}

cd $(dirname "$0")

vnxcppcodegen --cleanup generated/ pilot.ros_bridge modules/ ${VNX_INTERFACE_DIR}/vnx/ ${VNX_INTERFACE_DIR}/vnx/addons/ ${VNX_INTERFACE_DIR}/automy/math/ ${VNX_INTERFACE_DIR}/automy/basic/ ${VNX_INTERFACE_DIR}/pilot/ ${VNX_INTERFACE_DIR}/pilot/base/ ${VNX_INTERFACE_DIR}/pilot/kinematics/ ${VNX_INTERFACE_DIR}/pilot/kinematics/differential/

