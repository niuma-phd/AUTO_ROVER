#pragma once

#include "auto_rover_core/types.hpp"
#include "auto_rover_interfaces/ChassisState.h"
#include "auto_rover_interfaces/EgoState.h"
#include "auto_rover_interfaces/EmergencyStop.h"
#include "auto_rover_interfaces/MotionReference.h"
#include "auto_rover_interfaces/RoutePlan.h"
#include "auto_rover_interfaces/SafetyState.h"
#include "auto_rover_interfaces/Trajectory.h"

namespace auto_rover_ros1 {

auto_rover_interfaces::EgoState toRos(const auto_rover::EgoState& value);
auto_rover::EgoState toCore(const auto_rover_interfaces::EgoState& message);

auto_rover_interfaces::RoutePlan toRos(const auto_rover::RoutePlan& value);
auto_rover::RoutePlan toCore(const auto_rover_interfaces::RoutePlan& message);

auto_rover_interfaces::Trajectory toRos(const auto_rover::Trajectory& value);
auto_rover::Trajectory toCore(
    const auto_rover_interfaces::Trajectory& message);

auto_rover_interfaces::MotionReference toRos(
    const auto_rover::MotionReference& value);
auto_rover::MotionReference toCore(
    const auto_rover_interfaces::MotionReference& message);

auto_rover_interfaces::ChassisState toRos(
    const auto_rover::ChassisState& value);
auto_rover::ChassisState toCore(
    const auto_rover_interfaces::ChassisState& message);

auto_rover_interfaces::SafetyState toRos(
    const auto_rover::SafetyState& value);
auto_rover::SafetyState toCore(
    const auto_rover_interfaces::SafetyState& message);

auto_rover_interfaces::EmergencyStop toRos(
    const auto_rover::EmergencyStop& value);
auto_rover::EmergencyStop toCore(
    const auto_rover_interfaces::EmergencyStop& message);

}  // namespace auto_rover_ros1
