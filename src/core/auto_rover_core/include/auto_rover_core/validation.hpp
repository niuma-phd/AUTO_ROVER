#pragma once

#include <string>

#include "auto_rover_core/types.hpp"

namespace auto_rover {

double maxAbsCurvature(const VehicleProfile& profile);
ValidationResult validateVehicleProfile(const VehicleProfile& profile);
ValidationResult validateNucPhase1Profile(const VehicleProfile& profile);
ValidationResult validateEgoState(const EgoState& ego,
                                  const std::string& expected_frame,
                                  const std::string& expected_reference_frame);
ValidationResult validateRoutePlan(const RoutePlan& route,
                                   const VehicleProfile& profile,
                                   const std::string& expected_frame);
ValidationResult validateTrajectory(const Trajectory& trajectory,
                                    const VehicleProfile& profile,
                                    const std::string& expected_frame);
ValidationResult validateMotionReference(const MotionReference& motion,
                                         const VehicleProfile& profile);
ValidationResult validateChassisState(const ChassisState& chassis,
                                      const VehicleProfile& profile,
                                      bool measured_speed_required);
ValidationResult validateSafetyState(const SafetyState& safety);
ValidationResult validateExecutionCommand(
    const VehicleExecutionCommand& command, const VehicleProfile& profile,
    std::int64_t now_monotonic_ns);

bool isFresh(std::int64_t receipt_monotonic_ns,
             std::int64_t now_monotonic_ns,
             std::int64_t freshness_limit_ns);
bool withinDeclaredValidity(std::int64_t production_ns,
                            std::int64_t now_ns,
                            std::int64_t valid_for_ns);

}  // namespace auto_rover
