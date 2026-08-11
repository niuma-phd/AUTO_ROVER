#include <cmath>

#include <gtest/gtest.h>

#include "auto_rover_ros1_conversions/conversions.hpp"

namespace {

TEST(Ros1Conversions, EgoRoundTripPreservesExplicitTimeMeaning) {
  auto_rover::EgoState input;
  input.stamp_ns = 1234567890LL;
  input.frame_id = "camera_init";
  input.state_id = 7U;
  input.time_source = auto_rover::TimeSource::kPublishTime;
  input.reference_frame = "rear_axle_center";
  input.pose.position = {1.0, -2.0, 0.5};
  input.pose.orientation.w = 1.0;
  input.source_id = "localization_provider_generation_1";
  input.valid = true;

  const auto message = auto_rover_ros1::toRos(input);
  const auto output = auto_rover_ros1::toCore(message);

  EXPECT_EQ(input.stamp_ns, output.stamp_ns);
  EXPECT_EQ(input.frame_id, output.frame_id);
  EXPECT_EQ(input.state_id, output.state_id);
  EXPECT_EQ(input.time_source, output.time_source);
  EXPECT_EQ(input.reference_frame, output.reference_frame);
  EXPECT_DOUBLE_EQ(input.pose.position.x, output.pose.position.x);
  EXPECT_DOUBLE_EQ(input.pose.position.y, output.pose.position.y);
  EXPECT_DOUBLE_EQ(input.pose.position.z, output.pose.position.z);
  EXPECT_EQ(input.source_id, output.source_id);
  EXPECT_TRUE(output.valid);
}

TEST(Ros1Conversions, RouteAndTrajectoryRoundTripPreserveSemanticIdentity) {
  auto_rover::RoutePlan route;
  route.schema_version = 1U;
  route.stamp_ns = 2000000000LL;
  route.frame_id = "camera_init";
  route.route_id = "commissioning_route_01";
  route.plan_version = 4U;
  route.waypoints.push_back({0.0, 0.0, 0.0, 0.5});
  route.waypoints.push_back({2.0, 0.0, 0.0, 0.5});
  const auto route_output = auto_rover_ros1::toCore(auto_rover_ros1::toRos(route));
  ASSERT_EQ(2U, route_output.waypoints.size());
  EXPECT_EQ(route.route_id, route_output.route_id);
  EXPECT_EQ(route.plan_version, route_output.plan_version);
  EXPECT_DOUBLE_EQ(0.5, route_output.waypoints[1].speed_mps);

  auto_rover::Trajectory trajectory;
  trajectory.stamp_ns = 3000000000LL;
  trajectory.frame_id = "camera_init";
  trajectory.trajectory_id = "commissioning_route_01:4:1";
  trajectory.route_id = route.route_id;
  trajectory.plan_version = route.plan_version;
  trajectory.vehicle_profile_id = "nuc_senior_akm_v1";
  trajectory.valid_for_ns = 250000000LL;
  trajectory.completion_behavior =
      auto_rover::CompletionBehavior::kStopAndHold;
  trajectory.valid = true;
  trajectory.points.push_back(
      {0.0, 0.0, 0.0, 0.0, 0.5, 0.0, auto_rover::Direction::kForward});
  trajectory.points.push_back(
      {2.0, 0.0, 0.0, 0.0, 0.5, 2.0, auto_rover::Direction::kForward});
  const auto trajectory_output =
      auto_rover_ros1::toCore(auto_rover_ros1::toRos(trajectory));
  EXPECT_EQ(trajectory.trajectory_id, trajectory_output.trajectory_id);
  EXPECT_EQ(trajectory.valid_for_ns, trajectory_output.valid_for_ns);
  EXPECT_EQ(auto_rover::Direction::kForward,
            trajectory_output.points[1].direction);
}

TEST(Ros1Conversions, MotionAndChassisKeepUnavailableValuesExplicit) {
  auto_rover::MotionReference motion;
  motion.stamp_ns = 4000000000LL;
  motion.frame_id = "rear_axle_center";
  motion.command_id = 11U;
  motion.producer_generation_id = "conversion-tracker-generation-1";
  motion.trajectory_id = "route:1:1";
  motion.direction = auto_rover::Direction::kForward;
  motion.target_speed_mps = 0.04;
  motion.target_curvature_inv_m = -0.2;
  motion.valid_for_ns = 100000000LL;
  motion.valid = true;
  const auto motion_output =
      auto_rover_ros1::toCore(auto_rover_ros1::toRos(motion));
  EXPECT_EQ(motion.producer_generation_id,
            motion_output.producer_generation_id);
  EXPECT_DOUBLE_EQ(0.04, motion_output.target_speed_mps);
  EXPECT_DOUBLE_EQ(-0.2, motion_output.target_curvature_inv_m);

  auto_rover::ChassisState chassis;
  chassis.stamp_ns = 5000000000LL;
  chassis.frame_id = "rear_axle_center";
  chassis.state_id = 3U;
  chassis.time_source = auto_rover::TimeSource::kReceiptTime;
  chassis.valid_mask = auto_rover::ChassisState::kMeasuredSpeedValid;
  chassis.source_id = "fake_vcu";
  chassis.valid = true;
  for (const double measured_speed_mps : {-0.001, -3.75}) {
    chassis.measured_speed_mps = measured_speed_mps;
    const auto chassis_output =
        auto_rover_ros1::toCore(auto_rover_ros1::toRos(chassis));
    EXPECT_EQ(auto_rover::ChassisState::kMeasuredSpeedValid,
              chassis_output.valid_mask);
    EXPECT_DOUBLE_EQ(measured_speed_mps,
                     chassis_output.measured_speed_mps);
    EXPECT_EQ(auto_rover::GearState::kUnknown, chassis_output.gear_state);
    EXPECT_EQ(auto_rover::FaultState::kUnknown, chassis_output.fault_state);
    EXPECT_TRUE(chassis_output.valid);
  }
}

TEST(Ros1Conversions, SafetyAndEmergencyStopRoundTrip) {
  auto_rover::SafetyState safety;
  safety.stamp_ns = 6000000000LL;
  safety.state_id = 8U;
  safety.mode = auto_rover::SafetyMode::kEmergencyStopLatched;
  safety.latch_generation = 2U;
  safety.valid_for_ns = 100000000LL;
  safety.reasons = {auto_rover::StopReason::kEmergencyStop};
  safety.valid = true;
  const auto safety_output =
      auto_rover_ros1::toCore(auto_rover_ros1::toRos(safety));
  ASSERT_EQ(1U, safety_output.reasons.size());
  EXPECT_EQ(auto_rover::StopReason::kEmergencyStop,
            safety_output.reasons.front());
  EXPECT_EQ(2U, safety_output.latch_generation);

  auto_rover::EmergencyStop stop;
  stop.stamp_ns = 7000000000LL;
  stop.request_id = "operator-panel:42";
  stop.source_id = "operator-panel";
  stop.asserted = true;
  const auto stop_output =
      auto_rover_ros1::toCore(auto_rover_ros1::toRos(stop));
  EXPECT_EQ(stop.request_id, stop_output.request_id);
  EXPECT_TRUE(stop_output.asserted);
}

TEST(Ros1Conversions, UnsupportedSchemasFailClosedAtTheRosBoundary) {
  auto_rover::EgoState ego;
  ego.stamp_ns = 1000000000LL;
  ego.frame_id = "camera_init";
  ego.reference_frame = "rear_axle_center";
  ego.pose.orientation.w = 1.0;
  ego.valid = true;
  auto ego_message = auto_rover_ros1::toRos(ego);
  ego_message.schema_version = 2U;
  EXPECT_FALSE(auto_rover_ros1::toCore(ego_message).valid);

  auto_rover::Trajectory trajectory;
  trajectory.stamp_ns = 1000000000LL;
  trajectory.valid = true;
  auto trajectory_message = auto_rover_ros1::toRos(trajectory);
  trajectory_message.schema_version = 2U;
  EXPECT_FALSE(auto_rover_ros1::toCore(trajectory_message).valid);

  auto_rover::MotionReference motion;
  motion.stamp_ns = 1000000000LL;
  motion.valid = true;
  auto motion_message = auto_rover_ros1::toRos(motion);
  motion_message.schema_version = 2U;
  EXPECT_FALSE(auto_rover_ros1::toCore(motion_message).valid);

  auto_rover::ChassisState chassis;
  chassis.stamp_ns = 1000000000LL;
  chassis.valid = true;
  auto chassis_message = auto_rover_ros1::toRos(chassis);
  chassis_message.schema_version = 2U;
  EXPECT_FALSE(auto_rover_ros1::toCore(chassis_message).valid);

  auto_rover::SafetyState safety;
  safety.stamp_ns = 1000000000LL;
  safety.valid = true;
  auto safety_message = auto_rover_ros1::toRos(safety);
  safety_message.schema_version = 2U;
  EXPECT_FALSE(auto_rover_ros1::toCore(safety_message).valid);

  auto_rover::EmergencyStop stop;
  stop.stamp_ns = 1000000000LL;
  stop.request_id = "wrong-schema";
  stop.source_id = "test";
  stop.asserted = true;
  auto stop_message = auto_rover_ros1::toRos(stop);
  stop_message.schema_version = 2U;
  const auto rejected_stop = auto_rover_ros1::toCore(stop_message);
  EXPECT_FALSE(rejected_stop.asserted);
  EXPECT_TRUE(rejected_stop.request_id.empty());
}

TEST(Ros1Conversions, UnsupportedEnumValuesFailClosedAtTheRosBoundary) {
  auto_rover::EgoState ego;
  ego.time_source = auto_rover::TimeSource::kPublishTime;
  ego.valid = true;
  auto ego_message = auto_rover_ros1::toRos(ego);
  ego_message.time_source = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(ego_message).valid);

  auto_rover::Trajectory trajectory;
  trajectory.completion_behavior =
      auto_rover::CompletionBehavior::kStopAndHold;
  trajectory.valid = true;
  trajectory.points.push_back(auto_rover::TrajectoryPoint{});
  trajectory.points.back().direction = auto_rover::Direction::kForward;
  auto trajectory_message = auto_rover_ros1::toRos(trajectory);
  trajectory_message.points.front().direction = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(trajectory_message).valid);

  auto_rover::MotionReference motion;
  motion.direction = auto_rover::Direction::kForward;
  motion.valid = true;
  auto motion_message = auto_rover_ros1::toRos(motion);
  motion_message.direction = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(motion_message).valid);

  auto_rover::ChassisState chassis;
  chassis.time_source = auto_rover::TimeSource::kReceiptTime;
  chassis.valid = true;
  auto chassis_message = auto_rover_ros1::toRos(chassis);
  chassis_message.valid_mask = auto_rover::ChassisState::kGearValid;
  chassis_message.gear_state = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(chassis_message).valid);
  chassis_message = auto_rover_ros1::toRos(chassis);
  chassis_message.time_source = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(chassis_message).valid);

  auto_rover::SafetyState safety;
  safety.mode = auto_rover::SafetyMode::kDisarmed;
  safety.valid = true;
  auto safety_message = auto_rover_ros1::toRos(safety);
  safety_message.mode = 255U;
  EXPECT_FALSE(auto_rover_ros1::toCore(safety_message).valid);
  safety_message = auto_rover_ros1::toRos(safety);
  safety_message.reason_codes.push_back(255U);
  EXPECT_FALSE(auto_rover_ros1::toCore(safety_message).valid);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
