#pragma once

#include "auto_rover_core/types.hpp"

namespace auto_rover {

bool isFinite(double value);
bool isFinite(const Vector3& value);
bool isFinite(const Quaternion& value);
bool isFinite(const Pose3& value);
bool isFinite(const Twist3& value);

bool normalizeQuaternion(const Quaternion& input, Quaternion* output);
Quaternion multiplyQuaternions(const Quaternion& left,
                               const Quaternion& right);
Vector3 rotateVector(const Quaternion& rotation, const Vector3& vector);
Pose3 composePoses(const Pose3& world_from_parent,
                   const Pose3& parent_from_child);
Quaternion quaternionFromYaw(double yaw_rad);
double yawFromQuaternion(const Quaternion& quaternion);
double normalizeAngle(double angle_rad);
double distance2d(double first_x, double first_y, double second_x,
                  double second_y);

}  // namespace auto_rover
