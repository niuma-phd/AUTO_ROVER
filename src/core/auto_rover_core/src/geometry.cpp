#include "auto_rover_core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_rover {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kQuaternionNormFloor = 1e-12;

}  // namespace

bool isFinite(double value) { return std::isfinite(value); }

bool isFinite(const Vector3& value) {
  return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
}

bool isFinite(const Quaternion& value) {
  return isFinite(value.x) && isFinite(value.y) && isFinite(value.z) &&
         isFinite(value.w);
}

bool isFinite(const Pose3& value) {
  return isFinite(value.position) && isFinite(value.orientation);
}

bool isFinite(const Twist3& value) {
  return isFinite(value.linear) && isFinite(value.angular);
}

bool normalizeQuaternion(const Quaternion& input, Quaternion* output) {
  if (output == nullptr || !isFinite(input)) {
    return false;
  }
  const double norm_squared = input.x * input.x + input.y * input.y +
                              input.z * input.z + input.w * input.w;
  if (!isFinite(norm_squared) || norm_squared <= kQuaternionNormFloor) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  output->x = input.x * inverse_norm;
  output->y = input.y * inverse_norm;
  output->z = input.z * inverse_norm;
  output->w = input.w * inverse_norm;
  return isFinite(*output);
}

Quaternion multiplyQuaternions(const Quaternion& left,
                               const Quaternion& right) {
  Quaternion output;
  output.x = left.w * right.x + left.x * right.w + left.y * right.z -
             left.z * right.y;
  output.y = left.w * right.y - left.x * right.z + left.y * right.w +
             left.z * right.x;
  output.z = left.w * right.z + left.x * right.y - left.y * right.x +
             left.z * right.w;
  output.w = left.w * right.w - left.x * right.x - left.y * right.y -
             left.z * right.z;
  Quaternion normalized;
  if (!normalizeQuaternion(output, &normalized)) {
    return Quaternion{};
  }
  return normalized;
}

Vector3 rotateVector(const Quaternion& rotation, const Vector3& vector) {
  Quaternion normalized;
  if (!normalizeQuaternion(rotation, &normalized) || !isFinite(vector)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid};
  }
  const Vector3 quaternion_vector{normalized.x, normalized.y, normalized.z};
  const Vector3 twice_cross{
      2.0 * (quaternion_vector.y * vector.z -
             quaternion_vector.z * vector.y),
      2.0 * (quaternion_vector.z * vector.x -
             quaternion_vector.x * vector.z),
      2.0 * (quaternion_vector.x * vector.y -
             quaternion_vector.y * vector.x)};
  return {
      vector.x + normalized.w * twice_cross.x +
          (quaternion_vector.y * twice_cross.z -
           quaternion_vector.z * twice_cross.y),
      vector.y + normalized.w * twice_cross.y +
          (quaternion_vector.z * twice_cross.x -
           quaternion_vector.x * twice_cross.z),
      vector.z + normalized.w * twice_cross.z +
          (quaternion_vector.x * twice_cross.y -
           quaternion_vector.y * twice_cross.x)};
}

Pose3 composePoses(const Pose3& world_from_parent,
                   const Pose3& parent_from_child) {
  Pose3 output;
  const Vector3 rotated =
      rotateVector(world_from_parent.orientation, parent_from_child.position);
  output.position.x = world_from_parent.position.x + rotated.x;
  output.position.y = world_from_parent.position.y + rotated.y;
  output.position.z = world_from_parent.position.z + rotated.z;
  output.orientation = multiplyQuaternions(world_from_parent.orientation,
                                           parent_from_child.orientation);
  return output;
}

Quaternion quaternionFromYaw(double yaw_rad) {
  if (!isFinite(yaw_rad)) {
    return Quaternion{};
  }
  const double half = 0.5 * yaw_rad;
  Quaternion output;
  output.z = std::sin(half);
  output.w = std::cos(half);
  return output;
}

double yawFromQuaternion(const Quaternion& quaternion) {
  Quaternion normalized;
  if (!normalizeQuaternion(quaternion, &normalized)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double sine =
      2.0 * (normalized.w * normalized.z + normalized.x * normalized.y);
  const double cosine = 1.0 -
                        2.0 * (normalized.y * normalized.y +
                               normalized.z * normalized.z);
  return normalizeAngle(std::atan2(sine, cosine));
}

double normalizeAngle(double angle_rad) {
  if (!isFinite(angle_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double normalized = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

double distance2d(double first_x, double first_y, double second_x,
                  double second_y) {
  if (!isFinite(first_x) || !isFinite(first_y) || !isFinite(second_x) ||
      !isFinite(second_y)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::hypot(second_x - first_x, second_y - first_y);
}

}  // namespace auto_rover
