#!/usr/bin/env python3
"""Synthetic FAST-LIVO-shaped source for deterministic software tests only."""

import math
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    normalized = math.fmod(angle + math.pi, 2.0 * math.pi)
    if normalized < 0.0:
        normalized += 2.0 * math.pi
    return normalized - math.pi


def compose_pose(parent, parent_to_child):
    cosine = math.cos(parent.yaw)
    sine = math.sin(parent.yaw)
    return Pose2(
        parent.x
        + cosine * parent_to_child.x
        - sine * parent_to_child.y,
        parent.y
        + sine * parent_to_child.x
        + cosine * parent_to_child.y,
        normalize_angle(parent.yaw + parent_to_child.yaw),
    )


def inverse_pose(transform):
    cosine = math.cos(transform.yaw)
    sine = math.sin(transform.yaw)
    return Pose2(
        -cosine * transform.x - sine * transform.y,
        sine * transform.x - cosine * transform.y,
        normalize_angle(-transform.yaw),
    )


def source_pose_from_rear(rear_pose, source_to_rear_x, source_to_rear_y,
                          source_to_rear_yaw):
    source_to_rear = Pose2(
        source_to_rear_x, source_to_rear_y, source_to_rear_yaw
    )
    return compose_pose(rear_pose, inverse_pose(source_to_rear))


def integrate_pose(pose, speed_mps, yaw_rate_radps, elapsed_s):
    yaw_change = yaw_rate_radps * elapsed_s
    midpoint_yaw = pose.yaw + 0.5 * yaw_change
    distance = speed_mps * elapsed_s
    return Pose2(
        pose.x + distance * math.cos(midpoint_yaw),
        pose.y + distance * math.sin(midpoint_yaw),
        normalize_angle(pose.yaw + yaw_change),
    )


def main():
    import rospy
    from auto_rover_interfaces.msg import ChassisState
    from nav_msgs.msg import Odometry

    rospy.init_node("fake_fast_livo_source")
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
    feedback_freshness_s = float(
        rospy.get_param("~feedback_freshness_s", 0.20)
    )
    if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
        raise RuntimeError("publish_rate_hz must be finite and positive")
    if not math.isfinite(feedback_freshness_s) or feedback_freshness_s <= 0.0:
        raise RuntimeError("feedback_freshness_s must be finite and positive")

    rear_pose = Pose2(
        float(rospy.get_param("~initial_rear_x_m", 0.0)),
        float(rospy.get_param("~initial_rear_y_m", 0.0)),
        float(rospy.get_param("~initial_rear_yaw_rad", 0.0)),
    )
    source_to_rear = Pose2(
        float(rospy.get_param("~source_to_rear_x_m", -0.20)),
        float(rospy.get_param("~source_to_rear_y_m", 0.0)),
        float(rospy.get_param("~source_to_rear_yaw_rad", 0.0)),
    )
    numeric_values = (
        rear_pose.x,
        rear_pose.y,
        rear_pose.yaw,
        source_to_rear.x,
        source_to_rear.y,
        source_to_rear.yaw,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise RuntimeError("fake source pose parameters must be finite")

    publisher = rospy.Publisher("aft_mapped_to_init", Odometry, queue_size=1)
    lock = threading.Lock()
    state = {
        "rear_pose": rear_pose,
        "speed": 0.0,
        "yaw_rate": 0.0,
        "last_feedback_monotonic": None,
        "last_step_monotonic": None,
    }

    def chassis_callback(message):
        speed_valid = (
            message.valid_mask & ChassisState.MEASURED_SPEED_VALID
        ) != 0
        yaw_rate_valid = (
            message.valid_mask & ChassisState.YAW_RATE_VALID
        ) != 0
        values_finite = math.isfinite(message.measured_speed_mps) and math.isfinite(
            message.yaw_rate_radps
        )
        with lock:
            if not message.valid or not speed_valid or not yaw_rate_valid or not values_finite:
                state["last_feedback_monotonic"] = None
                state["last_step_monotonic"] = None
                return
            state["speed"] = message.measured_speed_mps
            state["yaw_rate"] = message.yaw_rate_radps
            state["last_feedback_monotonic"] = time.monotonic()

    rospy.Subscriber("chassis_state", ChassisState, chassis_callback, queue_size=1)
    rate = rospy.Rate(publish_rate_hz)
    while not rospy.is_shutdown():
        now_monotonic = time.monotonic()
        with lock:
            feedback_time = state["last_feedback_monotonic"]
            if (
                feedback_time is None
                or now_monotonic - feedback_time > feedback_freshness_s
            ):
                state["last_step_monotonic"] = None
                should_publish = False
            else:
                last_step = state["last_step_monotonic"]
                elapsed = 0.0 if last_step is None else now_monotonic - last_step
                state["last_step_monotonic"] = now_monotonic
                state["rear_pose"] = integrate_pose(
                    state["rear_pose"], state["speed"], state["yaw_rate"], elapsed
                )
                source_pose = source_pose_from_rear(
                    state["rear_pose"],
                    source_to_rear.x,
                    source_to_rear.y,
                    source_to_rear.yaw,
                )
                should_publish = True
        if should_publish:
            message = Odometry()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "camera_init"
            message.child_frame_id = "aft_mapped"
            message.pose.pose.position.x = source_pose.x
            message.pose.pose.position.y = source_pose.y
            message.pose.pose.position.z = 0.0
            message.pose.pose.orientation.z = math.sin(0.5 * source_pose.yaw)
            message.pose.pose.orientation.w = math.cos(0.5 * source_pose.yaw)
            publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
