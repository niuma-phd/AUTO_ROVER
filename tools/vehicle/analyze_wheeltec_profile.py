#!/usr/bin/env python3
"""Deterministically summarize Wheeltec raised-bench NDJSON evidence.

This tool is deliberately offline.  It accepts only regular files, never opens
a serial device, and never imports ROS.  Its time associations are between host
monotonic receipt/write events; they are not VCU acknowledgements or source-time
latency measurements.
"""

import argparse
import bisect
import hashlib
import json
import math
import os
import stat
import sys
from collections import Counter
from pathlib import Path


ANALYSIS_SCHEMA = "auto_rover.wheeltec.profile_analysis.v1"
BENCH_SCHEMA = "auto_rover.wheeltec.bench_characterization.v1"
RAW_PROFILE_SCHEMA = "auto_rover.wheeltec.raw_profile_capture.v1"
SUPPORTED_SCHEMAS = {BENCH_SCHEMA, RAW_PROFILE_SCHEMA}
REAR_TRACK_M = 0.322
MAXIMUM_FILE_BYTES = 512 * 1024 * 1024
MAXIMUM_LINE_BYTES = 4 * 1024 * 1024
MAXIMUM_RECORDS = 5_000_000
DEFAULT_SUSTAINED_FRAMES = 5
PHASES = ("baseline", "ramp_up", "hold", "ramp_down", "post_stop")
PHASE_RANK = {phase: index for index, phase in enumerate(PHASES)}
TRANSPORT_STATUSES = {
    "ok",
    "would_block",
    "deadline_exceeded",
    "disconnected",
    "disabled",
    "invalid_argument",
    "io_error",
}
READ_TIMEOUT_STATUSES = {"would_block", "deadline_exceeded"}

BENCH_RECORD_TYPES = {
    "cli_preflight",
    "open_result",
    "metadata",
    "rx_attempt",
    "rx_result",
    "feedback_frame",
    "tx_attempt",
    "tx_result",
    "emergency_zero_tx_attempt_postwrite_record",
    "emergency_zero_tx_result_postwrite_record",
    "summary",
}

RAW_PROFILE_RECORD_TYPES = {
    "cli_preflight",
    "open_result",
    "startup_exact_zero_result",
    "metadata",
    "drain_rx_attempt",
    "drain_rx_result",
    "normal_tx_attempt",
    "normal_tx_result",
    "rx_attempt",
    "rx_result",
    "raw_rx_chunk",
    "parser_observation_event",
    "feedback_observation_event",
    "emergency_zero_tx_attempt_postwrite_record",
    "emergency_zero_tx_result_postwrite_record",
    "summary",
}


class AnalysisError(ValueError):
    """The evidence cannot be interpreted without weakening validation."""


def _error(line_number, message):
    if line_number is None:
        raise AnalysisError(message)
    raise AnalysisError("line {}: {}".format(line_number, message))


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise AnalysisError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def _reject_nonfinite(token):
    raise AnalysisError("non-finite JSON number is forbidden: {}".format(token))


def _read_records(path):
    path = Path(path)
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        file_descriptor = os.open(str(path), flags)
    except OSError as error:
        raise AnalysisError("cannot open regular evidence file: {}".format(error))

    try:
        file_status = os.fstat(file_descriptor)
        if not stat.S_ISREG(file_status.st_mode):
            raise AnalysisError("input must be a regular file")
        if file_status.st_size <= 0:
            raise AnalysisError("input evidence file is empty")
        if file_status.st_size > MAXIMUM_FILE_BYTES:
            raise AnalysisError("input evidence file exceeds size limit")

        records = []
        digest = hashlib.sha256()
        with os.fdopen(file_descriptor, "rb", closefd=False) as stream:
            for line_number, raw_line in enumerate(stream, 1):
                digest.update(raw_line)
                if len(raw_line) > MAXIMUM_LINE_BYTES:
                    _error(line_number, "NDJSON line exceeds size limit")
                if not raw_line.strip():
                    _error(line_number, "blank NDJSON lines are forbidden")
                if len(records) >= MAXIMUM_RECORDS:
                    _error(line_number, "record count exceeds limit")
                try:
                    text = raw_line.decode("utf-8")
                except UnicodeDecodeError as error:
                    _error(line_number, "invalid UTF-8: {}".format(error))
                try:
                    record = json.loads(
                        text,
                        object_pairs_hook=_reject_duplicate_keys,
                        parse_constant=_reject_nonfinite,
                    )
                except json.JSONDecodeError as error:
                    _error(line_number, "invalid JSON: {}".format(error.msg))
                except AnalysisError as error:
                    _error(line_number, str(error))
                if not isinstance(record, dict):
                    _error(line_number, "each NDJSON value must be an object")
                records.append((line_number, record))
    finally:
        os.close(file_descriptor)

    return records, digest.hexdigest(), file_status.st_size


def _required(record, key, line_number):
    if key not in record:
        _error(line_number, "missing required field '{}'".format(key))
    return record[key]


def _string(record, key, line_number, nonempty=True):
    value = _required(record, key, line_number)
    if not isinstance(value, str) or (nonempty and not value):
        _error(line_number, "'{}' must be a{} string".format(
            key, " non-empty" if nonempty else ""
        ))
    return value


def _boolean(record, key, line_number):
    value = _required(record, key, line_number)
    if not isinstance(value, bool):
        _error(line_number, "'{}' must be boolean".format(key))
    return value


def _integer(record, key, line_number, minimum=None):
    value = _required(record, key, line_number)
    if isinstance(value, bool) or not isinstance(value, int):
        _error(line_number, "'{}' must be an integer".format(key))
    if minimum is not None and value < minimum:
        _error(line_number, "'{}' is below its minimum".format(key))
    return value


def _number(record, key, line_number, minimum=None):
    value = _required(record, key, line_number)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _error(line_number, "'{}' must be numeric".format(key))
    value = float(value)
    if not math.isfinite(value):
        _error(line_number, "'{}' must be finite".format(key))
    if minimum is not None and value < minimum:
        _error(line_number, "'{}' is below its minimum".format(key))
    return value


def _optional_number(record, key, line_number):
    if key not in record or record[key] is None:
        return None
    return _number(record, key, line_number)


def _optional_integer(record, key, line_number, minimum=None):
    if key not in record or record[key] is None:
        return None
    return _integer(record, key, line_number, minimum)


def _almost_equal(first, second, tolerance=1.0e-12):
    return abs(first - second) <= tolerance


def _signed_int16(high, low):
    bits = (high << 8) | low
    return bits if bits <= 0x7FFF else bits - 0x10000


def _hex_bytes(record, key, expected_size, line_number):
    raw_hex = _string(record, key, line_number)
    if len(raw_hex) != expected_size * 2 or any(
        byte not in "0123456789abcdefABCDEF" for byte in raw_hex
    ):
        _error(line_number, "'{}' must contain exactly {} hexadecimal bytes".format(
            key, expected_size
        ))
    try:
        return bytes.fromhex(raw_hex)
    except ValueError:
        _error(line_number, "'{}' is invalid hexadecimal data".format(key))


def _xor(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def _decode_command(record, line_number):
    frame = _hex_bytes(record, "raw_hex", 11, line_number)
    if frame[0] != 0x7B or frame[10] != 0x7D:
        _error(line_number, "command frame header or tail is invalid")
    if _xor(frame[:9]) != frame[9]:
        _error(line_number, "command frame checksum is invalid")
    if frame[1] != 0 or frame[2] != 0:
        _error(line_number, "normal command mode/reserved bytes must be zero")
    forward_wire = _signed_int16(frame[3], frame[4])
    lateral_wire = _signed_int16(frame[5], frame[6])
    yaw_wire = _signed_int16(frame[7], frame[8])
    if lateral_wire != 0:
        _error(line_number, "normal Ackermann command has non-zero lateral wire value")
    return {
        "forward_wire": forward_wire,
        "lateral_wire": lateral_wire,
        "yaw_wire": yaw_wire,
        "forward_mps": forward_wire / 1000.0,
        "yaw_radps": yaw_wire / 1000.0,
        "raw_hex": frame.hex(),
    }


def _decode_feedback(record, line_number):
    frame = _hex_bytes(record, "raw_hex", 24, line_number)
    if frame[0] != 0x7B or frame[23] != 0x7D:
        _error(line_number, "feedback frame header or tail is invalid")
    if _xor(frame[:22]) != frame[22]:
        _error(line_number, "feedback frame checksum is invalid")
    return {
        "flag_stop": frame[1],
        "forward_mps": _signed_int16(frame[2], frame[3]) / 1000.0,
        "lateral_mps": _signed_int16(frame[4], frame[5]) / 1000.0,
        "yaw_radps": _signed_int16(frame[6], frame[7]) / 1000.0,
        "supply_voltage_v": ((frame[20] << 8) | frame[21]) / 1000.0,
        "raw_hex": frame.hex(),
    }


def _validate_common_record_headers(records):
    schemas = set()
    counts = Counter()
    for line_number, record in records:
        schema = _string(record, "schema", line_number)
        record_type = _string(record, "record_type", line_number)
        if schema not in SUPPORTED_SCHEMAS:
            _error(line_number, "unsupported schema '{}'".format(schema))
        schemas.add(schema)
        allowed = (
            BENCH_RECORD_TYPES if schema == BENCH_SCHEMA
            else RAW_PROFILE_RECORD_TYPES
        )
        if record_type not in allowed:
            _error(line_number, "unsupported record_type '{}' for {}".format(
                record_type, schema
            ))
        counts[record_type] += 1
    if len(schemas) != 1:
        raise AnalysisError("mixed input schemas are forbidden")
    schema = next(iter(schemas))
    if counts["metadata"] != 1:
        raise AnalysisError("exactly one metadata record is required")
    if counts["summary"] != 1:
        raise AnalysisError("exactly one summary record is required")
    if records[-1][1]["record_type"] != "summary":
        raise AnalysisError("summary must be the final record")
    return schema, counts


def _validate_raw_cli_prefix(schema, records, counts):
    """Validate the physical-CLI prefix when it is present.

    Core/injected fixtures intentionally start at ``metadata``.  A physical
    CLI record, however, must carry the complete preflight/open/startup
    zero-only-recovery trio in the exact order emitted before the active
    session begins.  Historic v1 exact-zero-only prefixes remain readable.
    """
    if schema != RAW_PROFILE_SCHEMA:
        return
    prefix_types = (
        "cli_preflight",
        "open_result",
        "startup_exact_zero_result",
    )
    if not any(counts[record_type] for record_type in prefix_types):
        return
    for record_type in prefix_types:
        if counts[record_type] != 1:
            raise AnalysisError(
                "physical raw-profile evidence requires exactly one '{}'"
                .format(record_type)
            )

    metadata_index = next(
        index
        for index, (_, record) in enumerate(records)
        if record["record_type"] == "metadata"
    )
    leading_types = [
        record["record_type"] for _, record in records[:metadata_index]
    ]
    if leading_types != list(prefix_types):
        raise AnalysisError(
            "physical raw-profile prefix must be "
            "cli_preflight -> open_result -> startup_exact_zero_result -> metadata"
        )

    cli_line, cli = records[0]
    open_line, opened = records[1]
    zero_line, zero = records[2]
    for key in (
        "four_gates_confirmed",
        "raw_raised_bench_opt_in",
        "operator_confirmation_token_matched",
    ):
        if not _boolean(cli, key, cli_line):
            _error(cli_line, "'{}' must be true".format(key))
    if _string(opened, "status", open_line) != "ok":
        _error(open_line, "physical open_result must be ok")
    if _integer(opened, "os_error", open_line, 0) != 0:
        _error(open_line, "successful physical open_result must have os_error 0")
    attempts = _integer(zero, "attempts", zero_line, 1)
    if attempts > 3:
        _error(zero_line, "startup exact-zero attempts exceeds bounded maximum")
    if not _boolean(zero, "zero_host_write_completed", zero_line):
        _error(zero_line, "startup exact-zero host write must be complete")
    if _boolean(zero, "delivery_unconfirmed", zero_line):
        _error(zero_line, "startup exact-zero delivery may not be unconfirmed")
    if _string(zero, "terminal_transport_status", zero_line) != "ok":
        _error(zero_line, "startup exact-zero terminal transport status must be ok")
    if not _boolean(zero, "exact_zero_frame", zero_line):
        _error(zero_line, "startup record must identify an exact-zero frame")
    if _boolean(zero, "vcu_acknowledgement", zero_line):
        _error(zero_line, "startup host write may not claim a VCU acknowledgement")
    if _string(zero, "recording_order", zero_line) != (
        "all_zero_attempts_completed_before_this_record"
    ):
        _error(zero_line, "startup exact-zero recording order is unsupported")

    parser_recovery_fields = (
        "startup_sequence",
        "parser_resync_padding_bytes",
        "parser_resync_padding_host_write_completed",
        "recovery_restarts",
        "activation_status",
        "write_stream_poisoned",
    )
    parser_recovery_field_count = sum(
        key in zero for key in parser_recovery_fields
    )
    if parser_recovery_field_count not in (0, len(parser_recovery_fields)):
        _error(
            zero_line,
            "startup parser-recovery fields must be complete or absent "
            "for legacy evidence",
        )
    if parser_recovery_field_count:
        if _string(zero, "startup_sequence", zero_line) != (
            "ten_zero_padding_then_exact_zero"
        ):
            _error(zero_line, "startup parser-recovery sequence is unsupported")
        if _integer(zero, "parser_resync_padding_bytes", zero_line, 0) != 10:
            _error(
                zero_line,
                "startup recovery requires exactly 10 zero padding bytes",
            )
        if not _boolean(
            zero,
            "parser_resync_padding_host_write_completed",
            zero_line,
        ):
            _error(
                zero_line,
                "startup parser-resync padding host write must be complete",
            )
        recovery_restarts = _integer(
            zero, "recovery_restarts", zero_line, 0
        )
        if recovery_restarts >= attempts:
            _error(zero_line, "startup recovery restart count is inconsistent")
        if _string(zero, "activation_status", zero_line) != "success":
            _error(zero_line, "startup parser recovery must report success")
        if _boolean(zero, "write_stream_poisoned", zero_line):
            _error(zero_line, "startup recovery may not be poisoned")

    profile_ids = (
        _string(cli, "profile_id", cli_line),
        _string(opened, "profile_id", open_line),
        _string(zero, "profile_id", zero_line),
        _string(records[metadata_index][1], "profile_id",
                records[metadata_index][0]),
    )
    if len(set(profile_ids)) != 1:
        raise AnalysisError("physical raw-profile prefix profile_id values disagree")


def _validate_metadata(schema, line_number, metadata):
    if _string(metadata, "protocol_status", line_number) != "UNVERIFIED":
        _error(line_number, "protocol_status must remain UNVERIFIED")
    if "vcu_ack_available" in metadata and _boolean(
        metadata, "vcu_ack_available", line_number
    ):
        _error(line_number, "metadata may not claim a VCU ACK")
    if "successful_write_semantics" in metadata and _string(
        metadata, "successful_write_semantics", line_number
    ) != "host_os_full_write_only":
        _error(line_number, "successful write semantics must be host-only")
    if "rear_track_m" in metadata:
        track = _number(metadata, "rear_track_m", line_number, 0.0)
        if not _almost_equal(track, REAR_TRACK_M):
            _error(line_number, "rear_track_m conflicts with the 0.322 m analysis basis")
    if "raised_wheel_start_rear_track_m" in metadata:
        track = _number(
            metadata, "raised_wheel_start_rear_track_m", line_number, 0.0
        )
        if not _almost_equal(track, REAR_TRACK_M):
            _error(line_number, "raised-wheel rear track conflicts with 0.322 m")
    if "derived_rear_track_m" in metadata:
        track = _number(metadata, "derived_rear_track_m", line_number, 0.0)
        if not _almost_equal(track, REAR_TRACK_M):
            _error(line_number, "derived rear track conflicts with 0.322 m")

    if schema == BENCH_SCHEMA:
        mode = _string(metadata, "mode", line_number)
        if mode not in {"straight-ramp", "exact-zero"}:
            _error(line_number, "unsupported bench mode")
        target = _number(metadata, "target_speed_mps", line_number, 0.0)
        hold_ns = _integer(metadata, "hold_duration_ns", line_number, 0)
        sustained_frames = metadata.get(
            "fresh_feedback_receipts_required", DEFAULT_SUSTAINED_FRAMES
        )
        if isinstance(sustained_frames, bool) or not isinstance(
            sustained_frames, int
        ) or sustained_frames <= 0:
            _error(line_number, "fresh_feedback_receipts_required is invalid")
        return {
            "profile_id": None,
            "mode": mode,
            "target_forward_mps": target,
            "target_curvature_inv_m": 0.0,
            "target_yaw_radps": 0.0,
            "turn_radius_m": None,
            "hold_duration_ns": hold_ns,
            "command_acceleration_mps2": _optional_number(
                metadata, "fixed_acceleration_mps2", line_number
            ),
            "command_cap_mps": _optional_number(
                metadata, "hard_max_forward_speed_mps", line_number
            ),
            "sustained_frames": sustained_frames,
        }

    profile_id = _string(metadata, "profile_id", line_number)
    target = _number(metadata, "target_forward_mps", line_number, 0.0)
    curvature = _number(metadata, "curvature_inv_m", line_number)
    turn_radius = _optional_number(metadata, "turn_radius_m", line_number)
    if curvature == 0.0 and turn_radius is not None:
        _error(line_number, "straight profile turn_radius_m must be null")
    if curvature != 0.0:
        expected_radius = 1.0 / abs(curvature)
        if turn_radius is None or not _almost_equal(turn_radius, expected_radius, 1.0e-9):
            _error(line_number, "turn_radius_m is inconsistent with curvature")
    hold_ns = _integer(metadata, "hold_duration_ns", line_number, 1)
    acceleration = _number(
        metadata, "command_acceleration_mps2", line_number, 0.0
    )
    if acceleration <= 0.0:
        _error(line_number, "command_acceleration_mps2 must be positive")
    command_cap = _number(metadata, "command_cap_mps", line_number, 0.0)
    if target > command_cap:
        _error(line_number, "target_forward_mps exceeds command_cap_mps")
    for key in (
        "feedback_tracking_acceptance_gate",
        "feedback_asymmetry_acceptance_gate",
        "feedback_standstill_acceptance_gate",
        "phase1_codec_used",
        "installed",
    ):
        if _boolean(metadata, key, line_number):
            _error(line_number, "{} must be false for raw profile capture".format(key))
    if _string(metadata, "left_right_difference_sign", line_number) != "left_minus_right":
        _error(line_number, "raw-profile left/right difference sign is unsupported")
    extended_fields = (
        "speed_tier_semantics",
        "speed_tier_wire",
        "speed_tier_mps",
        "target_center_forward_wire",
        "target_yaw_wire",
        "target_yaw_radps",
        "target_outer_command_mps",
        "target_inner_command_mps",
        "derived_target_left_mps",
        "derived_target_right_mps",
        "outer_tier_slack_mps",
        "turn_direction",
        "turn_radius_mm",
        "derived_rear_track_mm",
    )
    present_extended = [key for key in extended_fields if key in metadata]
    extended_target_contract = bool(present_extended)
    extended = {}
    if extended_target_contract:
        if len(present_extended) != len(extended_fields):
            _error(
                line_number,
                "extended raw-profile target metadata must be complete",
            )
        semantics = _string(metadata, "speed_tier_semantics", line_number)
        allowed_semantics = {
            "straight_center_equals_rear_wheels",
            "legacy_center_forward_command",
            "outer_rear_wheel_command_upper_limit",
        }
        if semantics not in allowed_semantics:
            _error(line_number, "unsupported speed_tier_semantics")
        tier_wire = _integer(metadata, "speed_tier_wire", line_number, 1)
        tier_mps = _number(metadata, "speed_tier_mps", line_number, 0.0)
        if not _almost_equal(tier_mps, tier_wire / 1000.0):
            _error(line_number, "speed tier wire/mps disagree")
        center_wire = _integer(
            metadata, "target_center_forward_wire", line_number, 1
        )
        if not _almost_equal(target, center_wire / 1000.0):
            _error(line_number, "target center wire disagrees with target_forward_mps")
        yaw_wire = _integer(metadata, "target_yaw_wire", line_number)
        if yaw_wire < -32768 or yaw_wire > 32767:
            _error(line_number, "target_yaw_wire is outside int16 range")
        yaw_radps = _number(metadata, "target_yaw_radps", line_number)
        if not _almost_equal(yaw_radps, yaw_wire / 1000.0):
            _error(line_number, "target yaw wire disagrees with target_yaw_radps")
        radius_mm = _optional_integer(
            metadata, "turn_radius_mm", line_number, 1
        )
        direction = _string(metadata, "turn_direction", line_number)
        if curvature == 0.0:
            if (
                radius_mm is not None
                or yaw_wire != 0
                or direction != "straight"
                or semantics != "straight_center_equals_rear_wheels"
            ):
                _error(line_number, "straight target integer geometry is inconsistent")
            expected_yaw_wire = 0
        else:
            expected_direction = "left" if curvature > 0.0 else "right"
            direction_sign = 1 if curvature > 0.0 else -1
            if (
                radius_mm is None
                or turn_radius is None
                or not _almost_equal(turn_radius, radius_mm / 1000.0, 1.0e-12)
                or not _almost_equal(
                    curvature,
                    direction_sign * 1000.0 / radius_mm,
                    1.0e-12,
                )
                or direction != expected_direction
                or semantics == "straight_center_equals_rear_wheels"
            ):
                _error(line_number, "turn radius/direction integer geometry is inconsistent")
            expected_yaw_wire = direction_sign * (
                center_wire * 1000 // radius_mm
            )
        if yaw_wire != expected_yaw_wire:
            _error(line_number, "target yaw wire disagrees with integer radius rule")
        if semantics in {
            "straight_center_equals_rear_wheels",
            "legacy_center_forward_command",
        } and center_wire != tier_wire:
            _error(line_number, "center-command speed tier must equal target center")
        track_mm = _integer(metadata, "derived_rear_track_mm", line_number, 1)
        if track_mm != 322:
            _error(line_number, "derived_rear_track_mm conflicts with 0.322 m")
        yaw_magnitude = abs(yaw_wire)
        outer_wire_milli = center_wire * 1000 + yaw_magnitude * 161
        inner_wire_milli = center_wire * 1000 - yaw_magnitude * 161
        if inner_wire_milli < 0:
            _error(line_number, "target inner command is negative")
        outer_mps = _number(
            metadata, "target_outer_command_mps", line_number, 0.0
        )
        inner_mps = _number(
            metadata, "target_inner_command_mps", line_number, 0.0
        )
        if (
            not _almost_equal(outer_mps, outer_wire_milli / 1000000.0, 1.0e-12)
            or not _almost_equal(inner_mps, inner_wire_milli / 1000000.0, 1.0e-12)
        ):
            _error(line_number, "target inner/outer geometry disagrees with integer wires")
        if (
            semantics == "outer_rear_wheel_command_upper_limit"
            and outer_wire_milli > tier_wire * 1000
        ):
            _error(line_number, "outer command exceeds its speed tier")
        if semantics == "outer_rear_wheel_command_upper_limit":
            next_center = center_wire + 1
            next_yaw_magnitude = next_center * 1000 // radius_mm
            next_outer_wire_milli = (
                next_center * 1000 + next_yaw_magnitude * 161
            )
            if next_center <= tier_wire and next_outer_wire_milli <= tier_wire * 1000:
                _error(line_number, "outer-tier center wire is not maximal")
        left_mps = target - yaw_radps * REAR_TRACK_M / 2.0
        right_mps = target + yaw_radps * REAR_TRACK_M / 2.0
        explicit_left = _number(
            metadata, "derived_target_left_mps", line_number
        )
        explicit_right = _number(
            metadata, "derived_target_right_mps", line_number
        )
        if (
            not _almost_equal(explicit_left, left_mps, 1.0e-12)
            or not _almost_equal(explicit_right, right_mps, 1.0e-12)
        ):
            _error(line_number, "derived target left/right geometry disagrees")
        slack = _optional_number(
            metadata, "outer_tier_slack_mps", line_number
        )
        if semantics == "outer_rear_wheel_command_upper_limit":
            if slack is None or not _almost_equal(
                slack, tier_mps - outer_mps, 1.0e-12
            ):
                _error(line_number, "outer-tier slack disagrees with tier geometry")
        elif slack is not None:
            _error(line_number, "non-outer speed tier must have null outer slack")
        extended = {
            "speed_tier_semantics": semantics,
            "speed_tier_wire": tier_wire,
            "speed_tier_mps": tier_mps,
            "target_center_forward_wire": center_wire,
            "target_yaw_wire": yaw_wire,
            "target_yaw_radps": yaw_radps,
            "target_outer_command_mps": outer_mps,
            "target_inner_command_mps": inner_mps,
            "derived_target_left_mps": explicit_left,
            "derived_target_right_mps": explicit_right,
            "outer_tier_slack_mps": slack,
            "turn_direction": direction,
            "turn_radius_mm": radius_mm,
        }
    return {
        "profile_id": profile_id,
        "mode": "raw-profile",
        "target_forward_mps": target,
        "target_curvature_inv_m": curvature,
        "target_yaw_radps": (
            extended["target_yaw_radps"]
            if extended_target_contract
            else target * curvature
        ),
        "target_yaw_wire": (
            extended["target_yaw_wire"] if extended_target_contract else None
        ),
        "turn_radius_m": turn_radius,
        "turn_radius_mm": (
            extended["turn_radius_mm"] if extended_target_contract else None
        ),
        "hold_duration_ns": hold_ns,
        "command_acceleration_mps2": acceleration,
        "command_cap_mps": command_cap,
        "sustained_frames": DEFAULT_SUSTAINED_FRAMES,
        "extended_target_contract": extended_target_contract,
        "extended_target": extended,
    }


def _expected_raw_profile_yaw_wire(profile, forward_wire):
    radius_mm = profile.get("turn_radius_mm")
    if profile.get("extended_target_contract"):
        if radius_mm is None:
            return 0
        direction_sign = 1 if profile["target_yaw_wire"] > 0 else -1
        return direction_sign * (forward_wire * 1000 // radius_mm)
    return math.trunc(forward_wire * profile["target_curvature_inv_m"])


def _physical_phase_and_profile(record, line_number, profile):
    phase = _string(record, "phase", line_number)
    if phase not in PHASE_RANK:
        _error(line_number, "invalid physical raw-profile phase")
    if _string(record, "profile_id", line_number) != profile["profile_id"]:
        _error(line_number, "physical raw-profile profile_id mismatch")
    return phase


def _physical_io_attempt(record, line_number, profile):
    phase = _physical_phase_and_profile(record, line_number, profile)
    if _string(record, "clock", line_number) != "CLOCK_MONOTONIC":
        _error(line_number, "physical IO attempt clock must be CLOCK_MONOTONIC")
    before_ns = _integer(record, "before_monotonic_ns", line_number, 1)
    deadline_ns = _integer(
        record, "absolute_deadline_monotonic_ns", line_number, 1
    )
    if deadline_ns < before_ns:
        _error(line_number, "physical IO attempt deadline precedes attempt")
    requested = _integer(record, "requested_bytes", line_number, 1)
    return {
        "phase": phase,
        "before_ns": before_ns,
        "deadline_ns": deadline_ns,
        "requested": requested,
    }


def _physical_io_result(record, line_number, profile):
    phase = _physical_phase_and_profile(record, line_number, profile)
    if _string(record, "clock", line_number) != "CLOCK_MONOTONIC":
        _error(line_number, "physical IO result clock must be CLOCK_MONOTONIC")
    before_ns = _integer(record, "before_monotonic_ns", line_number, 1)
    after_ns = _integer(record, "after_monotonic_ns", line_number, 1)
    deadline_ns = _integer(
        record, "absolute_deadline_monotonic_ns", line_number, 1
    )
    if after_ns < before_ns:
        _error(line_number, "physical IO result completion precedes attempt")
    if deadline_ns < before_ns:
        _error(line_number, "physical IO result deadline precedes attempt")
    status = _string(record, "status", line_number)
    if status not in TRANSPORT_STATUSES:
        _error(line_number, "unsupported physical transport status")
    transferred = _integer(record, "transferred", line_number, 0)
    os_error = _integer(record, "os_error", line_number, 0)
    delivery_unconfirmed = _boolean(
        record, "delivery_unconfirmed", line_number
    )
    if status == "ok" and os_error != 0:
        _error(line_number, "successful physical IO result must have os_error 0")
    return {
        "phase": phase,
        "before_ns": before_ns,
        "after_ns": after_ns,
        "deadline_ns": deadline_ns,
        "status": status,
        "transferred": transferred,
        "delivery_unconfirmed": delivery_unconfirmed,
    }


def _physical_command_fields(record, line_number):
    decoded = _decode_command(record, line_number)
    forward_wire = _integer(record, "forward_wire", line_number)
    yaw_wire = _integer(record, "yaw_wire", line_number)
    forward_mps = _number(record, "forward_command_mps", line_number)
    yaw_radps = _number(record, "yaw_command_radps", line_number)
    if (
        forward_wire != decoded["forward_wire"]
        or yaw_wire != decoded["yaw_wire"]
        or not _almost_equal(forward_mps, decoded["forward_mps"])
        or not _almost_equal(yaw_radps, decoded["yaw_radps"])
    ):
        _error(line_number, "physical command explicit fields disagree with raw frame")
    return decoded


def _physical_command_pair(
    records, index, expected_result_type, profile, exact_zero=False
):
    attempt_line, attempt_record = records[index]
    label = "emergency zero" if exact_zero else "normal TX"
    if index + 1 >= len(records):
        _error(attempt_line, "{} attempt has no result".format(label))
    result_line, result_record = records[index + 1]
    if result_record["record_type"] != expected_result_type:
        _error(
            attempt_line,
            "{} attempt must be followed immediately by its result".format(label),
        )
    attempt = _physical_io_attempt(attempt_record, attempt_line, profile)
    result = _physical_io_result(result_record, result_line, profile)
    attempt_command = _physical_command_fields(attempt_record, attempt_line)
    result_command = _physical_command_fields(result_record, result_line)
    if attempt["requested"] != 11:
        _error(attempt_line, "{} attempt must request 11 bytes".format(label))
    if result["transferred"] > attempt["requested"]:
        _error(result_line, "{} result transferred too many bytes".format(label))
    pair_fields_match = (
        attempt["phase"] == result["phase"]
        and attempt["before_ns"] == result["before_ns"]
        and attempt["deadline_ns"] == result["deadline_ns"]
        and attempt_command == result_command
    )
    if not pair_fields_match:
        _error(
            result_line,
            "{} attempt/result fields disagree".format(label),
        )
    if result["status"] == "ok" and (
        result["transferred"] != 11 or result["delivery_unconfirmed"]
    ):
        _error(result_line, "status ok requires a complete confirmed command write")
    successful = (
        result["status"] == "ok"
        and result["transferred"] == 11
        and not result["delivery_unconfirmed"]
    )
    if exact_zero:
        if (
            attempt_command["forward_wire"] != 0
            or attempt_command["yaw_wire"] != 0
        ):
            _error(attempt_line, "emergency zero command must be exact zero")
    else:
        expected_yaw_wire = _expected_raw_profile_yaw_wire(
            profile, attempt_command["forward_wire"]
        )
        if (
            attempt_command["forward_wire"] < 0
            or attempt_command["forward_mps"] > profile["target_forward_mps"]
            or attempt_command["yaw_wire"] != expected_yaw_wire
        ):
            _error(attempt_line, "normal TX attempt violates its raw profile")
    return {
        "successful": successful,
        "forward_wire": attempt_command["forward_wire"],
        "yaw_wire": attempt_command["yaw_wire"],
    }


def _physical_read_pair(records, index, result_type, profile, label):
    attempt_line, attempt_record = records[index]
    if index + 1 >= len(records):
        _error(attempt_line, "{} attempt has no result".format(label))
    result_line, result_record = records[index + 1]
    if result_record["record_type"] != result_type:
        _error(
            attempt_line,
            "{} attempt must be followed immediately by its result".format(label),
        )
    attempt = _physical_io_attempt(attempt_record, attempt_line, profile)
    result = _physical_io_result(result_record, result_line, profile)
    if (
        attempt["phase"] != result["phase"]
        or attempt["before_ns"] != result["before_ns"]
        or attempt["deadline_ns"] != result["deadline_ns"]
    ):
        _error(result_line, "{} attempt/result fields disagree".format(label))
    if result["transferred"] > attempt["requested"]:
        _error(result_line, "{} result transferred too many bytes".format(label))
    if result["delivery_unconfirmed"]:
        _error(result_line, "read result may not claim command delivery uncertainty")
    if result["status"] == "ok" and result["transferred"] == 0:
        _error(result_line, "successful read must transfer at least one byte")
    if (
        result["status"] in READ_TIMEOUT_STATUSES
        and result["transferred"] != 0
    ):
        _error(result_line, "timed-out read must transfer zero bytes")
    return attempt, result


def _physical_raw_chunk(record, line_number, profile, read_result):
    if _string(record, "receipt_clock", line_number) != "CLOCK_MONOTONIC":
        _error(line_number, "raw RX chunk clock must be CLOCK_MONOTONIC")
    receipt_ns = _integer(record, "receipt_monotonic_ns", line_number, 1)
    phase = _physical_phase_and_profile(record, line_number, profile)
    transferred = _integer(record, "transferred", line_number, 1)
    raw_hex = _string(record, "raw_hex", line_number)
    if len(raw_hex) != transferred * 2 or any(
        byte not in "0123456789abcdefABCDEF" for byte in raw_hex
    ):
        _error(line_number, "raw RX chunk length disagrees with transferred bytes")
    try:
        bytes.fromhex(raw_hex)
    except ValueError:
        _error(line_number, "raw RX chunk contains invalid hexadecimal data")
    if (
        receipt_ns != read_result["after_ns"]
        or phase != read_result["phase"]
        or transferred != read_result["transferred"]
    ):
        _error(line_number, "raw RX chunk disagrees with RX result")
    return {"receipt_ns": receipt_ns, "phase": phase, "transferred": transferred}


def _physical_parser_observation(
    record, line_number, profile, chunk, previous_totals
):
    receipt_ns = _integer(record, "receipt_monotonic_ns", line_number, 1)
    phase = _physical_phase_and_profile(record, line_number, profile)
    if receipt_ns != chunk["receipt_ns"] or phase != chunk["phase"]:
        _error(line_number, "parser observation disagrees with raw RX chunk")
    if _boolean(record, "online_acceptance_gate_applied", line_number):
        _error(line_number, "physical parser observation applied an acceptance gate")
    next_totals = {}
    for name in ("checksum_failures", "framing_failures", "discarded_bytes"):
        delta = _integer(record, name + "_delta", line_number, 0)
        total = _integer(record, name + "_total", line_number, 0)
        if total != previous_totals[name] + delta:
            _error(line_number, "parser delta/total fields are inconsistent")
        next_totals[name] = total
    trailing = _integer(record, "trailing_buffered_bytes", line_number, 0)
    return next_totals, trailing


def _validate_physical_raw_summary(summary, line_number, ledger):
    statistics = _required(summary, "statistics", line_number)
    if not isinstance(statistics, dict):
        _error(line_number, "summary statistics must be an object")
    expected = {
        "read_calls": (ledger["read_calls"], "summary read_calls disagrees with records"),
        "read_timeouts": (
            ledger["read_timeouts"],
            "summary read_timeouts disagrees with RX results",
        ),
        "raw_rx_bytes": (
            ledger["raw_rx_bytes"],
            "summary raw_rx_bytes disagrees with raw RX chunks",
        ),
        "valid_feedback_frames": (
            ledger["feedback_events"],
            "summary feedback count disagrees with records",
        ),
        "feedback_observation_events": (
            ledger["feedback_events"],
            "summary feedback event count disagrees with records",
        ),
        "normal_tx_host_writes_completed": (
            ledger["normal_completed"],
            "summary normal TX count disagrees with records",
        ),
        "zero_frame_host_writes_completed": (
            ledger["zero_completed"],
            "summary zero-frame TX count disagrees with records",
        ),
        "nonzero_frame_host_writes_completed": (
            ledger["nonzero_completed"],
            "summary nonzero-frame TX count disagrees with records",
        ),
        "maximum_commanded_forward_wire": (
            ledger["maximum_forward_wire"],
            "summary maximum command disagrees with completed normal TX records",
        ),
        "tx_attempts": (
            ledger["tx_attempts"],
            "summary TX attempts disagrees with attempt records",
        ),
        "tx_host_writes_completed": (
            ledger["tx_completed"],
            "summary completed TX count disagrees with result records",
        ),
    }
    for key, (actual, message) in expected.items():
        if _integer(statistics, key, line_number, 0) != actual:
            _error(line_number, message)
    parser_expected = {
        "parser_checksum_failures": ledger["parser_totals"]["checksum_failures"],
        "parser_framing_failures": ledger["parser_totals"]["framing_failures"],
        "parser_discarded_bytes": ledger["parser_totals"]["discarded_bytes"],
        "parser_trailing_buffered_bytes": ledger["parser_trailing"],
    }
    for key, actual in parser_expected.items():
        if _integer(statistics, key, line_number, 0) != actual:
            _error(line_number, "summary parser statistics disagree with parser events")
    completed = _boolean(summary, "zero_host_write_completed", line_number)
    if completed != ledger["emergency_zero_completed"]:
        _error(line_number, "final emergency-zero completion disagrees with records")


def _validate_physical_raw_event_stream(records, counts, profile):
    """Reconstruct physical session counters from the ordered event stream."""
    if counts["cli_preflight"] != 1:
        return None
    metadata_index = next(
        index
        for index, (_, record) in enumerate(records)
        if record["record_type"] == "metadata"
    )
    ledger = {
        "read_calls": 0,
        "read_timeouts": 0,
        "raw_rx_bytes": 0,
        "feedback_events": 0,
        "normal_completed": 0,
        "zero_completed": 0,
        "nonzero_completed": 0,
        "maximum_forward_wire": 0,
        "tx_attempts": 0,
        "tx_completed": 0,
        "parser_totals": {
            "checksum_failures": 0,
            "framing_failures": 0,
            "discarded_bytes": 0,
        },
        "parser_trailing": 0,
        "emergency_zero_completed": False,
    }
    index = metadata_index + 1
    emergency_started = False
    emergency_attempts = 0
    while index < len(records) - 1:
        line_number, record = records[index]
        record_type = record["record_type"]
        if emergency_started and record_type != (
            "emergency_zero_tx_attempt_postwrite_record"
        ):
            _error(
                line_number,
                "only emergency-zero pairs may follow the first emergency-zero attempt",
            )
        if record_type == "normal_tx_attempt":
            pair = _physical_command_pair(
                records, index, "normal_tx_result", profile
            )
            ledger["tx_attempts"] += 1
            if pair["successful"]:
                ledger["tx_completed"] += 1
                ledger["normal_completed"] += 1
                if pair["forward_wire"] == 0 and pair["yaw_wire"] == 0:
                    ledger["zero_completed"] += 1
                else:
                    ledger["nonzero_completed"] += 1
                    ledger["maximum_forward_wire"] = max(
                        ledger["maximum_forward_wire"], pair["forward_wire"]
                    )
            index += 2
            continue
        if record_type in ("drain_rx_attempt", "rx_attempt"):
            is_drain = record_type == "drain_rx_attempt"
            result_type = "drain_rx_result" if is_drain else "rx_result"
            label = "drain RX" if is_drain else "RX"
            _, read_result = _physical_read_pair(
                records, index, result_type, profile, label
            )
            ledger["read_calls"] += 1
            if not is_drain and read_result["status"] in READ_TIMEOUT_STATUSES:
                ledger["read_timeouts"] += 1
            index += 2
            if read_result["status"] != "ok":
                continue
            if index >= len(records) - 1 or records[index][1]["record_type"] != "raw_rx_chunk":
                _error(
                    records[index - 1][0],
                    "successful {} result must be followed by a raw RX chunk".format(label),
                )
            chunk_line, chunk_record = records[index]
            chunk = _physical_raw_chunk(
                chunk_record, chunk_line, profile, read_result
            )
            ledger["raw_rx_bytes"] += chunk["transferred"]
            index += 1
            if is_drain:
                continue
            if (
                index >= len(records) - 1
                or records[index][1]["record_type"]
                != "parser_observation_event"
            ):
                _error(
                    chunk_line,
                    "raw RX chunk must be followed by parser observation",
                )
            parser_line, parser_record = records[index]
            parser_totals, parser_trailing = _physical_parser_observation(
                parser_record,
                parser_line,
                profile,
                chunk,
                ledger["parser_totals"],
            )
            ledger["parser_totals"] = parser_totals
            ledger["parser_trailing"] = parser_trailing
            index += 1
            while (
                index < len(records) - 1
                and records[index][1]["record_type"]
                == "feedback_observation_event"
            ):
                feedback_line, feedback_record = records[index]
                if (
                    _string(feedback_record, "receipt_clock", feedback_line)
                    != "CLOCK_MONOTONIC"
                    or _integer(
                        feedback_record,
                        "receipt_monotonic_ns",
                        feedback_line,
                        1,
                    )
                    != chunk["receipt_ns"]
                    or _physical_phase_and_profile(
                        feedback_record, feedback_line, profile
                    )
                    != chunk["phase"]
                ):
                    _error(
                        feedback_line,
                        "feedback observation disagrees with its parser event",
                    )
                ledger["feedback_events"] += 1
                index += 1
            continue
        if record_type == "emergency_zero_tx_attempt_postwrite_record":
            emergency_started = True
            emergency_attempts += 1
            if emergency_attempts > 3:
                _error(line_number, "emergency-zero attempt count exceeds maximum")
            if ledger["emergency_zero_completed"]:
                _error(line_number, "emergency-zero attempt follows a successful zero")
            pair = _physical_command_pair(
                records,
                index,
                "emergency_zero_tx_result_postwrite_record",
                profile,
                exact_zero=True,
            )
            ledger["tx_attempts"] += 1
            if pair["successful"]:
                ledger["tx_completed"] += 1
                ledger["zero_completed"] += 1
                ledger["emergency_zero_completed"] = True
            index += 2
            continue
        _error(
            line_number,
            "unexpected unpaired physical raw-profile event '{}'".format(
                record_type
            ),
        )
    if emergency_attempts == 0:
        raise AnalysisError("physical raw-profile evidence lacks final emergency zero")
    summary_line, summary = records[-1]
    _validate_physical_raw_summary(summary, summary_line, ledger)
    return ledger


def _successful_normal_tx(schema, line_number, record, profile):
    if _string(record, "clock", line_number) != "CLOCK_MONOTONIC":
        _error(line_number, "normal TX clock must be CLOCK_MONOTONIC")
    before_ns = _integer(record, "before_monotonic_ns", line_number, 1)
    after_ns = _integer(record, "after_monotonic_ns", line_number, 1)
    if after_ns < before_ns:
        _error(line_number, "normal TX completion precedes its attempt")
    status = _string(record, "status", line_number)
    transferred = _integer(record, "transferred", line_number, 0)
    delivery_unconfirmed = _boolean(record, "delivery_unconfirmed", line_number)
    decoded = _decode_command(record, line_number)

    if schema == RAW_PROFILE_SCHEMA:
        phase = _string(record, "phase", line_number)
        if phase not in PHASE_RANK:
            _error(line_number, "invalid profile phase")
        if _string(record, "profile_id", line_number) != profile["profile_id"]:
            _error(line_number, "normal TX profile_id mismatch")
        explicit_forward_wire = _integer(record, "forward_wire", line_number)
        explicit_yaw_wire = _integer(record, "yaw_wire", line_number)
        explicit_forward = _number(record, "forward_command_mps", line_number)
        explicit_yaw = _number(record, "yaw_command_radps", line_number)
        if (
            explicit_forward_wire != decoded["forward_wire"]
            or explicit_yaw_wire != decoded["yaw_wire"]
            or not _almost_equal(explicit_forward, decoded["forward_mps"])
            or not _almost_equal(explicit_yaw, decoded["yaw_radps"])
        ):
            _error(line_number, "normal TX explicit fields disagree with raw frame")
        expected_yaw_wire = _expected_raw_profile_yaw_wire(
            profile, decoded["forward_wire"]
        )
        if decoded["forward_wire"] < 0 or (
            decoded["forward_mps"] > profile["target_forward_mps"]
        ) or decoded["yaw_wire"] != expected_yaw_wire:
            _error(line_number, "normal TX violates its raw profile definition")
    else:
        phase = None
        if decoded["forward_wire"] < 0 or decoded["yaw_wire"] != 0:
            _error(line_number, "bench straight-ramp TX must be forward with zero yaw")
        if (
            profile["command_cap_mps"] is not None
            and decoded["forward_mps"] > profile["command_cap_mps"]
        ):
            _error(line_number, "bench TX exceeds its recorded command cap")

    successful = status == "ok" and transferred == 11 and not delivery_unconfirmed
    if status == "ok" and transferred != 11:
        _error(line_number, "status ok requires a complete 11-byte normal TX")
    result = dict(decoded)
    result.update(
        {
            "line": line_number,
            "before_ns": before_ns,
            "after_ns": after_ns,
            "phase": phase,
            "successful": successful,
            "status": status,
            "transferred": transferred,
            "delivery_unconfirmed": delivery_unconfirmed,
        }
    )
    return result


def _feedback_record(schema, line_number, record, profile):
    if _string(record, "receipt_clock", line_number) != "CLOCK_MONOTONIC":
        _error(line_number, "feedback receipt clock must be CLOCK_MONOTONIC")
    receipt_ns = _integer(record, "receipt_monotonic_ns", line_number, 1)
    decoded = _decode_feedback(record, line_number)
    names = (
        ("forward_speed_mps", "forward_mps"),
        ("lateral_speed_mps", "lateral_mps"),
        ("yaw_rate_radps", "yaw_radps"),
    )
    for field_name, decoded_name in names:
        value = _number(record, field_name, line_number)
        if not _almost_equal(value, decoded[decoded_name]):
            _error(line_number, "{} disagrees with raw frame".format(field_name))
    flag_stop = _integer(record, "composite_stop_flag_raw", line_number, 0)
    if flag_stop not in (0, 1) or flag_stop != decoded["flag_stop"]:
        _error(line_number, "unsupported or inconsistent FlagStop value")
    control_allowed = _boolean(record, "control_allowed", line_number)
    control_inhibited = _boolean(record, "control_inhibited", line_number)
    if control_allowed != (flag_stop == 0) or control_inhibited != (flag_stop == 1):
        _error(line_number, "FlagStop/control booleans are inconsistent")
    if _boolean(record, "source_time_available", line_number):
        _error(line_number, "feedback may not claim a source timestamp")
    if "vcu_ack_available" in record and _boolean(
        record, "vcu_ack_available", line_number
    ):
        _error(line_number, "feedback may not claim a VCU ACK")

    half_track = REAR_TRACK_M / 2.0
    left_mps = decoded["forward_mps"] - decoded["yaw_radps"] * half_track
    right_mps = decoded["forward_mps"] + decoded["yaw_radps"] * half_track
    result = dict(decoded)
    result.update(
        {
            "line": line_number,
            "receipt_ns": receipt_ns,
            "left_mps": left_mps,
            "right_mps": right_mps,
            "flag_stop": flag_stop,
            "control_allowed": control_allowed,
            "control_inhibited": control_inhibited,
            "phase": None,
            "last_forward_wire": None,
            "last_yaw_wire": None,
        }
    )

    if "supply_voltage_v" in record:
        supply = _number(record, "supply_voltage_v", line_number)
        if not _almost_equal(supply, decoded["supply_voltage_v"]):
            _error(line_number, "supply_voltage_v disagrees with raw frame")

    if schema == RAW_PROFILE_SCHEMA:
        phase = _string(record, "phase", line_number)
        if phase not in PHASE_RANK:
            _error(line_number, "invalid feedback profile phase")
        if _string(record, "profile_id", line_number) != profile["profile_id"]:
            _error(line_number, "feedback profile_id mismatch")
        if _boolean(record, "online_acceptance_gate_applied", line_number):
            _error(line_number, "raw profile feedback applied an acceptance gate")
        last_forward_wire = _integer(
            record, "last_successful_forward_wire", line_number
        )
        last_yaw_wire = _integer(record, "last_successful_yaw_wire", line_number)
        explicit_left = _number(record, "derived_left_mps", line_number)
        explicit_right = _number(record, "derived_right_mps", line_number)
        explicit_difference = _number(
            record, "left_right_difference_mps", line_number
        )
        explicit_tracking_error = _number(record, "tracking_error_mps", line_number)
        post_zero_tail = _boolean(record, "post_zero_tail", line_number)
        if post_zero_tail != (phase == "post_stop"):
            _error(line_number, "post_zero_tail is inconsistent with feedback phase")
        if (
            not _almost_equal(explicit_left, left_mps, 1.0e-9)
            or not _almost_equal(explicit_right, right_mps, 1.0e-9)
            or not _almost_equal(explicit_difference, left_mps - right_mps, 1.0e-9)
            or not _almost_equal(
                explicit_tracking_error,
                decoded["forward_mps"] - last_forward_wire / 1000.0,
                1.0e-9,
            )
        ):
            _error(line_number, "derived raw-profile feedback fields are inconsistent")
        result["phase"] = phase
        result["last_forward_wire"] = last_forward_wire
        result["last_yaw_wire"] = last_yaw_wire
    return result


def _infer_bench_phases(transmissions, target_mps):
    if not transmissions:
        return
    first_motion = next(
        (
            index
            for index, item in enumerate(transmissions)
            if item["forward_wire"] != 0 or item["yaw_wire"] != 0
        ),
        None,
    )
    if first_motion is None:
        for item in transmissions:
            item["phase"] = "baseline"
        return

    first_post_zero = next(
        (
            index
            for index in range(first_motion + 1, len(transmissions))
            if transmissions[index]["forward_wire"] == 0
            and transmissions[index]["yaw_wire"] == 0
        ),
        len(transmissions),
    )
    motion_end = first_post_zero
    peak_wire = max(
        item["forward_wire"] for item in transmissions[first_motion:motion_end]
    )
    target_wire = int(math.trunc(target_mps * 1000.0))
    reached_target = target_wire > 0 and peak_wire >= target_wire
    first_peak = next(
        index
        for index in range(first_motion, motion_end)
        if transmissions[index]["forward_wire"] == peak_wire
    )
    first_decrease = next(
        (
            index
            for index in range(first_peak + 1, motion_end)
            if transmissions[index]["forward_wire"] < peak_wire
        ),
        motion_end,
    )

    for index, item in enumerate(transmissions):
        if index < first_motion:
            item["phase"] = "baseline"
        elif index >= first_post_zero:
            item["phase"] = "post_stop"
        elif reached_target and first_peak <= index < first_decrease:
            item["phase"] = "hold"
        elif index < (first_peak if reached_target else first_decrease):
            item["phase"] = "ramp_up"
        elif not reached_target and index <= first_peak:
            item["phase"] = "ramp_up"
        else:
            item["phase"] = "ramp_down"


def _validate_phase_order(transmissions):
    previous_rank = -1
    for item in transmissions:
        rank = PHASE_RANK[item["phase"]]
        if rank < previous_rank:
            _error(item["line"], "normal TX phase order regressed")
        previous_rank = rank
        if item["phase"] in {"baseline", "post_stop"} and (
            item["forward_wire"] != 0 or item["yaw_wire"] != 0
        ):
            _error(item["line"], "baseline/post_stop normal TX must be exact zero")


def _associate_feedback(feedback, transmissions, schema):
    completion_keys = [
        (item["after_ns"], item["line"]) for item in transmissions
    ]
    for frame in feedback:
        # CLOCK_MONOTONIC can repeat in injected tests and, in principle, on a
        # coarse host clock.  NDJSON record order breaks equal-time ties: a TX
        # result written later on the same tick cannot be associated with an
        # earlier feedback record.
        index = bisect.bisect_right(
            completion_keys, (frame["receipt_ns"], frame["line"])
        ) - 1
        associated = transmissions[index] if index >= 0 else None
        frame["associated_tx"] = associated
        if schema == BENCH_SCHEMA:
            frame["phase"] = associated["phase"] if associated else "baseline"
        elif associated is not None:
            if (
                frame["last_forward_wire"] != associated["forward_wire"]
                or frame["last_yaw_wire"] != associated["yaw_wire"]
            ):
                _error(
                    frame["line"],
                    "last-successful command fields disagree with TX timeline",
                )


def _percentile(sorted_values, probability):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    location = (len(sorted_values) - 1) * probability
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return sorted_values[lower]
    weight = location - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _statistics(values):
    values = list(values)
    if not values:
        return {"available": False, "count": 0}
    ordered = sorted(values)
    return {
        "available": True,
        "count": len(values),
        "min": ordered[0],
        "max": ordered[-1],
        "mean": math.fsum(values) / len(values),
        "p05": _percentile(ordered, 0.05),
        "p50": _percentile(ordered, 0.50),
        "p95": _percentile(ordered, 0.95),
        "p99": _percentile(ordered, 0.99),
    }


def _phase_summary(transmissions, feedback, basis):
    result = {"basis": basis, "intervals": {}}
    for phase in PHASES:
        selected = [item for item in transmissions if item["phase"] == phase]
        selected_feedback = [item for item in feedback if item["phase"] == phase]
        if not selected:
            result["intervals"][phase] = {
                "available": False,
                "write_count": 0,
                "feedback_frame_count": len(selected_feedback),
            }
            continue
        start_ns = selected[0]["after_ns"]
        write_end_ns = selected[-1]["after_ns"]
        next_items = [
            item for item in transmissions
            if item["after_ns"] > write_end_ns
            and PHASE_RANK[item["phase"]] > PHASE_RANK[phase]
        ]
        interval_end_ns = next_items[0]["after_ns"] if next_items else write_end_ns
        result["intervals"][phase] = {
            "available": True,
            "write_count": len(selected),
            "start_monotonic_ns": start_ns,
            "last_write_monotonic_ns": write_end_ns,
            "next_phase_start_monotonic_ns": (
                interval_end_ns if next_items else None
            ),
            "write_span_s": (write_end_ns - start_ns) / 1.0e9,
            "duration_to_next_phase_s": (
                (interval_end_ns - start_ns) / 1.0e9 if next_items else None
            ),
            "forward_command_mps": _statistics(
                item["forward_mps"] for item in selected
            ),
            "yaw_command_radps": _statistics(
                item["yaw_radps"] for item in selected
            ),
            "feedback_frame_count": len(selected_feedback),
            "first_feedback_receipt_monotonic_ns": (
                selected_feedback[0]["receipt_ns"] if selected_feedback else None
            ),
            "last_feedback_receipt_monotonic_ns": (
                selected_feedback[-1]["receipt_ns"] if selected_feedback else None
            ),
            "feedback_receipt_span_s": (
                (
                    selected_feedback[-1]["receipt_ns"]
                    - selected_feedback[0]["receipt_ns"]
                )
                / 1.0e9
                if selected_feedback
                else None
            ),
        }
    return result


def _feedback_statistics(feedback):
    return {
        "forward_mps": _statistics(item["forward_mps"] for item in feedback),
        "lateral_mps": _statistics(item["lateral_mps"] for item in feedback),
        "yaw_radps": _statistics(item["yaw_radps"] for item in feedback),
        "left_mps": _statistics(item["left_mps"] for item in feedback),
        "right_mps": _statistics(item["right_mps"] for item in feedback),
        "supply_voltage_v": _statistics(
            item["supply_voltage_v"] for item in feedback
        ),
    }


def _frame_timing(feedback):
    if not feedback:
        return {"count": 0, "available": False}
    receipts = [item["receipt_ns"] for item in feedback]
    intervals = [
        receipts[index] - receipts[index - 1]
        for index in range(1, len(receipts))
    ]
    positive = [value for value in intervals if value > 0]
    zero_count = sum(value == 0 for value in intervals)
    span_ns = receipts[-1] - receipts[0]
    result = {
        "available": True,
        "count": len(feedback),
        "first_receipt_monotonic_ns": receipts[0],
        "last_receipt_monotonic_ns": receipts[-1],
        "receipt_span_s": span_ns / 1.0e9,
        "average_receipt_rate_hz": (
            (len(feedback) - 1) * 1.0e9 / span_ns
            if len(feedback) >= 2 and span_ns > 0
            else None
        ),
        "same_receipt_interval_count": zero_count,
        "positive_receipt_interval_s": _statistics(
            value / 1.0e9 for value in positive
        ),
    }
    if not positive:
        result["gap_candidates"] = {
            "available": False,
            "reason": "no positive inter-frame receipt interval",
            "count": 0,
        }
        return result
    median_ns = _percentile(sorted(positive), 0.5)
    threshold_ns = median_ns * 3.0
    gaps = []
    for index, interval_ns in enumerate(intervals, 1):
        if interval_ns > threshold_ns:
            gaps.append(
                {
                    "previous_receipt_monotonic_ns": receipts[index - 1],
                    "next_receipt_monotonic_ns": receipts[index],
                    "gap_s": interval_ns / 1.0e9,
                }
            )
    result["gap_candidates"] = {
        "available": True,
        "selection_rule": "positive receipt interval > 3 * median positive receipt interval",
        "threshold_s": threshold_ns / 1.0e9,
        "count": len(gaps),
        "events": gaps[:100],
        "events_truncated": len(gaps) > 100,
    }
    return result


def _first_sustained(frames, predicate, required_count):
    run = []
    for frame in frames:
        if predicate(frame):
            run.append(frame)
            if len(run) >= required_count and run[-1]["receipt_ns"] > run[0]["receipt_ns"]:
                return run[:required_count]
        else:
            run = []
    return None


def _motion_candidate(feedback, transmissions, required_count):
    baseline = [item for item in feedback if item["phase"] == "baseline"]
    motion = [
        item
        for item in feedback
        if item["phase"] in {"ramp_up", "hold"}
        and item["associated_tx"] is not None
        and item["associated_tx"]["forward_wire"] > 0
    ]
    base = {
        "selection_rule": (
            "first run of {} feedback frames, spanning positive host-receipt "
            "time, with both derived rear-wheel speeds above their observed "
            "baseline absolute maxima"
        ).format(required_count),
        "required_consecutive_frames": required_count,
        "baseline_frame_count": len(baseline),
    }
    if not baseline:
        base.update({"available": False, "reason": "no baseline feedback frames"})
        return base
    left_threshold = max(abs(item["left_mps"]) for item in baseline)
    right_threshold = max(abs(item["right_mps"]) for item in baseline)
    base["baseline_abs_envelope_mps"] = {
        "left": left_threshold,
        "right": right_threshold,
    }
    run = _first_sustained(
        motion,
        lambda item: item["left_mps"] > left_threshold
        and item["right_mps"] > right_threshold,
        required_count,
    )
    if run is None:
        base.update(
            {"available": False, "reason": "no sustained both-rear-wheel candidate"}
        )
        return base
    first = run[0]
    confirmed = run[-1]
    associated = first["associated_tx"]
    first_nonzero = next(
        (
            item
            for item in transmissions
            if item["forward_wire"] != 0 or item["yaw_wire"] != 0
        ),
        None,
    )
    base.update(
        {
            "available": True,
            "run_frames": len(run),
            "first_candidate_receipt_monotonic_ns": first["receipt_ns"],
            "confirmed_receipt_monotonic_ns": confirmed["receipt_ns"],
            "candidate_run_span_s": (
                confirmed["receipt_ns"] - first["receipt_ns"]
            ) / 1.0e9,
            "associated_command_forward_mps": associated["forward_mps"],
            "associated_command_yaw_radps": associated["yaw_radps"],
            "first_feedback_forward_mps": first["forward_mps"],
            "first_derived_left_mps": first["left_mps"],
            "first_derived_right_mps": first["right_mps"],
            "delay_from_first_nonzero_host_write_s": (
                (first["receipt_ns"] - first_nonzero["after_ns"]) / 1.0e9
                if first_nonzero is not None
                else None
            ),
            "causality_status": "candidate_only_no_vcu_ack_or_source_timestamp",
        }
    )
    return base


def _steady_candidate(feedback):
    selected = [item for item in feedback if item["phase"] == "hold"]
    result = {
        "selection_rule": (
            "all normalized feedback associated with the command hold phase; "
            "settling is not independently proven"
        ),
        "frame_count": len(selected),
    }
    if not selected:
        result.update({"available": False, "reason": "no hold-phase feedback"})
        return result
    matched = [item for item in selected if item["associated_tx"] is not None]
    if len(matched) != len(selected):
        result.update({"available": False, "reason": "hold feedback lacks TX association"})
        return result
    forward_error = [
        item["forward_mps"] - item["associated_tx"]["forward_mps"]
        for item in selected
    ]
    yaw_error = [
        item["yaw_radps"] - item["associated_tx"]["yaw_radps"]
        for item in selected
    ]
    left_command = [
        item["associated_tx"]["forward_mps"]
        - item["associated_tx"]["yaw_radps"] * REAR_TRACK_M / 2.0
        for item in selected
    ]
    right_command = [
        item["associated_tx"]["forward_mps"]
        + item["associated_tx"]["yaw_radps"] * REAR_TRACK_M / 2.0
        for item in selected
    ]
    left_error = [item["left_mps"] - expected for item, expected in zip(selected, left_command)]
    right_error = [item["right_mps"] - expected for item, expected in zip(selected, right_command)]
    actual_difference = [item["right_mps"] - item["left_mps"] for item in selected]
    command_difference = [right - left for left, right in zip(left_command, right_command)]
    difference_error = [
        actual - command
        for actual, command in zip(actual_difference, command_difference)
    ]
    result.update(
        {
            "available": True,
            "feedback": _feedback_statistics(selected),
            "command": {
                "forward_mps": _statistics(
                    item["associated_tx"]["forward_mps"] for item in selected
                ),
                "yaw_radps": _statistics(
                    item["associated_tx"]["yaw_radps"] for item in selected
                ),
                "left_mps": _statistics(left_command),
                "right_mps": _statistics(right_command),
            },
            "tracking_error": {
                "forward_mps": _statistics(forward_error),
                "yaw_radps": _statistics(yaw_error),
                "left_mps": _statistics(left_error),
                "right_mps": _statistics(right_error),
            },
            "overshoot": {
                "forward_positive_mps": max(0.0, max(forward_error)),
                "forward_negative_mps": min(0.0, min(forward_error)),
                "forward_peak_absolute_mps": max(abs(value) for value in forward_error),
                "left_positive_mps": max(0.0, max(left_error)),
                "right_positive_mps": max(0.0, max(right_error)),
            },
            "left_right": {
                "sign_convention": "right_minus_left",
                "actual_difference_mps": _statistics(actual_difference),
                "commanded_difference_mps": _statistics(command_difference),
                "difference_error_mps": _statistics(difference_error),
                "absolute_actual_difference_mps": _statistics(
                    abs(value) for value in actual_difference
                ),
            },
        }
    )
    return result


def _stop_candidate(feedback, transmissions, baseline, required_count):
    # The core labels the normal write that first reaches exact zero as
    # ramp_down, then changes its phase state to post_stop.  Subsequent zero
    # keepalives carry phase=post_stop.  Anchor the stop candidate to the first
    # successful exact-zero write after the final nonzero command, not to the
    # first repeated post-stop keepalive.
    last_nonzero_index = next(
        (
            index
            for index in range(len(transmissions) - 1, -1, -1)
            if transmissions[index]["forward_wire"] != 0
            or transmissions[index]["yaw_wire"] != 0
        ),
        None,
    )
    post_zero_tx = (
        next(
            (
                item
                for item in transmissions[last_nonzero_index + 1:]
                if item["forward_wire"] == 0 and item["yaw_wire"] == 0
            ),
            None,
        )
        if last_nonzero_index is not None
        else None
    )
    post = [item for item in feedback if item["phase"] == "post_stop"]
    result = {
        "selection_rule": (
            "post-stop host-receipt observations compared with pre-motion "
            "baseline absolute derived-wheel envelopes"
        ),
        "frame_count": len(post),
        "required_consecutive_frames": required_count,
        "source_timestamp_available": False,
        "latency_interpretation": "host-observation candidate only",
    }
    if post_zero_tx is None:
        result.update({"available": False, "reason": "no successful post-stop normal TX"})
        return result
    result["first_zero_host_write_monotonic_ns"] = post_zero_tx["after_ns"]
    if not post:
        result.update({"available": False, "reason": "no post-stop normalized feedback"})
        return result
    result["post_stop_observation_span_s"] = (
        post[-1]["receipt_ns"] - post[0]["receipt_ns"]
    ) / 1.0e9
    result["feedback"] = _feedback_statistics(post)
    if not baseline:
        result.update({"available": False, "reason": "no baseline for stop comparison"})
        return result
    left_threshold = max(abs(item["left_mps"]) for item in baseline)
    right_threshold = max(abs(item["right_mps"]) for item in baseline)
    result["baseline_abs_envelope_mps"] = {
        "left": left_threshold,
        "right": right_threshold,
    }
    moving = [
        item for item in post
        if abs(item["left_mps"]) > left_threshold
        or abs(item["right_mps"]) > right_threshold
    ]
    result["last_above_baseline_receipt_monotonic_ns"] = (
        moving[-1]["receipt_ns"] if moving else None
    )
    result["tail_to_last_above_baseline_s"] = (
        max(0.0, (moving[-1]["receipt_ns"] - post_zero_tx["after_ns"]) / 1.0e9)
        if moving else 0.0
    )
    standstill_run = _first_sustained(
        post,
        lambda item: abs(item["left_mps"]) <= left_threshold
        and abs(item["right_mps"]) <= right_threshold,
        required_count,
    )
    if standstill_run is None:
        result.update(
            {"available": False, "reason": "no sustained return-to-baseline candidate"}
        )
        return result
    result.update(
        {
            "available": True,
            "first_return_to_baseline_receipt_monotonic_ns": standstill_run[0]["receipt_ns"],
            "return_confirmed_receipt_monotonic_ns": standstill_run[-1]["receipt_ns"],
            "first_return_from_zero_host_write_s": max(
                0.0,
                (standstill_run[0]["receipt_ns"] - post_zero_tx["after_ns"]) / 1.0e9,
            ),
            "confirmation_from_zero_host_write_s": max(
                0.0,
                (standstill_run[-1]["receipt_ns"] - post_zero_tx["after_ns"]) / 1.0e9,
            ),
        }
    )
    return result


def _flag_stop_summary(feedback):
    counts = Counter(item["flag_stop"] for item in feedback)
    transitions = []
    previous = None
    for item in feedback:
        if previous is not None and item["flag_stop"] != previous:
            transitions.append(
                {
                    "receipt_monotonic_ns": item["receipt_ns"],
                    "from": previous,
                    "to": item["flag_stop"],
                }
            )
        previous = item["flag_stop"]
    return {
        "semantics": "binary composite current-cycle allow/inhibit; not ACK or specific fault",
        "counts": {"0": counts[0], "1": counts[1]},
        "transition_count": len(transitions),
        "transitions": transitions[:100],
        "transitions_truncated": len(transitions) > 100,
    }


def _validate_summary(schema, line_number, summary, feedback_count, tx_count):
    status = _string(summary, "status", line_number)
    zero_host_write_completed = _boolean(
        summary, "zero_host_write_completed", line_number
    )
    delivery_unconfirmed = _boolean(summary, "delivery_unconfirmed", line_number)
    statistics = _required(summary, "statistics", line_number)
    if not isinstance(statistics, dict):
        _error(line_number, "summary statistics must be an object")
    declared_feedback = _integer(
        statistics, "valid_feedback_frames", line_number, 0
    )
    if declared_feedback != feedback_count:
        _error(line_number, "summary feedback count disagrees with records")
    if schema == RAW_PROFILE_SCHEMA:
        observation_events = _integer(
            statistics, "feedback_observation_events", line_number, 0
        )
        if observation_events != feedback_count:
            _error(line_number, "summary feedback event count disagrees with records")
    tx_key = (
        "tx_host_writes_completed"
        if schema == BENCH_SCHEMA
        else "normal_tx_host_writes_completed"
    )
    if tx_key in statistics:
        declared_tx = _integer(statistics, tx_key, line_number, 0)
        if schema == RAW_PROFILE_SCHEMA and declared_tx != tx_count:
            _error(line_number, "summary normal TX count disagrees with records")
        if schema == BENCH_SCHEMA and declared_tx < tx_count:
            _error(line_number, "summary host TX count is below normal TX records")
    started_ns = _integer(statistics, "started_monotonic_ns", line_number, 1)
    ended_ns = _integer(statistics, "ended_monotonic_ns", line_number, 1)
    if ended_ns < started_ns:
        _error(line_number, "summary monotonic interval regressed")
    parser_integrity = {}
    for output_key, candidates in {
        "checksum_failures": ("checksum_failures", "parser_checksum_failures"),
        "framing_failures": ("framing_failures", "parser_framing_failures"),
        "discarded_bytes": ("discarded_bytes", "parser_discarded_bytes"),
        "trailing_buffered_bytes": (
            "trailing_buffered_bytes",
            "parser_trailing_buffered_bytes",
        ),
    }.items():
        present = [key for key in candidates if key in statistics]
        if len(present) > 1:
            _error(line_number, "summary has duplicate parser statistic meanings")
        if present:
            parser_integrity[output_key] = statistics[present[0]]
    for key, value in parser_integrity.items():
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            _error(line_number, "summary {} is invalid".format(key))
    return {
        "status": status,
        "zero_host_write_completed": zero_host_write_completed,
        "delivery_unconfirmed": delivery_unconfirmed,
        "started_monotonic_ns": started_ns,
        "ended_monotonic_ns": ended_ns,
        "duration_s": (ended_ns - started_ns) / 1.0e9,
        "parser_integrity": parser_integrity,
    }


def analyze_path(path):
    records, input_sha256, input_bytes = _read_records(path)
    schema, record_counts = _validate_common_record_headers(records)
    _validate_raw_cli_prefix(schema, records, record_counts)
    metadata_line, metadata = next(
        (line, record)
        for line, record in records
        if record["record_type"] == "metadata"
    )
    profile = _validate_metadata(schema, metadata_line, metadata)
    if schema == RAW_PROFILE_SCHEMA:
        _validate_physical_raw_event_stream(records, record_counts, profile)

    transmissions = []
    failed_normal_tx_count = 0
    feedback = []
    previous_tx_ns = None
    previous_feedback_ns = None
    normal_type = "tx_result" if schema == BENCH_SCHEMA else "normal_tx_result"
    feedback_type = (
        "feedback_frame"
        if schema == BENCH_SCHEMA
        else "feedback_observation_event"
    )
    for line_number, record in records:
        record_type = record["record_type"]
        if record_type == normal_type:
            tx = _successful_normal_tx(schema, line_number, record, profile)
            if not tx["successful"]:
                failed_normal_tx_count += 1
                continue
            if previous_tx_ns is not None and tx["after_ns"] < previous_tx_ns:
                _error(line_number, "successful normal TX clock regressed")
            previous_tx_ns = tx["after_ns"]
            transmissions.append(tx)
        elif record_type == feedback_type:
            frame = _feedback_record(schema, line_number, record, profile)
            if previous_feedback_ns is not None and frame["receipt_ns"] < previous_feedback_ns:
                _error(line_number, "feedback receipt clock regressed")
            previous_feedback_ns = frame["receipt_ns"]
            feedback.append(frame)

    if not transmissions:
        raise AnalysisError("at least one successful normal TX record is required")
    if not feedback:
        raise AnalysisError("at least one normalized feedback record is required")

    phase_basis = "explicit_record_phase"
    if schema == BENCH_SCHEMA:
        _infer_bench_phases(transmissions, profile["target_forward_mps"])
        phase_basis = "inferred_from_successful_normal_tx_wire_sequence"
    _validate_phase_order(transmissions)
    _associate_feedback(feedback, transmissions, schema)

    summary_line, source_summary_record = records[-1]
    terminal = _validate_summary(
        schema,
        summary_line,
        source_summary_record,
        len(feedback),
        len(transmissions),
    )
    required_count = profile["sustained_frames"]
    baseline = [item for item in feedback if item["phase"] == "baseline"]
    first_tx_ns = transmissions[0]["after_ns"]
    timeline = [
        {
            "sequence": index + 1,
            "phase": item["phase"],
            "before_monotonic_ns": item["before_ns"],
            "after_monotonic_ns": item["after_ns"],
            "elapsed_from_first_successful_tx_s": (
                item["after_ns"] - first_tx_ns
            ) / 1.0e9,
            "forward_wire": item["forward_wire"],
            "yaw_wire": item["yaw_wire"],
            "forward_mps": item["forward_mps"],
            "yaw_radps": item["yaw_radps"],
        }
        for index, item in enumerate(transmissions)
    ]
    all_feedback_stats = _feedback_statistics(feedback)
    profile_output = {
        "profile_id": profile["profile_id"],
        "mode": profile["mode"],
        "target_forward_mps": profile["target_forward_mps"],
        "target_curvature_inv_m": profile["target_curvature_inv_m"],
        "target_yaw_radps": profile["target_yaw_radps"],
        "turn_radius_m": profile["turn_radius_m"],
        "hold_duration_ns": profile["hold_duration_ns"],
        "command_acceleration_mps2": profile["command_acceleration_mps2"],
        "command_cap_mps": profile["command_cap_mps"],
    }
    if profile.get("extended_target_contract"):
        profile_output.update(profile["extended_target"])
    result = {
        "schema": ANALYSIS_SCHEMA,
        "analysis_status": "completed",
        "input": {
            "schema": schema,
            "sha256": input_sha256,
            "byte_count": input_bytes,
            "record_count": len(records),
            "record_type_counts": dict(sorted(record_counts.items())),
        },
        "profile": profile_output,
        "semantics": {
            "protocol_status": "UNVERIFIED",
            "timeline_clock": "CLOCK_MONOTONIC host IO/receipt",
            "source_timestamp_available": False,
            "vcu_ack_available": False,
            "successful_normal_tx_meaning": (
                "host operating system reported a complete 11-byte write only"
            ),
            "tx_feedback_association": (
                "latest completed normal host write before feedback receipt; "
                "not causal proof"
            ),
            "derived_rear_wheel_formula": {
                "rear_track_m": REAR_TRACK_M,
                "left": "forward - yaw_rate * rear_track / 2",
                "right": "forward + yaw_rate * rear_track / 2",
            },
        },
        "terminal": terminal,
        "normal_tx": {
            "successful_write_count": len(transmissions),
            "failed_or_unconfirmed_result_count": failed_normal_tx_count,
            "first_successful_write_monotonic_ns": first_tx_ns,
            "last_successful_write_monotonic_ns": transmissions[-1]["after_ns"],
            "forward_command_mps": _statistics(
                item["forward_mps"] for item in transmissions
            ),
            "yaw_command_radps": _statistics(
                item["yaw_radps"] for item in transmissions
            ),
            "timeline": timeline,
        },
        "phases": _phase_summary(transmissions, feedback, phase_basis),
        "feedback_frames": {
            "timing": _frame_timing(feedback),
            "values": all_feedback_stats,
        },
        "motion_onset_candidate": _motion_candidate(
            feedback, transmissions, required_count
        ),
        "steady_state_candidate": _steady_candidate(feedback),
        "stop_tail_candidate": _stop_candidate(
            feedback, transmissions, baseline, required_count
        ),
        "flag_stop": _flag_stop_summary(feedback),
        "limitations": [
            "VCU feedback contains no source timestamp or sequence number.",
            "A full host serial write is not a VCU acknowledgement or command echo.",
            (
                "Motion onset and stop-tail values are host-receipt candidates, "
                "not physical latency proofs."
            ),
            "Hold-phase statistics do not independently prove that the mechanism settled.",
            "This offline analysis neither sets nor weakens runtime safety thresholds.",
        ],
    }
    return result


def _parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Offline/read-only analysis of Wheeltec bench or raw-profile NDJSON; "
            "prints one deterministic JSON summary to stdout."
        )
    )
    parser.add_argument("input", type=Path, help="regular NDJSON evidence file")
    parser.add_argument(
        "--pretty", action="store_true", help="indent output JSON for review"
    )
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_arguments(argv)
    try:
        result = analyze_path(arguments.input)
        if arguments.pretty:
            output = json.dumps(
                result, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2
            )
        else:
            output = json.dumps(
                result,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
    except (AnalysisError, OSError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
