import copy
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = ROOT / "tools/vehicle/analyze_wheeltec_profile.py"


def load_tool():
    spec = importlib.util.spec_from_file_location(
        "analyze_wheeltec_profile", str(TOOL_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def signed_bytes(value):
    bits = value & 0xFFFF
    return [(bits >> 8) & 0xFF, bits & 0xFF]


def command_hex(forward_wire, yaw_wire):
    frame = [0x7B, 0, 0]
    frame.extend(signed_bytes(forward_wire))
    frame.extend(signed_bytes(0))
    frame.extend(signed_bytes(yaw_wire))
    checksum = 0
    for byte in frame:
        checksum ^= byte
    frame.extend([checksum, 0x7D])
    return bytes(frame).hex()


def feedback_hex(forward_wire, yaw_wire, flag_stop=0, voltage_mv=23300):
    frame = [0x7B, flag_stop]
    frame.extend(signed_bytes(forward_wire))
    frame.extend(signed_bytes(0))
    frame.extend(signed_bytes(yaw_wire))
    frame.extend([0] * 12)
    frame.extend([(voltage_mv >> 8) & 0xFF, voltage_mv & 0xFF])
    checksum = 0
    for byte in frame:
        checksum ^= byte
    frame.extend([checksum, 0x7D])
    return bytes(frame).hex()


def bench_metadata():
    return {
        "schema": "auto_rover.wheeltec.bench_characterization.v1",
        "record_type": "metadata",
        "mode": "straight-ramp",
        "protocol_status": "UNVERIFIED",
        "vcu_ack_available": False,
        "successful_write_semantics": "host_os_full_write_only",
        "target_speed_mps": 0.2,
        "hold_duration_ns": 200_000_000,
        "raised_wheel_start_rear_track_m": 0.322,
        "fresh_feedback_receipts_required": 5,
    }


def bench_tx(before_ns, forward_wire):
    raw = command_hex(forward_wire, 0)
    return {
        "schema": "auto_rover.wheeltec.bench_characterization.v1",
        "record_type": "tx_result",
        "clock": "CLOCK_MONOTONIC",
        "before_monotonic_ns": before_ns,
        "after_monotonic_ns": before_ns + 1,
        "absolute_deadline_monotonic_ns": before_ns + 1_000_000,
        "status": "ok",
        "transferred": 11,
        "os_error": 0,
        "delivery_unconfirmed": False,
        "raw_hex": raw,
    }


def bench_feedback(receipt_ns, forward_wire, yaw_wire=0, flag_stop=0):
    return {
        "schema": "auto_rover.wheeltec.bench_characterization.v1",
        "record_type": "feedback_frame",
        "receipt_clock": "CLOCK_MONOTONIC",
        "receipt_monotonic_ns": receipt_ns,
        "raw_hex": feedback_hex(forward_wire, yaw_wire, flag_stop),
        "forward_speed_mps": forward_wire / 1000.0,
        "lateral_speed_mps": 0.0,
        "yaw_rate_radps": yaw_wire / 1000.0,
        "supply_voltage_v": 23.3,
        "composite_stop_flag_raw": flag_stop,
        "control_allowed": flag_stop == 0,
        "control_inhibited": flag_stop == 1,
        "source_time_available": False,
        "control_enabled_available": False,
        "vcu_ack_available": False,
        "specific_fault_available": False,
        "command_echo_available": False,
    }


def bench_summary(feedback_count, tx_count):
    return {
        "schema": "auto_rover.wheeltec.bench_characterization.v1",
        "record_type": "summary",
        "status": "completed",
        "authorization_revoked": True,
        "zero_host_write_completed": True,
        "delivery_unconfirmed": False,
        "statistics": {
            "valid_feedback_frames": feedback_count,
            "tx_host_writes_completed": tx_count,
            "started_monotonic_ns": 90,
            "ended_monotonic_ns": 900,
        },
    }


def raw_metadata():
    return {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": "metadata",
        "profile_id": "left_01",
        "target_forward_mps": 1.0,
        "curvature_inv_m": 0.5,
        "turn_radius_m": 2.0,
        "hold_duration_ns": 60_000_000_000,
        "command_acceleration_mps2": 0.2,
        "command_cap_mps": 2.0,
        "feedback_tracking_acceptance_gate": False,
        "feedback_asymmetry_acceptance_gate": False,
        "feedback_standstill_acceptance_gate": False,
        "phase1_codec_used": False,
        "installed": False,
        "protocol_status": "UNVERIFIED",
        "left_right_difference_sign": "left_minus_right",
        "vcu_ack_available": False,
        "successful_write_semantics": "host_os_full_write_only",
        "derived_rear_track_m": 0.322,
    }


def raw_tx(before_ns, phase, forward_wire, yaw_wire):
    return {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": "normal_tx_result",
        "phase": phase,
        "profile_id": "left_01",
        "clock": "CLOCK_MONOTONIC",
        "before_monotonic_ns": before_ns,
        "after_monotonic_ns": before_ns + 1,
        "deadline_monotonic_ns": before_ns + 1_000_000,
        "status": "ok",
        "transferred": 11,
        "delivery_unconfirmed": False,
        "forward_wire": forward_wire,
        "yaw_wire": yaw_wire,
        "forward_command_mps": forward_wire / 1000.0,
        "yaw_command_radps": yaw_wire / 1000.0,
        "raw_hex": command_hex(forward_wire, yaw_wire),
    }


def raw_feedback(receipt_ns, phase, forward_wire, yaw_wire,
                 last_forward_wire, last_yaw_wire, flag_stop=0):
    forward = forward_wire / 1000.0
    yaw = yaw_wire / 1000.0
    left = forward - yaw * 0.322 / 2.0
    right = forward + yaw * 0.322 / 2.0
    return {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": "feedback_observation_event",
        "receipt_clock": "CLOCK_MONOTONIC",
        "receipt_monotonic_ns": receipt_ns,
        "phase": phase,
        "profile_id": "left_01",
        "last_successful_forward_wire": last_forward_wire,
        "last_successful_yaw_wire": last_yaw_wire,
        "raw_hex": feedback_hex(forward_wire, yaw_wire, flag_stop),
        "forward_speed_mps": forward,
        "lateral_speed_mps": 0.0,
        "yaw_rate_radps": yaw,
        "derived_left_mps": left,
        "derived_right_mps": right,
        "tracking_error_mps": forward - last_forward_wire / 1000.0,
        "left_right_difference_mps": left - right,
        "post_zero_tail": phase == "post_stop",
        "online_acceptance_gate_applied": False,
        "composite_stop_flag_raw": flag_stop,
        "control_allowed": flag_stop == 0,
        "control_inhibited": flag_stop == 1,
        "source_time_available": False,
        "vcu_ack_available": False,
    }


def raw_cli_prefix(profile_id="left_01"):
    schema = "auto_rover.wheeltec.raw_profile_capture.v1"
    return [
        {
            "schema": schema,
            "record_type": "cli_preflight",
            "profile_id": profile_id,
            "four_gates_confirmed": True,
            "raw_raised_bench_opt_in": True,
            "operator_confirmation_token_matched": True,
        },
        {
            "schema": schema,
            "record_type": "open_result",
            "profile_id": profile_id,
            "status": "ok",
            "os_error": 0,
        },
        {
            "schema": schema,
            "record_type": "startup_exact_zero_result",
            "profile_id": profile_id,
            "attempts": 1,
            "exact_zero_frame": True,
            "zero_host_write_completed": True,
            "vcu_acknowledgement": False,
            "delivery_unconfirmed": False,
            "terminal_transport_status": "ok",
            "recording_order": "all_zero_attempts_completed_before_this_record",
        },
    ]


def raw_summary(feedback_count, tx_count):
    return {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": "summary",
        "status": "completed",
        "zero_host_write_completed": True,
        "delivery_unconfirmed": False,
        "statistics": {
            "valid_feedback_frames": feedback_count,
            "feedback_observation_events": feedback_count,
            "normal_tx_host_writes_completed": tx_count,
            "parser_checksum_failures": 0,
            "parser_framing_failures": 0,
            "parser_discarded_bytes": 0,
            "parser_trailing_buffered_bytes": 0,
            "started_monotonic_ns": 90,
            "ended_monotonic_ns": 700,
        },
    }


def raw_io_attempt(record_type, before_ns, phase="baseline", raw=None,
                   forward_wire=0, yaw_wire=0):
    record = {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": record_type,
        "phase": phase,
        "profile_id": "left_01",
        "clock": "CLOCK_MONOTONIC",
        "before_monotonic_ns": before_ns,
        "absolute_deadline_monotonic_ns": before_ns + 1_000_000,
        "requested_bytes": 11 if raw is not None else 512,
    }
    if raw is not None:
        record.update(
            {
                "forward_wire": forward_wire,
                "yaw_wire": yaw_wire,
                "forward_command_mps": forward_wire / 1000.0,
                "yaw_command_radps": yaw_wire / 1000.0,
                "raw_hex": raw,
            }
        )
    return record


def raw_io_result(record_type, before_ns, status, transferred, phase="baseline",
                  raw=None, forward_wire=0, yaw_wire=0):
    record = {
        "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
        "record_type": record_type,
        "phase": phase,
        "profile_id": "left_01",
        "clock": "CLOCK_MONOTONIC",
        "before_monotonic_ns": before_ns,
        "after_monotonic_ns": before_ns + 1,
        "absolute_deadline_monotonic_ns": before_ns + 1_000_000,
        "status": status,
        "transferred": transferred,
        "os_error": 0 if status == "ok" else 110,
        "delivery_unconfirmed": False,
    }
    if raw is not None:
        record.update(
            {
                "forward_wire": forward_wire,
                "yaw_wire": yaw_wire,
                "forward_command_mps": forward_wire / 1000.0,
                "yaw_command_radps": yaw_wire / 1000.0,
                "raw_hex": raw,
            }
        )
    return record


def raw_physical_records():
    zero = command_hex(0, 0)
    moving = command_hex(100, 50)
    feedback = feedback_hex(0, 0)
    records = raw_cli_prefix()
    records.extend(
        [
            raw_metadata(),
            raw_io_attempt("normal_tx_attempt", 100, raw=zero),
            raw_io_result("normal_tx_result", 100, "ok", 11, raw=zero),
            raw_io_attempt("drain_rx_attempt", 110),
            raw_io_result(
                "drain_rx_result", 110, "deadline_exceeded", 0
            ),
            raw_io_attempt("rx_attempt", 120),
            raw_io_result("rx_result", 120, "ok", 24),
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "raw_rx_chunk",
                "receipt_clock": "CLOCK_MONOTONIC",
                "receipt_monotonic_ns": 121,
                "phase": "baseline",
                "profile_id": "left_01",
                "transferred": 24,
                "raw_hex": feedback,
            },
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "parser_observation_event",
                "receipt_monotonic_ns": 121,
                "phase": "baseline",
                "profile_id": "left_01",
                "checksum_failures_delta": 0,
                "framing_failures_delta": 0,
                "discarded_bytes_delta": 0,
                "checksum_failures_total": 0,
                "framing_failures_total": 0,
                "discarded_bytes_total": 0,
                "trailing_buffered_bytes": 0,
                "online_acceptance_gate_applied": False,
            },
            raw_feedback(121, "baseline", 0, 0, 0, 0),
            raw_io_attempt(
                "normal_tx_attempt", 130, phase="ramp_up", raw=moving,
                forward_wire=100, yaw_wire=50
            ),
            raw_io_result(
                "normal_tx_result", 130, "ok", 11, phase="ramp_up",
                raw=moving, forward_wire=100, yaw_wire=50
            ),
            raw_io_attempt("rx_attempt", 140, phase="ramp_up"),
            raw_io_result(
                "rx_result", 140, "deadline_exceeded", 0,
                phase="ramp_up"
            ),
            raw_io_attempt(
                "emergency_zero_tx_attempt_postwrite_record", 150,
                phase="ramp_up", raw=zero
            ),
            raw_io_result(
                "emergency_zero_tx_result_postwrite_record",
                150,
                "ok",
                11,
                phase="ramp_up",
                raw=zero,
            ),
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "summary",
                "status": "completed",
                "zero_host_write_completed": True,
                "delivery_unconfirmed": False,
                "statistics": {
                    "read_calls": 3,
                    "raw_rx_bytes": 24,
                    "valid_feedback_frames": 1,
                    "feedback_observation_events": 1,
                    "parser_checksum_failures": 0,
                    "parser_framing_failures": 0,
                    "parser_discarded_bytes": 0,
                    "parser_trailing_buffered_bytes": 0,
                    "read_timeouts": 1,
                    "tx_attempts": 3,
                    "tx_host_writes_completed": 3,
                    "normal_tx_host_writes_completed": 2,
                    "zero_frame_host_writes_completed": 2,
                    "nonzero_frame_host_writes_completed": 1,
                    "maximum_commanded_forward_wire": 100,
                    "started_monotonic_ns": 90,
                    "ended_monotonic_ns": 160,
                },
            },
        ]
    )
    return records


def raw_outer_metadata(radius_mm, turn_sign):
    expected = {
        2000: (5553, 2776, 5.999936, 5.106064),
        950: (5130, 5400, 5.9994, 4.2606),
    }
    center_wire, yaw_magnitude, outer_mps, inner_mps = expected[radius_mm]
    yaw_wire = turn_sign * yaw_magnitude
    center_mps = center_wire / 1000.0
    yaw_radps = yaw_wire / 1000.0
    left_mps = center_mps - yaw_radps * 0.322 / 2.0
    right_mps = center_mps + yaw_radps * 0.322 / 2.0
    metadata = raw_metadata()
    metadata.update(
        {
            "speed_tier_semantics": "outer_rear_wheel_command_upper_limit",
            "speed_tier_wire": 6000,
            "speed_tier_mps": 6.0,
            "target_center_forward_wire": center_wire,
            "target_forward_mps": center_mps,
            "target_yaw_wire": yaw_wire,
            "target_yaw_radps": yaw_radps,
            "target_outer_command_mps": outer_mps,
            "target_inner_command_mps": inner_mps,
            "derived_target_left_mps": left_mps,
            "derived_target_right_mps": right_mps,
            "outer_tier_slack_mps": 6.0 - outer_mps,
            "turn_direction": "left" if turn_sign > 0 else "right",
            "curvature_inv_m": turn_sign * 1000.0 / radius_mm,
            "turn_radius_m": radius_mm / 1000.0,
            "turn_radius_mm": radius_mm,
            "derived_rear_track_mm": 322,
            "command_cap_mps": 6.0,
        }
    )
    return metadata, center_wire, yaw_wire


class WheeltecProfileAnalyzerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tool = load_tool()

    def write_records(self, records):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name) / "profile.ndjson"
        path.write_text(
            "".join(json.dumps(item, separators=(",", ":")) + "\n"
                    for item in records),
            encoding="utf-8",
        )
        return temporary, path

    def test_bench_schema_reconstructs_timeline_and_candidates(self):
        records = [bench_metadata(), bench_tx(100, 0)]
        records.extend(
            bench_feedback(receipt, value)
            for receipt, value in zip(range(110, 160, 10), [0, 1, -1, 0, 1])
        )
        records.append(bench_tx(200, 100))
        records.extend(
            bench_feedback(receipt, 60)
            for receipt in range(210, 260, 10)
        )
        records.extend([bench_tx(260, 200), bench_tx(300, 200)])
        records.extend(
            bench_feedback(receipt, value)
            for receipt, value in zip(
                range(310, 410, 10),
                [190, 200, 205, 200, 198, 202, 200, 201, 199, 200],
            )
        )
        records.extend([bench_tx(500, 100), bench_tx(600, 0)])
        records.extend(
            bench_feedback(receipt, value)
            for receipt, value in zip(
                range(610, 680, 10), [50, 10, 1, 0, -1, 0, 0]
            )
        )
        feedback_count = sum(
            item["record_type"] == "feedback_frame" for item in records
        )
        tx_count = sum(item["record_type"] == "tx_result" for item in records)
        records.append(bench_summary(feedback_count, tx_count))
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)

        summary = self.tool.analyze_path(path)

        self.assertEqual(
            "auto_rover.wheeltec.profile_analysis.v1", summary["schema"]
        )
        self.assertFalse(summary["semantics"]["source_timestamp_available"])
        self.assertFalse(summary["semantics"]["vcu_ack_available"])
        self.assertEqual(6, summary["normal_tx"]["successful_write_count"])
        self.assertEqual(
            ["baseline", "ramp_up", "hold", "hold", "ramp_down", "post_stop"],
            [item["phase"] for item in summary["normal_tx"]["timeline"]],
        )
        self.assertEqual(5, summary["motion_onset_candidate"]["run_frames"])
        self.assertEqual(
            0.1,
            summary["motion_onset_candidate"]["associated_command_forward_mps"],
        )
        self.assertEqual(10, summary["steady_state_candidate"]["frame_count"])
        self.assertAlmostEqual(
            0.1995,
            summary["steady_state_candidate"]["feedback"]["forward_mps"]["mean"],
        )
        self.assertEqual(7, summary["stop_tail_candidate"]["frame_count"])
        self.assertTrue(summary["stop_tail_candidate"]["available"])
        self.assertEqual(feedback_count, summary["flag_stop"]["counts"]["0"])

    def test_raw_profile_schema_uses_explicit_phases_and_turning_values(self):
        records = [raw_metadata(), raw_tx(100, "baseline", 0, 0)]
        records.extend(
            raw_feedback(receipt, "baseline", 0, 0, 0, 0)
            for receipt in range(110, 160, 10)
        )
        # The injected monotonic clock may repeat: this feedback record occurs
        # before, but at the same timestamp as, the following TX completion.
        # NDJSON record order must break that tie.
        records.append(raw_feedback(201, "baseline", 0, 0, 0, 0))
        records.append(raw_tx(200, "ramp_up", 500, 250))
        records.extend(
            raw_feedback(receipt, "ramp_up", 400, 200, 500, 250)
            for receipt in range(210, 260, 10)
        )
        records.append(
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "raw_rx_chunk",
                "receipt_clock": "CLOCK_MONOTONIC",
                "receipt_monotonic_ns": 290,
                "phase": "ramp_up",
                "profile_id": "left_01",
                "transferred": 24,
                "raw_hex": feedback_hex(400, 200),
            }
        )
        records.append(
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "parser_observation_event",
                "receipt_monotonic_ns": 290,
                "phase": "ramp_up",
                "profile_id": "left_01",
                "checksum_failures_delta": 0,
                "framing_failures_delta": 0,
                "discarded_bytes_delta": 0,
                "checksum_failures_total": 0,
                "framing_failures_total": 0,
                "discarded_bytes_total": 0,
                "trailing_buffered_bytes": 0,
                "online_acceptance_gate_applied": False,
            }
        )
        records.append(raw_tx(300, "hold", 1000, 500))
        records.extend(
            raw_feedback(receipt, "hold", 1000, 500, 1000, 500)
            for receipt in range(310, 410, 10)
        )
        records.extend(
            [raw_tx(500, "ramp_down", 500, 250),
             raw_tx(600, "post_stop", 0, 0)]
        )
        records.extend(
            raw_feedback(receipt, "post_stop", 0, 0, 0, 0)
            for receipt in range(610, 660, 10)
        )
        feedback_count = sum(
            item["record_type"] == "feedback_observation_event"
            for item in records
        )
        tx_count = sum(
            item["record_type"] == "normal_tx_result" for item in records
        )
        records.append(
            {
                "schema": "auto_rover.wheeltec.raw_profile_capture.v1",
                "record_type": "summary",
                "status": "completed",
                "zero_host_write_completed": True,
                "delivery_unconfirmed": False,
                "statistics": {
                    "valid_feedback_frames": feedback_count,
                    "feedback_observation_events": feedback_count,
                    "normal_tx_host_writes_completed": tx_count,
                    "parser_checksum_failures": 0,
                    "parser_framing_failures": 0,
                    "parser_discarded_bytes": 0,
                    "parser_trailing_buffered_bytes": 0,
                    "started_monotonic_ns": 90,
                    "ended_monotonic_ns": 700,
                },
            }
        )
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)

        summary = self.tool.analyze_path(path)

        self.assertEqual("left_01", summary["profile"]["profile_id"])
        self.assertEqual(0, summary["terminal"]["parser_integrity"]["checksum_failures"])
        self.assertEqual("explicit_record_phase", summary["phases"]["basis"])
        hold = summary["steady_state_candidate"]
        self.assertEqual(10, hold["frame_count"])
        self.assertAlmostEqual(0.9195, hold["feedback"]["left_mps"]["mean"])
        self.assertAlmostEqual(1.0805, hold["feedback"]["right_mps"]["mean"])
        self.assertAlmostEqual(
            0.161,
            hold["left_right"]["actual_difference_mps"]["mean"],
        )
        self.assertEqual(0.5, summary["normal_tx"]["timeline"][2]["yaw_radps"])

    def test_outer_tier_r2_and_r0p95_use_actual_integer_target_yaw(self):
        cases = ((2000, 1, 2.776), (950, -1, -5.4))
        for radius_mm, turn_sign, expected_yaw in cases:
            with self.subTest(radius_mm=radius_mm, turn_sign=turn_sign):
                metadata, center_wire, yaw_wire = raw_outer_metadata(
                    radius_mm, turn_sign
                )
                records = [
                    metadata,
                    raw_tx(100, "baseline", 0, 0),
                    raw_feedback(110, "baseline", 0, 0, 0, 0),
                    raw_tx(200, "ramp_up", center_wire, yaw_wire),
                    raw_summary(1, 2),
                ]
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)

                summary = self.tool.analyze_path(path)

                self.assertEqual(
                    "outer_rear_wheel_command_upper_limit",
                    summary["profile"]["speed_tier_semantics"],
                )
                self.assertEqual(
                    center_wire,
                    summary["profile"]["target_center_forward_wire"],
                )
                self.assertEqual(
                    yaw_wire, summary["profile"]["target_yaw_wire"]
                )
                self.assertEqual(
                    expected_yaw,
                    summary["profile"]["target_yaw_radps"],
                )
                self.assertLessEqual(
                    summary["profile"]["target_outer_command_mps"],
                    summary["profile"]["speed_tier_mps"],
                )
                if radius_mm == 2000:
                    self.assertNotEqual(
                        metadata["target_forward_mps"]
                        * metadata["curvature_inv_m"],
                        summary["profile"]["target_yaw_radps"],
                    )

    def test_outer_tier_metadata_tampering_fails_closed(self):
        def valid_records():
            metadata, center_wire, yaw_wire = raw_outer_metadata(950, 1)
            return [
                metadata,
                raw_tx(100, "baseline", 0, 0),
                raw_feedback(110, "baseline", 0, 0, 0, 0),
                raw_tx(200, "ramp_up", center_wire, yaw_wire),
                raw_summary(1, 2),
            ]

        cases = []
        changed_center = valid_records()
        changed_center[0]["target_center_forward_wire"] += 1
        cases.append((changed_center, "target center wire disagrees"))

        changed_yaw = valid_records()
        changed_yaw[0]["target_yaw_wire"] += 1
        cases.append((changed_yaw, "target yaw wire disagrees"))

        changed_yaw_mps = valid_records()
        changed_yaw_mps[0]["target_yaw_radps"] += 0.001
        cases.append((changed_yaw_mps, "target yaw wire disagrees"))

        changed_tier = valid_records()
        changed_tier[0]["speed_tier_mps"] = 5.9
        cases.append((changed_tier, "speed tier wire/mps disagree"))

        changed_outer = valid_records()
        changed_outer[0]["target_outer_command_mps"] += 0.001
        cases.append((changed_outer, "target inner/outer geometry disagrees"))

        changed_inner = valid_records()
        changed_inner[0]["target_inner_command_mps"] += 0.001
        cases.append((changed_inner, "target inner/outer geometry disagrees"))

        changed_derived = valid_records()
        changed_derived[0]["derived_target_left_mps"] += 0.001
        cases.append((changed_derived, "derived target left/right geometry disagrees"))

        changed_bound = valid_records()
        changed_bound[0]["speed_tier_wire"] = 5999
        changed_bound[0]["speed_tier_mps"] = 5.999
        cases.append((changed_bound, "outer command exceeds its speed tier"))

        missing_field = valid_records()
        del missing_field[0]["target_yaw_wire"]
        cases.append((missing_field, "target metadata must be complete"))

        for index, (records, message) in enumerate(cases):
            with self.subTest(case=index):
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_physical_cli_prefix_proves_startup_zero_before_session(self):
        records = raw_physical_records()
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)

        summary = self.tool.analyze_path(path)

        self.assertEqual(
            1,
            summary["input"]["record_type_counts"][
                "startup_exact_zero_result"
            ],
        )
        self.assertEqual("left_01", summary["profile"]["profile_id"])

    def test_raw_physical_event_stream_rejects_unpaired_or_changed_io(self):
        cases = []

        changed_normal_result = copy.deepcopy(raw_physical_records())
        result = next(
            item for item in changed_normal_result
            if item["record_type"] == "normal_tx_result"
        )
        result["before_monotonic_ns"] += 1
        cases.append((changed_normal_result, "normal TX attempt/result fields disagree"))

        missing_read_result = copy.deepcopy(raw_physical_records())
        index = next(
            index for index, item in enumerate(missing_read_result)
            if item["record_type"] == "drain_rx_result"
        )
        del missing_read_result[index]
        cases.append((missing_read_result, "drain RX attempt must be followed"))

        changed_chunk = copy.deepcopy(raw_physical_records())
        chunk = next(
            item for item in changed_chunk
            if item["record_type"] == "raw_rx_chunk"
        )
        chunk["receipt_monotonic_ns"] += 1
        cases.append((changed_chunk, "raw RX chunk disagrees with RX result"))

        missing_parser = copy.deepcopy(raw_physical_records())
        index = next(
            index for index, item in enumerate(missing_parser)
            if item["record_type"] == "parser_observation_event"
        )
        del missing_parser[index]
        cases.append((missing_parser, "raw RX chunk must be followed by parser"))

        for index, (records, message) in enumerate(cases):
            with self.subTest(case=index):
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_physical_parser_ledger_rejects_delta_and_terminal_tampering(self):
        cases = []

        changed_delta = copy.deepcopy(raw_physical_records())
        parser = next(
            item for item in changed_delta
            if item["record_type"] == "parser_observation_event"
        )
        parser["checksum_failures_delta"] = 1
        cases.append((changed_delta, "parser delta/total fields are inconsistent"))

        changed_terminal = copy.deepcopy(raw_physical_records())
        changed_terminal[-1]["statistics"]["parser_trailing_buffered_bytes"] = 1
        cases.append((changed_terminal, "summary parser statistics disagree"))

        for index, (records, message) in enumerate(cases):
            with self.subTest(case=index):
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_physical_emergency_zero_is_exact_paired_and_terminal(self):
        cases = []

        nonzero_emergency = copy.deepcopy(raw_physical_records())
        for record_type in (
            "emergency_zero_tx_attempt_postwrite_record",
            "emergency_zero_tx_result_postwrite_record",
        ):
            record = next(
                item for item in nonzero_emergency
                if item["record_type"] == record_type
            )
            record.update(
                {
                    "forward_wire": 1,
                    "forward_command_mps": 0.001,
                    "raw_hex": command_hex(1, 0),
                }
            )
        cases.append((nonzero_emergency, "emergency zero command must be exact zero"))

        false_completion = copy.deepcopy(raw_physical_records())
        false_completion[-1]["zero_host_write_completed"] = False
        cases.append((false_completion, "final emergency-zero completion disagrees"))

        trailing_normal = copy.deepcopy(raw_physical_records())
        zero = command_hex(0, 0)
        trailing_normal[-1:-1] = [
            raw_io_attempt("normal_tx_attempt", 155, raw=zero),
            raw_io_result("normal_tx_result", 155, "ok", 11, raw=zero),
        ]
        cases.append((trailing_normal, "only emergency-zero pairs may follow"))

        for index, (records, message) in enumerate(cases):
            with self.subTest(case=index):
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_physical_summary_counters_are_reconstructed_from_events(self):
        counter_cases = {
            "read_calls": "summary read_calls disagrees",
            "read_timeouts": "summary read_timeouts disagrees",
            "raw_rx_bytes": "summary raw_rx_bytes disagrees",
            "valid_feedback_frames": "summary feedback count disagrees",
            "feedback_observation_events": "summary feedback event count disagrees",
            "normal_tx_host_writes_completed": "summary normal TX count disagrees",
            "zero_frame_host_writes_completed": "summary zero-frame TX count disagrees",
            "nonzero_frame_host_writes_completed": "summary nonzero-frame TX count disagrees",
            "maximum_commanded_forward_wire": "summary maximum command disagrees",
            "tx_attempts": "summary TX attempts disagrees",
            "tx_host_writes_completed": "summary completed TX count disagrees",
        }
        for key, message in counter_cases.items():
            with self.subTest(counter=key):
                records = copy.deepcopy(raw_physical_records())
                records[-1]["statistics"][key] += 1
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_physical_cli_prefix_fails_closed_if_incomplete_or_unsafe(self):
        def complete_records():
            records = raw_cli_prefix()
            records.extend(
                [
                    raw_metadata(),
                    raw_tx(100, "baseline", 0, 0),
                    raw_feedback(110, "baseline", 0, 0, 0, 0),
                    raw_summary(1, 1),
                ]
            )
            return records

        cases = []
        missing_zero = complete_records()
        del missing_zero[2]
        cases.append((missing_zero, "exactly one 'startup_exact_zero_result'"))

        wrong_order = complete_records()
        wrong_order[1], wrong_order[2] = wrong_order[2], wrong_order[1]
        cases.append((wrong_order, "cli_preflight -> open_result"))

        unconfirmed_zero = complete_records()
        unconfirmed_zero[2]["delivery_unconfirmed"] = True
        cases.append((unconfirmed_zero, "delivery may not be unconfirmed"))

        not_exact_zero = complete_records()
        not_exact_zero[2]["exact_zero_frame"] = False
        cases.append((not_exact_zero, "identify an exact-zero frame"))

        ack_claim = complete_records()
        ack_claim[2]["vcu_acknowledgement"] = True
        cases.append((ack_claim, "may not claim a VCU acknowledgement"))

        wrong_record_order = complete_records()
        wrong_record_order[2]["recording_order"] = "before_zero_attempts"
        cases.append((wrong_record_order, "recording order is unsupported"))

        for index, (records, message) in enumerate(cases):
            with self.subTest(case=index):
                temporary, path = self.write_records(records)
                self.addCleanup(temporary.cleanup)
                with self.assertRaisesRegex(self.tool.AnalysisError, message):
                    self.tool.analyze_path(path)

    def test_raw_stop_candidate_starts_at_first_zero_after_final_nonzero(self):
        records = [raw_metadata(), raw_tx(100, "baseline", 0, 0)]
        records.extend(
            raw_feedback(receipt, "baseline", 0, 0, 0, 0)
            for receipt in range(110, 160, 10)
        )
        records.append(raw_tx(200, "ramp_up", 100, 50))
        records.extend(
            raw_feedback(receipt, "ramp_up", 100, 50, 100, 50)
            for receipt in range(210, 260, 10)
        )
        # The first exact-zero write still carries ramp_down.  The next zero
        # is only a post_stop keepalive and must not become the stop t0.
        records.extend(
            [
                raw_tx(300, "ramp_down", 0, 0),
                raw_tx(320, "post_stop", 0, 0),
            ]
        )
        records.extend(
            raw_feedback(receipt, "post_stop", 0, 0, 0, 0)
            for receipt in range(330, 380, 10)
        )
        feedback_count = sum(
            item["record_type"] == "feedback_observation_event"
            for item in records
        )
        tx_count = sum(
            item["record_type"] == "normal_tx_result" for item in records
        )
        records.append(raw_summary(feedback_count, tx_count))
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)

        summary = self.tool.analyze_path(path)

        self.assertEqual(
            301,
            summary["stop_tail_candidate"][
                "first_zero_host_write_monotonic_ns"
            ],
        )

    def test_unknown_schema_fails_closed(self):
        temporary, path = self.write_records(
            [{"schema": "unknown.v1", "record_type": "metadata"}]
        )
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(self.tool.AnalysisError, "unsupported schema"):
            self.tool.analyze_path(path)

    def test_duplicate_json_key_fails_closed(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "profile.ndjson"
        path.write_text(
            '{"schema":"auto_rover.wheeltec.bench_characterization.v1",'
            '"schema":"auto_rover.wheeltec.bench_characterization.v1",'
            '"record_type":"metadata"}\n',
            encoding="utf-8",
        )
        with self.assertRaisesRegex(self.tool.AnalysisError, "duplicate JSON key"):
            self.tool.analyze_path(path)

    def test_nonfinite_number_fails_closed(self):
        record = bench_metadata()
        record["target_speed_mps"] = math.inf
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "profile.ndjson"
        path.write_text(
            json.dumps(record).replace("Infinity", "NaN") + "\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(self.tool.AnalysisError, "non-finite"):
            self.tool.analyze_path(path)

    def test_corrupt_command_frame_fails_closed(self):
        records = [bench_metadata(), bench_tx(100, 0)]
        records[1]["raw_hex"] = records[1]["raw_hex"][:-4] + "007d"
        records.extend(
            [bench_feedback(110, 0), bench_summary(1, 1)]
        )
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(self.tool.AnalysisError, "checksum"):
            self.tool.analyze_path(path)

    def test_ack_or_source_timestamp_claim_fails_closed(self):
        records = [bench_metadata(), bench_tx(100, 0), bench_feedback(110, 0)]
        records[2]["vcu_ack_available"] = True
        records.append(bench_summary(1, 1))
        temporary, path = self.write_records(records)
        self.addCleanup(temporary.cleanup)
        with self.assertRaisesRegex(self.tool.AnalysisError, "VCU ACK"):
            self.tool.analyze_path(path)


if __name__ == "__main__":
    unittest.main()
