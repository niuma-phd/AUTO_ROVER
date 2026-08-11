#include "auto_rover_vcu_wheeltec_serial/raw_profile.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <set>
#include <new>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

int g_failures = 0;
int g_test_cases_run = 0;
std::uint64_t g_longest_profile_evidence_bytes = 0U;
std::int64_t g_longest_profile_duration_ns = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
  }
}

void runTest(void (*test)()) {
  ++g_test_cases_run;
  test();
}

std::uint8_t xorBytes(const std::uint8_t* bytes, std::size_t count) {
  std::uint8_t value = 0U;
  for (std::size_t index = 0U; index < count; ++index) {
    value = static_cast<std::uint8_t>(value ^ bytes[index]);
  }
  return value;
}

void putSignedBigEndian(std::int16_t value, std::uint8_t* high,
                        std::uint8_t* low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  *high = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  *low = static_cast<std::uint8_t>(bits & 0xFFU);
}

std::int16_t signedBigEndian(std::uint8_t high, std::uint8_t low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low));
  return bits <= 0x7FFFU
             ? static_cast<std::int16_t>(bits)
             : static_cast<std::int16_t>(
                   static_cast<std::int32_t>(bits) - 65536);
}

wheeltec::FeedbackFrame feedback(std::int16_t forward,
                                 std::int16_t lateral = 0,
                                 std::int16_t yaw = 0,
                                 std::uint8_t stop_flag = 0U) {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = stop_flag;
  putSignedBigEndian(forward, &frame[2U], &frame[3U]);
  putSignedBigEndian(lateral, &frame[4U], &frame[5U]);
  putSignedBigEndian(yaw, &frame[6U], &frame[7U]);
  putSignedBigEndian(24000, &frame[20U], &frame[21U]);
  frame[22U] = xorBytes(frame.data(), 22U);
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

class ScriptedTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedTransport(std::int64_t* clock) : clock_(clock) {}

  bool isConnected() const override { return connected; }
  std::uint64_t connectionGeneration() const override { return 1U; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t) override {
    io_order.push_back('W');
    writes.emplace_back(data, data + size);
    write_receipts.push_back(*clock_);
    if (!connected) {
      return {wheeltec::TransportStatus::kDisconnected, 0U, ENODEV, true};
    }
    if (!write_durations_ns.empty()) {
      *clock_ += write_durations_ns.front();
      write_durations_ns.pop_front();
    }
    const bool nonzero = size == wheeltec::kCommandFrameSize &&
                         commandForward(writes.back()) != 0;
    const bool zero = size == wheeltec::kCommandFrameSize &&
                      commandForward(writes.back()) == 0 &&
                      commandYaw(writes.back()) == 0;
    if (zero && throw_first_zero_write && !zero_throw_triggered) {
      zero_throw_triggered = true;
      throw std::bad_alloc();
    }
    if (zero && !scripted_zero_write_results.empty()) {
      const wheeltec::IoResult result = scripted_zero_write_results.front();
      scripted_zero_write_results.pop_front();
      return result;
    }
    if (zero && zero_failures_remaining > 0U) {
      --zero_failures_remaining;
      return {wheeltec::TransportStatus::kIoError, 0U, EIO, false};
    }
    if (throw_first_nonzero_write && nonzero &&
        !nonzero_throw_triggered) {
      nonzero_throw_triggered = true;
      writes_when_nonzero_throw_triggered = writes.size();
      throw std::bad_alloc();
    }
    if (partial_first_nonzero && nonzero && !partial_triggered) {
      partial_triggered = true;
      writes_when_partial_triggered = writes.size();
      return {wheeltec::TransportStatus::kOk, size - 1U, EIO, true};
    }
    if (disconnect_first_nonzero && nonzero && !disconnect_triggered) {
      disconnect_triggered = true;
      connected = false;
      return {wheeltec::TransportStatus::kDisconnected, 0U, ENODEV, true};
    }
    return {wheeltec::TransportStatus::kOk, size, 0, false};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t deadline_ns) override {
    io_order.push_back('R');
    if (deadline_ns - *clock_ <= 1000000) {
      *clock_ = deadline_ns;
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
              false};
    }
    if (!connected) {
      return {wheeltec::TransportStatus::kDisconnected, 0U, ENODEV, true};
    }
    if (reads.empty()) {
      *clock_ = deadline_ns;
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
              false};
    }
    if (!read_delays_ns.empty()) {
      *clock_ += read_delays_ns.front();
      read_delays_ns.pop_front();
    }
    const std::vector<std::uint8_t> bytes = std::move(reads.front());
    reads.pop_front();
    if (bytes.size() > capacity) {
      return {wheeltec::TransportStatus::kOk, capacity + 1U, 0, false};
    }
    std::copy(bytes.begin(), bytes.end(), data);
    return {wheeltec::TransportStatus::kOk, bytes.size(), 0, false};
  }

  static std::int16_t commandForward(const std::vector<std::uint8_t>& frame) {
    return frame.size() == wheeltec::kCommandFrameSize
               ? signedBigEndian(frame[3U], frame[4U])
               : 0;
  }

  static std::int16_t commandYaw(const std::vector<std::uint8_t>& frame) {
    return frame.size() == wheeltec::kCommandFrameSize
               ? signedBigEndian(frame[7U], frame[8U])
               : 0;
  }

  bool connected{true};
  bool partial_first_nonzero{false};
  bool partial_triggered{false};
  std::size_t writes_when_partial_triggered{0U};
  bool throw_first_nonzero_write{false};
  bool nonzero_throw_triggered{false};
  std::size_t writes_when_nonzero_throw_triggered{0U};
  bool disconnect_first_nonzero{false};
  bool disconnect_triggered{false};
  bool throw_first_zero_write{false};
  bool zero_throw_triggered{false};
  std::uint32_t zero_failures_remaining{0U};
  std::int64_t* clock_{nullptr};
  std::deque<std::vector<std::uint8_t>> reads;
  std::deque<std::int64_t> read_delays_ns;
  std::deque<std::int64_t> write_durations_ns;
  std::deque<wheeltec::IoResult> scripted_zero_write_results;
  std::vector<std::vector<std::uint8_t>> writes;
  std::vector<std::int64_t> write_receipts;
  std::vector<char> io_order;
};

wheeltec::RawProfileConfig validConfig(wheeltec::RawProfile profile) {
  wheeltec::RawProfileConfig config;
  config.profile = profile;
  config.unverified_protocol_acknowledged = true;
  config.physical_device_opt_in = true;
  config.actuation_opt_in = true;
  config.raw_raised_bench_opt_in = true;
  config.operator_confirmation_token =
      wheeltec::kRawProfileOperatorConfirmationToken;
  config.passive_evidence_token =
      "passive-capture-sha256:"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  config.exact_zero_evidence_token =
      "exact-zero-sha256:"
      "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789";
  return config;
}

struct RecordState {
  std::uint64_t count{0U};
  std::uint64_t observation_count{0U};
  std::uint64_t raw_chunk_count{0U};
  std::uint64_t parser_error_event_count{0U};
  std::uint64_t recorded_bytes{0U};
  bool saw_metadata{false};
  bool saw_record_only_semantics{false};
  std::uint64_t fail_at{0U};
  std::uint64_t now_calls{0U};
  std::uint64_t rollback_at_now_call{0U};
  std::uint64_t wait_calls{0U};
  std::uint64_t jump_at_wait_call{0U};
  std::int64_t wait_jump_ns{101000000};
  std::uint64_t stop_calls{0U};
  std::uint64_t stop_after_calls{0U};
  std::string metadata_line;
  std::string feedback_line;
  std::string normal_tx_line;
  bool capture_all{false};
  std::vector<std::string> all_lines;
};

wheeltec::RawProfileOperations operations(std::int64_t* clock,
                                          RecordState* records) {
  wheeltec::RawProfileOperations result;
  result.monotonic_now_ns = [clock, records]() {
    ++records->now_calls;
    if (records->rollback_at_now_call != 0U &&
        records->now_calls == records->rollback_at_now_call) {
      *clock -= 200000000;
    }
    return *clock;
  };
  result.wait_until_monotonic_ns = [clock, records](std::int64_t deadline) {
    ++records->wait_calls;
    if (*clock < deadline) {
      *clock = deadline;
    }
    if (records->jump_at_wait_call != 0U &&
        records->wait_calls == records->jump_at_wait_call) {
      *clock += records->wait_jump_ns;
    }
    return true;
  };
  result.record_json_line = [records](const std::string& record) {
    ++records->count;
    records->recorded_bytes +=
        static_cast<std::uint64_t>(record.size() + 1U);
    if (records->capture_all) {
      records->all_lines.push_back(record);
    }
    if (record.find("\"record_type\":\"metadata\"") !=
        std::string::npos) {
      records->saw_metadata = true;
      if (records->metadata_line.empty()) {
        records->metadata_line = record;
      }
      records->saw_record_only_semantics =
          record.find("\"feedback_tracking_acceptance_gate\":false") !=
              std::string::npos &&
          record.find("\"installed\":false") != std::string::npos;
    }
    if (record.find("\"record_type\":\"feedback_observation_event\"") !=
        std::string::npos) {
      ++records->observation_count;
      if (records->feedback_line.empty()) {
        records->feedback_line = record;
      }
    }
    if (record.find("\"record_type\":\"normal_tx_result\"") !=
            std::string::npos &&
        record.find("\"forward_wire\":0,") == std::string::npos &&
        records->normal_tx_line.empty()) {
      records->normal_tx_line = record;
    }
    if (record.find("\"record_type\":\"raw_rx_chunk\"") !=
        std::string::npos) {
      ++records->raw_chunk_count;
    }
    if (record.find("\"record_type\":\"parser_observation_event\"") !=
            std::string::npos &&
        (record.find("\"checksum_failures_delta\":0") ==
             std::string::npos ||
         record.find("\"framing_failures_delta\":0") ==
             std::string::npos ||
         record.find("\"discarded_bytes_delta\":0") ==
             std::string::npos)) {
      ++records->parser_error_event_count;
    }
    return records->fail_at == 0U || records->count < records->fail_at;
  };
  result.stop_requested = [records]() {
    ++records->stop_calls;
    return records->stop_after_calls != 0U &&
           records->stop_calls >= records->stop_after_calls;
  };
  return result;
}

void addFeedback(ScriptedTransport* transport,
                 const wheeltec::FeedbackFrame& frame,
                 std::size_t count = 6000U) {
  for (std::size_t index = 0U; index < count; ++index) {
    transport->reads.emplace_back(frame.begin(), frame.end());
  }
}

bool isExactZero(const std::vector<std::uint8_t>& frame) {
  return frame.size() == wheeltec::kCommandFrameSize &&
         ScriptedTransport::commandForward(frame) == 0 &&
         signedBigEndian(frame[5U], frame[6U]) == 0 &&
         ScriptedTransport::commandYaw(frame) == 0;
}

bool strictEnvelopeHolds(const ScriptedTransport& transport,
                         std::size_t normal_writes) {
  if (normal_writes == 0U || normal_writes > transport.writes.size() ||
      normal_writes > transport.write_receipts.size()) {
    return false;
  }
  wheeltec::RawProfileWireAccelerationEnvelope replay;
  for (std::size_t index = 0U; index < normal_writes; ++index) {
    const std::int16_t forward =
        ScriptedTransport::commandForward(transport.writes[index]);
    if (!replay.transitionAllowed(forward,
                                  transport.write_receipts[index]) ||
        !replay.noteSuccessfulNormalWrite(
            forward, transport.write_receipts[index])) {
      return false;
    }
  }
  return true;
}

void testProfileDefinitionsAndIndependentEncoder() {
  std::set<std::string> names;
  std::size_t straight_count = 0U;
  std::size_t legacy_turn_count = 0U;
  std::size_t outer_tier_turn_count = 0U;
  for (std::size_t index = 0U;
       index < wheeltec::kRawProfileFixedProfileCount; ++index) {
    wheeltec::RawProfile profile = wheeltec::RawProfile::kDisabled;
    expect(wheeltec::rawProfileAt(index, &profile),
           "every fixed-profile catalog index resolves");
    wheeltec::RawProfileDefinition definition;
    expect(wheeltec::rawProfileDefinition(profile, &definition),
           "each fixed raw profile has a definition");
    const std::string name(wheeltec::rawProfileName(profile));
    wheeltec::RawProfile parsed = wheeltec::RawProfile::kDisabled;
    expect(names.insert(name).second &&
               wheeltec::rawProfileFromName(name, &parsed) &&
               parsed == profile && definition.name == name &&
               definition.hold_duration_ns ==
                   wheeltec::kRawProfileTargetHoldDurationNs,
           "fixed names are unique and parse/name round-trip exactly");
    expect(definition.target_forward_wire > 0 &&
               definition.target_forward_wire <=
                   wheeltec::kRawProfileMaximumCommandForwardWire,
           "fixed profile center command remains inside the independent 6 m/s cap");
    const wheeltec::RawProfileEncodeResult target =
        wheeltec::encodeRawProfileCommand(
            profile, definition.target_forward_wire);
    expect(target.ok() &&
               target.forward_wire == definition.target_forward_wire &&
               target.yaw_wire == definition.target_yaw_wire,
           "every fixed profile target has one matching independent integer encoding");
    if (definition.turn_radius_m == 0.0) {
      ++straight_count;
    } else if (definition.speed_tier_semantics ==
               wheeltec::RawProfileSpeedTierSemantics::
                   kLegacyCenterForwardCommand) {
      ++legacy_turn_count;
    } else {
      ++outer_tier_turn_count;
    }
  }
  wheeltec::RawProfile ignored = wheeltec::RawProfile::kDisabled;
  expect(names.size() == wheeltec::kRawProfileFixedProfileCount &&
             straight_count == 12U && legacy_turn_count == 4U &&
             outer_tier_turn_count == 48U &&
             !wheeltec::rawProfileAt(
                 wheeltec::kRawProfileFixedProfileCount, &ignored) &&
             !wheeltec::rawProfileFromName("straight-6.1", &ignored) &&
             !wheeltec::rawProfileFromName("disabled", &ignored) &&
             !wheeltec::rawProfileFromName("left-6.0-r0.95", &ignored),
         "catalog is closed over 12 straights, four legacy turns, and 48 explicit outer-tier turns");

  for (std::size_t tier_index = 0U; tier_index < 12U; ++tier_index) {
    wheeltec::RawProfile profile = wheeltec::RawProfile::kDisabled;
    wheeltec::RawProfileDefinition definition;
    expect(wheeltec::rawProfileAt(tier_index, &profile) &&
               wheeltec::rawProfileDefinition(profile, &definition) &&
               definition.turn_radius_m == 0.0 &&
               definition.speed_tier_wire ==
                   static_cast<std::int16_t>((tier_index + 1U) * 500U) &&
               definition.target_forward_wire == definition.speed_tier_wire &&
               definition.target_outer_wire_milli ==
                   static_cast<std::int64_t>(definition.speed_tier_wire) *
                       1000LL &&
               definition.target_inner_wire_milli ==
                   definition.target_outer_wire_milli,
           "straight catalog covers every 0.5 m/s tier through 6.0 m/s");
  }

  const wheeltec::RawProfileEncodeResult straight =
      wheeltec::encodeRawProfileCommand(
          wheeltec::RawProfile::kStraight6p0, 6000);
  expect(straight.ok() && straight.forward_wire == 6000 &&
             straight.yaw_wire == 0,
         "independent experimental encoder reaches 6 m/s without changing the production codec");
  const wheeltec::RawProfileEncodeResult left =
      wheeltec::encodeRawProfileCommand(
          wheeltec::RawProfile::kLeft0p5Radius0p95, 500);
  const wheeltec::RawProfileEncodeResult right =
      wheeltec::encodeRawProfileCommand(
          wheeltec::RawProfile::kRight0p5Radius0p95, 500);
  expect(left.ok() && right.ok() && left.yaw_wire == 526 &&
             right.yaw_wire == -526,
         "turn profiles encode w=v/R with explicit left/right sign");
  expect(!wheeltec::encodeRawProfileCommand(
              wheeltec::RawProfile::kStraight6p0, 6001)
              .ok(),
         "experimental encoder rejects commands above 6 m/s");
  wheeltec::WireMotionCandidate production_candidate;
  production_candidate.forward_speed_mps = 0.501;
  wheeltec::CodecLimits phase1_limits;
  phase1_limits.max_forward_speed_mps = 0.50;
  expect(!wheeltec::encodeWireMotion(production_candidate, phase1_limits)
              .ok(),
         "the explicit Phase-1 codec parameter independently retains its 0.50 m/s cap");
}

void testOuterTierTurnGridIntegerGeometry() {
  constexpr std::int64_t kHalfTrackNumerator = 161LL;
  constexpr std::int64_t kWireScale = 1000LL;
  std::size_t grid_count = 0U;
  std::set<std::string> geometry_cells;
  for (std::size_t index = 0U;
       index < wheeltec::kRawProfileFixedProfileCount; ++index) {
    wheeltec::RawProfile profile = wheeltec::RawProfile::kDisabled;
    wheeltec::RawProfileDefinition definition;
    if (!wheeltec::rawProfileAt(index, &profile) ||
        !wheeltec::rawProfileDefinition(profile, &definition) ||
        definition.speed_tier_semantics !=
            wheeltec::RawProfileSpeedTierSemantics::
                kOuterRearWheelCommandUpperLimit) {
      continue;
    }
    ++grid_count;
    const wheeltec::RawProfileEncodeResult encoded =
        wheeltec::encodeRawProfileCommand(
            profile, definition.target_forward_wire);
    const std::int64_t yaw_magnitude =
        std::abs(static_cast<std::int64_t>(encoded.yaw_wire));
    const std::int64_t outer_milli =
        static_cast<std::int64_t>(encoded.forward_wire) * kWireScale +
        yaw_magnitude * kHalfTrackNumerator;
    const std::int64_t inner_milli =
        static_cast<std::int64_t>(encoded.forward_wire) * kWireScale -
        yaw_magnitude * kHalfTrackNumerator;
    const std::int64_t tier_milli =
        static_cast<std::int64_t>(definition.speed_tier_wire) * kWireScale;
    const std::int16_t next_center =
        static_cast<std::int16_t>(definition.target_forward_wire + 1);
    const std::int64_t next_yaw_magnitude = definition.turn_radius_m == 2.0
        ? static_cast<std::int64_t>(next_center) / 2LL
        : static_cast<std::int64_t>(next_center) * 20LL / 19LL;
    const std::int64_t next_outer_milli =
        static_cast<std::int64_t>(next_center) * kWireScale +
        next_yaw_magnitude * kHalfTrackNumerator;
    bool complete_ramp_geometry_is_safe = true;
    for (std::int16_t center = 0;
         center <= definition.target_forward_wire; ++center) {
      const wheeltec::RawProfileEncodeResult ramp =
          wheeltec::encodeRawProfileCommand(profile, center);
      const std::int64_t ramp_yaw =
          std::abs(static_cast<std::int64_t>(ramp.yaw_wire));
      const std::int64_t ramp_outer =
          static_cast<std::int64_t>(center) * kWireScale +
          ramp_yaw * kHalfTrackNumerator;
      const std::int64_t ramp_inner =
          static_cast<std::int64_t>(center) * kWireScale -
          ramp_yaw * kHalfTrackNumerator;
      if (!ramp.ok() || ramp_outer > tier_milli || ramp_inner < 0) {
        complete_ramp_geometry_is_safe = false;
        break;
      }
    }
    std::ostringstream cell;
    cell << (definition.curvature_inv_m > 0.0 ? "left" : "right")
         << ':' << definition.turn_radius_m << ':'
         << definition.speed_tier_wire;
    const std::string profile_name(definition.name);
    const bool name_and_yaw_sign_agree =
        (profile_name.compare(0U, 5U, "left-") == 0 &&
         encoded.yaw_wire > 0) ||
        (profile_name.compare(0U, 6U, "right-") == 0 &&
         encoded.yaw_wire < 0);
    expect(encoded.ok() && encoded.yaw_wire == definition.target_yaw_wire &&
               name_and_yaw_sign_agree &&
               definition.turn_radius_m >=
                               wheeltec::kRawProfileMinimumTurnRadiusM &&
               definition.speed_tier_wire >= 500 &&
               definition.speed_tier_wire <= 6000 &&
               definition.speed_tier_wire % 500 == 0 &&
               outer_milli <= tier_milli && inner_milli >= 0 &&
               next_outer_milli > tier_milli &&
               !wheeltec::encodeRawProfileCommand(profile, next_center).ok() &&
               complete_ramp_geometry_is_safe &&
               definition.target_outer_wire_milli == outer_milli &&
               definition.target_inner_wire_milli == inner_milli &&
               geometry_cells.insert(cell.str()).second,
           "each outer-tier turn uses the maximal safe integer center wire and records exact derived geometry");
  }
  expect(grid_count == 48U && geometry_cells.size() == 48U,
         "outer-tier grid contains left/right, both radii, and all twelve tiers exactly once");

  for (std::size_t radius_index = 0U; radius_index < 2U; ++radius_index) {
    for (std::size_t tier_index = 0U; tier_index < 12U; ++tier_index) {
      const std::size_t left_catalog_index =
          16U + radius_index * 24U + tier_index;
      const std::size_t right_catalog_index = left_catalog_index + 12U;
      wheeltec::RawProfile left_profile = wheeltec::RawProfile::kDisabled;
      wheeltec::RawProfile right_profile = wheeltec::RawProfile::kDisabled;
      wheeltec::RawProfileDefinition left_definition;
      wheeltec::RawProfileDefinition right_definition;
      wheeltec::rawProfileAt(left_catalog_index, &left_profile);
      wheeltec::rawProfileAt(right_catalog_index, &right_profile);
      wheeltec::rawProfileDefinition(left_profile, &left_definition);
      wheeltec::rawProfileDefinition(right_profile, &right_definition);
      const wheeltec::RawProfileEncodeResult left =
          wheeltec::encodeRawProfileCommand(
              left_profile, left_definition.target_forward_wire);
      const wheeltec::RawProfileEncodeResult right =
          wheeltec::encodeRawProfileCommand(
              right_profile, right_definition.target_forward_wire);
      expect(left.ok() && right.ok() &&
                 left_definition.target_forward_wire ==
                     right_definition.target_forward_wire &&
                 left_definition.target_outer_wire_milli ==
                     right_definition.target_outer_wire_milli &&
                 left_definition.target_inner_wire_milli ==
                     right_definition.target_inner_wire_milli &&
                 left.forward_wire == right.forward_wire &&
                 left.yaw_wire == -right.yaw_wire,
             "every left/right outer-tier cell is an exact integer mirror pair");
    }
  }

  const wheeltec::RawProfile left_profiles[] = {
      wheeltec::RawProfile::kLeftOuter0p5Radius2p0,
      wheeltec::RawProfile::kLeftOuter6p0Radius2p0,
      wheeltec::RawProfile::kLeftOuter0p5Radius0p95,
      wheeltec::RawProfile::kLeftOuter6p0Radius0p95,
  };
  const wheeltec::RawProfile right_profiles[] = {
      wheeltec::RawProfile::kRightOuter0p5Radius2p0,
      wheeltec::RawProfile::kRightOuter6p0Radius2p0,
      wheeltec::RawProfile::kRightOuter0p5Radius0p95,
      wheeltec::RawProfile::kRightOuter6p0Radius0p95,
  };
  for (std::size_t index = 0U; index < 4U; ++index) {
    wheeltec::RawProfileDefinition left_definition;
    wheeltec::RawProfileDefinition right_definition;
    wheeltec::rawProfileDefinition(left_profiles[index], &left_definition);
    wheeltec::rawProfileDefinition(right_profiles[index], &right_definition);
    const wheeltec::RawProfileEncodeResult left =
        wheeltec::encodeRawProfileCommand(
            left_profiles[index], left_definition.target_forward_wire);
    const wheeltec::RawProfileEncodeResult right =
        wheeltec::encodeRawProfileCommand(
            right_profiles[index], right_definition.target_forward_wire);
    expect(left.ok() && right.ok() &&
               left_definition.target_forward_wire ==
                   right_definition.target_forward_wire &&
               left_definition.speed_tier_wire ==
                   right_definition.speed_tier_wire &&
               left.forward_wire == right.forward_wire &&
               left.yaw_wire == -right.yaw_wire,
           "left/right outer-tier profiles are exact integer mirror images");
  }

  const wheeltec::RawProfile legacy_profiles[] = {
      wheeltec::RawProfile::kLeft0p5Radius2p0,
      wheeltec::RawProfile::kRight0p5Radius2p0,
      wheeltec::RawProfile::kLeft0p5Radius0p95,
      wheeltec::RawProfile::kRight0p5Radius0p95,
  };
  for (const wheeltec::RawProfile profile : legacy_profiles) {
    wheeltec::RawProfileDefinition definition;
    expect(wheeltec::rawProfileDefinition(profile, &definition) &&
               definition.speed_tier_semantics ==
                   wheeltec::RawProfileSpeedTierSemantics::
                       kLegacyCenterForwardCommand &&
               definition.target_forward_wire == 500 &&
               std::string(definition.name).find("outer") ==
                   std::string::npos,
           "legacy center-0.5 turn profiles retain their names and command semantics");
  }
}

void testMetadataDistinguishesLegacyCenterAndOuterTierCommands() {
  const wheeltec::RawProfile profiles[] = {
      wheeltec::RawProfile::kLeft0p5Radius0p95,
      wheeltec::RawProfile::kLeftOuter6p0Radius0p95,
  };
  for (const wheeltec::RawProfile profile : profiles) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0), 10U);
    RecordState records;
    records.stop_after_calls = 1U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(profile), operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kInterrupted &&
               result.zero_host_write_completed &&
               records.metadata_line.find(
                   "\"target_center_forward_wire\":") !=
                   std::string::npos &&
               records.metadata_line.find("\"target_yaw_wire\":") !=
                   std::string::npos &&
               records.metadata_line.find(
                   "\"target_outer_command_mps\":") !=
                   std::string::npos &&
               records.metadata_line.find(
                   "\"target_inner_command_mps\":") !=
                   std::string::npos &&
               records.metadata_line.find(
                   "\"derived_rear_track_mm\":322") !=
                   std::string::npos,
           "turn metadata always records actual integer center/yaw and derived outer/inner commands");
    if (profile == wheeltec::RawProfile::kLeft0p5Radius0p95) {
      expect(records.metadata_line.find(
                 "\"speed_tier_semantics\":\"legacy_center_forward_command\"") !=
                     std::string::npos &&
                 records.metadata_line.find(
                     "\"target_center_forward_wire\":500") !=
                     std::string::npos,
             "legacy turn metadata preserves center-command semantics");
    } else {
      expect(records.metadata_line.find(
                 "\"speed_tier_semantics\":\"outer_rear_wheel_command_upper_limit\"") !=
                     std::string::npos &&
                 records.metadata_line.find(
                     "\"speed_tier_wire\":6000") != std::string::npos &&
                 records.metadata_line.find(
                     "\"target_center_forward_wire\":5130") !=
                     std::string::npos &&
                 records.metadata_line.find(
                     "\"target_yaw_wire\":5400") != std::string::npos,
             "outer-tier metadata records tier and the maximal encoded center/yaw pair");
    }
  }
}

void testStrictIntegerAccelerationEnvelope() {
  wheeltec::RawProfileWireAccelerationEnvelope envelope;
  constexpr std::int64_t kStart = 1000000000;
  expect(!envelope.transitionAllowed(1, kStart) &&
             envelope.transitionAllowed(0, kStart) &&
             envelope.noteSuccessfulNormalWrite(0, kStart),
         "normal writes establish an exact-zero baseline before motion");
  expect(!envelope.transitionAllowed(4, kStart + 19999999) &&
             envelope.transitionAllowed(4, kStart + 20000000),
         "integer slew boundary is exact at 0.20 m/s^2");
  std::int16_t coupled = -1;
  expect(envelope.coupleToward(6000, kStart + 17000000, &coupled) &&
             coupled == 3 && envelope.lastSuccessfulForwardWire() == 0,
         "coupling rounds down and an attempted or failed write cannot advance the baseline");
  expect(envelope.noteSuccessfulNormalWrite(3, kStart + 17000000) &&
             envelope.coupleToward(0, kStart + 22000000, &coupled) &&
             coupled == 2,
         "the same strict envelope applies to ramp down");
}

void testAuthorizationAndEvidenceFailBeforeIo() {
  expect(!wheeltec::rawProfileConfigIsValid(wheeltec::RawProfileConfig{}),
         "raw-profile actuation remains disabled and profile-free by default");
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  RecordState records;
  wheeltec::RawProfileConfig config =
      validConfig(wheeltec::RawProfile::kStraight0p5);
  config.raw_raised_bench_opt_in = false;
  wheeltec::WheeltecRawProfileSession session;
  wheeltec::RawProfileResult result =
      session.run(&transport, config, operations(&clock, &records));
  expect(result.status == wheeltec::RawProfileStatus::kAuthorizationDenied &&
             transport.writes.empty() && records.count == 0U,
         "distinct raised-bench gate fails before evidence I/O");

  config = validConfig(wheeltec::RawProfile::kStraight0p5);
  config.exact_zero_evidence_token.clear();
  result = session.run(&transport, config, operations(&clock, &records));
  expect(result.status == wheeltec::RawProfileStatus::kEvidenceMissing &&
             transport.writes.empty() && records.count == 0U,
         "both predecessor evidence hashes are mandatory before I/O");
}

void testPreallocatedEvidenceBufferFailsClosedAtCapacity() {
  wheeltec::RawProfileEvidenceBuffer invalid(
      wheeltec::kRawProfileEvidenceBufferCapacityBytes + 1U);
  expect(!invalid.prepare(),
         "the active evidence buffer cannot exceed its reviewed fixed capacity");

  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  addFeedback(&transport, feedback(0));
  wheeltec::RawProfileEvidenceBuffer evidence(4096U);
  expect(evidence.prepare(),
         "a bounded test evidence buffer is fully reserved before I/O");
  RecordState records;
  wheeltec::RawProfileOperations ops = operations(&clock, &records);
  ops.record_json_line = [&evidence](const std::string& line) {
    return evidence.appendLine(line);
  };
  wheeltec::WheeltecRawProfileSession session;
  const wheeltec::RawProfileResult result = session.run(
      &transport, validConfig(wheeltec::RawProfile::kStraight0p5), ops);
  expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
             evidence.failed() && !transport.writes.empty() &&
             isExactZero(transport.writes.back()),
         "buffer capacity exhaustion stops normal evidence production and cannot block bounded exact zero");
}

void testOriginalEightProfilesCompleteWithSixtySecondHoldAndBoundedZero() {
  const std::vector<wheeltec::RawProfile> profiles = {
      wheeltec::RawProfile::kStraight0p5,
      wheeltec::RawProfile::kStraight1p0,
      wheeltec::RawProfile::kStraight1p5,
      wheeltec::RawProfile::kStraight2p0,
      wheeltec::RawProfile::kLeft0p5Radius2p0,
      wheeltec::RawProfile::kRight0p5Radius2p0,
      wheeltec::RawProfile::kLeft0p5Radius0p95,
      wheeltec::RawProfile::kRight0p5Radius0p95,
  };
  for (const wheeltec::RawProfile profile : profiles) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(3000, -2500, 3000));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(profile), operations(&clock, &records));
    wheeltec::RawProfileDefinition definition;
    wheeltec::rawProfileDefinition(profile, &definition);
    std::int16_t maximum = 0;
    std::int16_t first_nonzero = 0;
    bool yaw_sign_ok = true;
    std::int64_t first_target_ns = 0;
    std::int64_t first_ramp_down_ns = 0;
    std::int16_t previous_forward = 0;
    std::size_t write_index = 0U;
    for (const std::vector<std::uint8_t>& frame : transport.writes) {
      const std::int16_t forward =
          ScriptedTransport::commandForward(frame);
      maximum = std::max(maximum, forward);
      if (forward > 0 && first_nonzero == 0) {
        first_nonzero = forward;
      }
      const std::int16_t yaw = ScriptedTransport::commandYaw(frame);
      if (definition.curvature_inv_m > 0.0 && yaw < 0) {
        yaw_sign_ok = false;
      }
      if (definition.curvature_inv_m < 0.0 && yaw > 0) {
        yaw_sign_ok = false;
      }
      if (write_index <
              result.statistics.normal_tx_host_writes_completed &&
          forward == definition.target_forward_wire &&
          first_target_ns == 0) {
        first_target_ns = transport.write_receipts[write_index];
      }
      if (write_index <
              result.statistics.normal_tx_host_writes_completed &&
          first_target_ns > 0 && previous_forward ==
              definition.target_forward_wire &&
          forward < definition.target_forward_wire &&
          first_ramp_down_ns == 0) {
        first_ramp_down_ns = transport.write_receipts[write_index];
      }
      previous_forward = forward;
      ++write_index;
    }
    expect(result.completed() && maximum == definition.target_forward_wire &&
               first_nonzero > 0 && first_nonzero <= 4 &&
               yaw_sign_ok && !transport.writes.empty() &&
               isExactZero(transport.writes.back()) &&
               result.zero_host_write_completed &&
               strictEnvelopeHolds(
                   transport,
                   static_cast<std::size_t>(
                       result.statistics.normal_tx_host_writes_completed)),
           "each original profile completes its virtual ramp/60s hold/ramp and finishes with exact zero");
    expect(first_target_ns > 0 && first_ramp_down_ns > first_target_ns &&
               first_ramp_down_ns - first_target_ns >=
                   wheeltec::kRawProfileTargetHoldDurationNs,
           "ramp-down cannot begin until the fixed sixty-second target hold has elapsed");
    expect(records.saw_metadata && records.saw_record_only_semantics &&
               records.observation_count > 3000U,
           "extreme feedback, asymmetry, tracking error, and post-zero tail are recorded but never acceptance-gated");
    expect(records.recorded_bytes < 16U * 1024U * 1024U,
           "the original-eight one-frame-per-cycle fixtures remain below half the CLI's preallocated 32 MiB evidence buffer");
    expect(result.statistics.ended_monotonic_ns -
                   result.statistics.started_monotonic_ns >=
               60000000000LL,
           "each original profile actually includes a sixty-second hold");
  }
}

void testLongestSixMeterProfileFitsTimeAndEvidenceBounds() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  addFeedback(&transport, feedback(0), 8000U);
  RecordState records;
  wheeltec::RawProfileConfig config =
      validConfig(wheeltec::RawProfile::kStraight6p0);
  expect(config.maximum_session_duration_ns ==
             wheeltec::kRawProfileMaximumSessionDurationNs &&
             wheeltec::rawProfileConfigIsValid(config),
         "the fixed 130 second session bound admits the longest profile");
  wheeltec::RawProfileConfig overlong = config;
  ++overlong.maximum_session_duration_ns;
  expect(!wheeltec::rawProfileConfigIsValid(overlong),
         "the session configuration rejects one nanosecond above 130 seconds");

  wheeltec::WheeltecRawProfileSession session;
  const wheeltec::RawProfileResult result =
      session.run(&transport, config, operations(&clock, &records));
  g_longest_profile_evidence_bytes = records.recorded_bytes;
  g_longest_profile_duration_ns =
      result.statistics.ended_monotonic_ns -
      result.statistics.started_monotonic_ns;
  std::int64_t first_target_ns = 0;
  std::int64_t first_ramp_down_ns = 0;
  std::int64_t first_post_zero_ns = 0;
  std::int16_t previous_forward = 0;
  const std::size_t normal_writes = static_cast<std::size_t>(
      result.statistics.normal_tx_host_writes_completed);
  for (std::size_t index = 0U;
       index < normal_writes && index < transport.writes.size(); ++index) {
    const std::int16_t forward =
        ScriptedTransport::commandForward(transport.writes[index]);
    if (first_target_ns == 0 && forward == 6000) {
      first_target_ns = transport.write_receipts[index];
    }
    if (first_target_ns > 0 && first_ramp_down_ns == 0 &&
        previous_forward == 6000 && forward < 6000) {
      first_ramp_down_ns = transport.write_receipts[index];
    }
    if (first_ramp_down_ns > 0 && first_post_zero_ns == 0 &&
        previous_forward > 0 && forward == 0) {
      first_post_zero_ns = transport.write_receipts[index];
    }
    previous_forward = forward;
  }
  constexpr std::uint64_t kConservativeCliPrefixBufferOverheadBytes =
      64U * 1024U;
  constexpr std::uint64_t kMinimumEvidenceMarginBytes =
      15U * 1024U * 1024U;
  expect(result.completed() && result.zero_host_write_completed &&
             result.statistics.maximum_commanded_forward_wire == 6000 &&
             first_target_ns > 0 && first_ramp_down_ns > first_target_ns &&
             first_ramp_down_ns - first_target_ns >=
                 wheeltec::kRawProfileTargetHoldDurationNs &&
             first_post_zero_ns > first_ramp_down_ns &&
             result.statistics.ended_monotonic_ns -
                     result.statistics.started_monotonic_ns <
                 wheeltec::kRawProfileMaximumSessionDurationNs &&
             strictEnvelopeHolds(transport, normal_writes) &&
             !transport.writes.empty() &&
             isExactZero(transport.writes.back()),
         "6 m/s completes ramp-up, 60 second hold, ramp-down, post-zero, and final zero below 130 seconds");
  expect(records.recorded_bytes +
             kConservativeCliPrefixBufferOverheadBytes +
             kMinimumEvidenceMarginBytes <
             wheeltec::kRawProfileEvidenceBufferCapacityBytes,
         "the longest one-frame-per-cycle run plus conservative CLI-buffer overhead leaves at least 15 MiB free");
  expect(records.metadata_line.find(
             "\"speed_tier_semantics\":\"straight_center_equals_rear_wheels\"") !=
                 std::string::npos &&
             records.metadata_line.find(
                 "\"speed_tier_wire\":6000") != std::string::npos &&
             records.metadata_line.find(
                 "\"target_center_forward_wire\":6000") !=
                 std::string::npos &&
             records.metadata_line.find(
                 "\"target_outer_command_mps\":6") !=
                 std::string::npos &&
             records.metadata_line.find(
                 "\"target_inner_command_mps\":6") !=
                 std::string::npos,
         "metadata records tier semantics and actual encoded center/outer/inner commands");

  std::int64_t deadline_clock = 1000000000;
  ScriptedTransport deadline_transport(&deadline_clock);
  addFeedback(&deadline_transport, feedback(0), 8000U);
  RecordState deadline_records;
  wheeltec::RawProfileConfig short_config =
      validConfig(wheeltec::RawProfile::kStraight6p0);
  short_config.maximum_session_duration_ns = 120000000000LL;
  const wheeltec::RawProfileResult deadline_result = session.run(
      &deadline_transport, short_config,
      operations(&deadline_clock, &deadline_records));
  expect(deadline_result.status ==
             wheeltec::RawProfileStatus::kSessionDeadlineExceeded &&
             deadline_result.zero_host_write_completed &&
             deadline_result.statistics.maximum_commanded_forward_wire ==
                 6000 &&
             !deadline_transport.writes.empty() &&
             isExactZero(deadline_transport.writes.back()),
         "an insufficient 6 m/s session deadline fails closed through the exact-zero path");
}

void testParserNoiseIsPreservedButDoesNotReplaceFreshFeedback() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    wheeltec::FeedbackFrame corrupt = feedback(0);
    corrupt[22U] = static_cast<std::uint8_t>(corrupt[22U] ^ 0x01U);
    const wheeltec::FeedbackFrame valid = feedback(0);
    std::vector<std::uint8_t> noisy{0x11U, 0x22U};
    noisy.insert(noisy.end(), corrupt.begin(), corrupt.end());
    noisy.insert(noisy.end(), valid.begin(), valid.end());
    transport.reads.push_back(std::move(noisy));
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.completed() &&
               result.statistics.parser_checksum_failures > 0U &&
               result.statistics.parser_discarded_bytes > 0U &&
               records.raw_chunk_count > 0U &&
               records.parser_error_event_count > 0U,
           "checksum/framing noise is preserved and resynchronized without becoming an online feedback-value gate");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    for (std::size_t index = 0U; index < 100U; ++index) {
      transport.reads.push_back({0x11U, 0x22U, 0x33U});
    }
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kFeedbackMissing &&
               result.statistics.valid_feedback_frames == 0U &&
               result.statistics.parser_discarded_bytes > 0U &&
               isExactZero(transport.writes.back()),
           "parser noise never refreshes the valid-feedback watchdog and still exits through exact zero");
  }
}

void testFlagStopAndTransportFailuresStopAndAttemptZero() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0, 0, 0, 1U), 100U);
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kControlInhibited &&
               !transport.writes.empty() &&
               isExactZero(transport.writes.back()),
           "FlagStop inhibits the experiment and still takes the bounded zero path");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.partial_first_nonzero = true;
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kWriteFailed &&
               transport.partial_triggered &&
               result.delivery_unconfirmed &&
               !result.zero_host_write_completed &&
               transport.writes.size() ==
                   transport.writes_when_partial_triggered &&
               !isExactZero(transport.writes.back()),
           "a partial normal command poisons the stream and forbids every appended zero frame");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.throw_first_nonzero_write = true;
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
               transport.nonzero_throw_triggered &&
               result.delivery_unconfirmed &&
               !result.zero_host_write_completed &&
               transport.writes.size() ==
                   transport.writes_when_nonzero_throw_triggered &&
               !isExactZero(transport.writes.back()),
           "a throwing normal write poisons the stream and the catch path cannot append a zero frame");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.scripted_zero_write_results.push_back(
        {wheeltec::TransportStatus::kIoError, 1U, EIO, true});
    RecordState records;
    records.fail_at = 1U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
               result.delivery_unconfirmed &&
               !result.zero_host_write_completed &&
               transport.writes.size() == 1U &&
               transport.io_order == std::vector<char>{'W'},
           "a partial bounded-zero write poisons the stream and is never retried");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.fail_at = 40U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
               !transport.writes.empty() &&
               isExactZero(transport.writes.back()),
           "evidence sink failure stops motion and cannot block the direct zero write");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    RecordState records;
    records.fail_at = 1U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
               result.statistics.nonzero_frame_host_writes_completed == 0U &&
               !transport.writes.empty() &&
               isExactZero(transport.writes.back()),
           "metadata evidence failure before motion still invokes direct exact zero");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.zero_failures_remaining = 2U;
    std::uint64_t callback_count = 0U;
    std::size_t writes_seen_after_retries = 0U;
    wheeltec::RawProfileOperations ops;
    ops.monotonic_now_ns = [&clock]() { return clock; };
    ops.wait_until_monotonic_ns = [&clock](std::int64_t deadline) {
      if (clock < deadline) {
        clock = deadline;
      }
      return true;
    };
    ops.record_json_line =
        [&callback_count, &writes_seen_after_retries, &transport](
            const std::string&) {
          ++callback_count;
          if (callback_count > 1U) {
            writes_seen_after_retries = transport.writes.size();
          }
          return false;
        };
    ops.stop_requested = []() { return false; };
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5), ops);
    expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
               result.zero_host_write_completed &&
               transport.writes.size() == 3U &&
               writes_seen_after_retries == 3U,
           "all bounded zero retries finish before any best-effort emergency evidence callback can block them");
  }
}

void testClockWatchdogDisconnectAndSignalFailuresUseZeroPath() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.jump_at_wait_call = 1U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kCommandWatchdogExpired &&
               !transport.writes.empty() &&
               isExactZero(transport.writes.back()),
           "scheduler gap above 100 ms trips the command watchdog and exact-zero path");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.rollback_at_now_call = 40U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kClockInvalid &&
               !transport.writes.empty() &&
               isExactZero(transport.writes.back()),
           "monotonic rollback is fatal and uses direct exact zero");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.disconnect_first_nonzero = true;
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kDisconnected &&
               transport.disconnect_triggered &&
               !result.zero_host_write_completed &&
               result.delivery_unconfirmed,
           "serial disconnect is fatal and explicitly leaves zero delivery unconfirmed");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.stop_after_calls = 30U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kInterrupted &&
               result.statistics.nonzero_frame_host_writes_completed > 0U &&
               isExactZero(transport.writes.back()),
           "caught-signal stop request during motion uses the same bounded exact-zero path");
  }
}

void testWatchdogAndSessionDeadlineCannotBeSplitAcrossIo() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.read_delays_ns = {0, 20000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.jump_at_wait_call = 2U;
    records.wait_jump_ns = 79000000;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kCommandWatchdogExpired &&
               result.statistics.normal_tx_host_writes_completed == 2U &&
               isExactZero(transport.writes.back()),
           "99 ms before-read gap plus a 20 ms read cannot bypass the 100 ms completion watchdog");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.jump_at_wait_call = 2U;
    records.wait_jump_ns = 80000000;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.completed(),
           "an exact 100 ms successful-completion interval remains inclusive");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.write_durations_ns = {0, 101000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kCommandWatchdogExpired &&
               result.statistics.normal_tx_host_writes_completed == 2U &&
               isExactZero(transport.writes.back()),
           "a full host write completing beyond 100 ms is counted as occurred, never advances the baseline, and immediately triggers exact zero");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.write_durations_ns = {10000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.completed(),
           "a full write completing exactly at its 10 ms deadline is inclusive");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.read_delays_ns = {0, 20000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::RawProfileConfig config =
        validConfig(wheeltec::RawProfile::kStraight0p5);
    config.maximum_session_duration_ns = 50000000;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result =
        session.run(&transport, config, operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kSessionDeadlineExceeded &&
               result.statistics.normal_tx_host_writes_completed == 2U &&
               isExactZero(transport.writes.back()),
           "a read begun before but received after the hard session deadline cannot issue another normal command");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::RawProfileConfig config =
        validConfig(wheeltec::RawProfile::kStraight0p5);
    config.maximum_session_duration_ns = 40000000;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result =
        session.run(&transport, config, operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kSessionDeadlineExceeded &&
               result.statistics.normal_tx_host_writes_completed == 2U,
           "the exact session deadline is exclusive for starting normal TX");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.write_durations_ns = {0, 16000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::RawProfileConfig config =
        validConfig(wheeltec::RawProfile::kStraight0p5);
    config.maximum_session_duration_ns = 35000000;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result =
        session.run(&transport, config, operations(&clock, &records));
    expect(result.status ==
               wheeltec::RawProfileStatus::kSessionDeadlineExceeded &&
               result.statistics.normal_tx_host_writes_completed == 2U &&
               isExactZero(transport.writes.back()),
           "a write that physically completes beyond the session deadline is counted, aborts immediately, and cannot advance the baseline");
  }
}

void testPrearmCountsDistinctReceiptsNotFrames() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    const wheeltec::FeedbackFrame frame = feedback(0);
    std::vector<std::uint8_t> batch;
    for (std::size_t index = 0U; index < 5U; ++index) {
      batch.insert(batch.end(), frame.begin(), frame.end());
    }
    transport.reads.push_back(std::move(batch));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kFeedbackMissing &&
               result.statistics.valid_feedback_frames == 5U &&
               result.statistics.nonzero_frame_host_writes_completed == 0U,
           "five valid frames in one chunk are one receipt and cannot arm motion");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.read_delays_ns =
        {0, 30000000, 30000000, 30000000, 30000000};
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.completed() &&
               result.statistics.nonzero_frame_host_writes_completed > 0U,
           "five strictly increasing valid receipts spanning exactly 0.2 s can arm the fixed profile");
  }
}

void testInitialAndPrearmIoAreExactZeroBeforeAnyReadOrMotion() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.scripted_zero_write_results.push_back(
        {wheeltec::TransportStatus::kIoError, 0U, EIO, false});
    transport.scripted_zero_write_results.push_back(
        {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
         false});
    RecordState records;
    wheeltec::RawProfileOperations ops = operations(&clock, &records);
    ops.record_json_line = [](const std::string&) -> bool {
      throw std::bad_alloc();
    };
    const wheeltec::RawProfileEncodeResult zero =
        wheeltec::encodeRawProfileCommand(
            wheeltec::RawProfile::kStraight0p5, 0);
    const wheeltec::RawProfileImmediateZeroResult result =
        wheeltec::writeRawProfileImmediateZeroNoRecord(
            &transport, zero.frame, ops);
    expect(result.zero_host_write_completed && !result.delivery_unconfirmed &&
               result.terminal_transport_status ==
                   wheeltec::TransportStatus::kOk &&
               result.attempts == 3U &&
               transport.writes.size() == 3U &&
               transport.io_order == std::vector<char>{'W', 'W', 'W'},
           "the post-open startup helper performs all bounded exact-zero attempts without invoking evidence allocation or reads");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.scripted_zero_write_results.push_back(
        {wheeltec::TransportStatus::kIoError, 1U, EIO, false});
    RecordState records;
    const wheeltec::RawProfileEncodeResult zero =
        wheeltec::encodeRawProfileCommand(
            wheeltec::RawProfile::kStraight0p5, 0);
    const wheeltec::RawProfileImmediateZeroResult result =
        wheeltec::writeRawProfileImmediateZeroNoRecord(
            &transport, zero.frame, operations(&clock, &records));
    expect(!result.zero_host_write_completed && result.delivery_unconfirmed &&
               result.attempts == 1U && transport.writes.size() == 1U &&
               transport.io_order == std::vector<char>{'W'},
           "a startup-zero partial transfer is terminal and never appends a retry frame");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.scripted_zero_write_results.push_back(
        {wheeltec::TransportStatus::kIoError, 0U, EIO, true});
    RecordState records;
    const wheeltec::RawProfileEncodeResult zero =
        wheeltec::encodeRawProfileCommand(
            wheeltec::RawProfile::kStraight0p5, 0);
    const wheeltec::RawProfileImmediateZeroResult result =
        wheeltec::writeRawProfileImmediateZeroNoRecord(
            &transport, zero.frame, operations(&clock, &records));
    expect(!result.zero_host_write_completed && result.delivery_unconfirmed &&
               result.attempts == 1U && transport.writes.size() == 1U &&
               transport.io_order == std::vector<char>{'W'},
           "an unconfirmed zero-byte startup write is terminal and never retries");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.throw_first_zero_write = true;
    RecordState records;
    const wheeltec::RawProfileEncodeResult zero =
        wheeltec::encodeRawProfileCommand(
            wheeltec::RawProfile::kStraight0p5, 0);
    const wheeltec::RawProfileImmediateZeroResult result =
        wheeltec::writeRawProfileImmediateZeroNoRecord(
            &transport, zero.frame, operations(&clock, &records));
    expect(!result.zero_host_write_completed && result.delivery_unconfirmed &&
               result.terminal_transport_status ==
                   wheeltec::TransportStatus::kIoError &&
               result.attempts == 1U && transport.zero_throw_triggered &&
               transport.writes.size() == 1U &&
               transport.io_order == std::vector<char>{'W'},
           "a throwing startup write is contained and never appends a retry frame");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0));
    RecordState records;
    records.stop_after_calls = 8U;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight2p0),
        operations(&clock, &records));
    bool prearm_only_zero = true;
    for (const std::vector<std::uint8_t>& frame : transport.writes) {
      if (!isExactZero(frame)) {
        prearm_only_zero = false;
      }
    }
    expect(result.status == wheeltec::RawProfileStatus::kInterrupted &&
               !transport.io_order.empty() &&
               transport.io_order.front() == 'W' &&
               !transport.writes.empty() &&
               prearm_only_zero,
           "the first transport I/O is exact zero and all prearm writes remain zero");
  }
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.zero_failures_remaining = 1U;
    addFeedback(&transport, feedback(0));
    RecordState records;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(wheeltec::RawProfile::kStraight0p5),
        operations(&clock, &records));
    expect(result.status == wheeltec::RawProfileStatus::kWriteFailed &&
               transport.io_order.size() >= 2U &&
               transport.io_order[0U] == 'W' &&
               transport.io_order[1U] == 'W' &&
               std::find(transport.io_order.begin(),
                         transport.io_order.end(), 'R') ==
                   transport.io_order.end() &&
               result.zero_host_write_completed,
           "a failed first exact-zero host write prevents every read and motion command and immediately retries bounded zero");
  }
}

void testActiveSessionExceptionCannotEscapeWithoutExactZero() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  addFeedback(&transport, feedback(0));
  RecordState records;
  wheeltec::RawProfileOperations ops = operations(&clock, &records);
  std::uint64_t callback_count = 0U;
  ops.record_json_line = [&callback_count](const std::string&) {
    ++callback_count;
    if (callback_count == 120U) {
      throw std::bad_alloc();
    }
    return true;
  };
  wheeltec::WheeltecRawProfileSession session;
  const wheeltec::RawProfileResult result = session.run(
      &transport, validConfig(wheeltec::RawProfile::kStraight0p5), ops);
  bool saw_nonzero = false;
  for (const std::vector<std::uint8_t>& frame : transport.writes) {
    if (ScriptedTransport::commandForward(frame) > 0) {
      saw_nonzero = true;
    }
  }
  expect(result.status == wheeltec::RawProfileStatus::kRecordError &&
             saw_nonzero && result.zero_host_write_completed &&
             isExactZero(transport.writes.back()),
         "an allocation exception after nonzero motion is contained and invokes no-record bounded exact zero");
}

}  // namespace

int main(int argc, char** argv) {
  if ((argc == 2 || argc == 3) &&
      (std::string(argv[1]) == "--emit-minimal-sample" ||
       std::string(argv[1]) == "--emit-full-sample")) {
    const bool emit_full = std::string(argv[1]) == "--emit-full-sample";
    wheeltec::RawProfile sample_profile =
        wheeltec::RawProfile::kStraight0p5;
    if (argc == 3 &&
        !wheeltec::rawProfileFromName(argv[2], &sample_profile)) {
      return 2;
    }
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, feedback(0), 8000U);
    RecordState records;
    records.capture_all = emit_full;
    wheeltec::WheeltecRawProfileSession session;
    const wheeltec::RawProfileResult result = session.run(
        &transport, validConfig(sample_profile),
        operations(&clock, &records));
    if (!result.completed() || records.metadata_line.empty() ||
        records.feedback_line.empty() || records.normal_tx_line.empty()) {
      return 2;
    }
    if (emit_full) {
      for (const std::string& line : records.all_lines) {
        std::printf("%s\n", line.c_str());
      }
      std::printf("%s\n",
                  wheeltec::rawProfileSummaryRecordJson(result).c_str());
    } else {
      std::printf("%s\n%s\n%s\n%s\n", records.metadata_line.c_str(),
                  records.normal_tx_line.c_str(),
                  records.feedback_line.c_str(),
                  wheeltec::rawProfileSummaryRecordJson(result).c_str());
    }
    return 0;
  }
  runTest(testProfileDefinitionsAndIndependentEncoder);
  runTest(testOuterTierTurnGridIntegerGeometry);
  runTest(testMetadataDistinguishesLegacyCenterAndOuterTierCommands);
  runTest(testStrictIntegerAccelerationEnvelope);
  runTest(testAuthorizationAndEvidenceFailBeforeIo);
  runTest(testPreallocatedEvidenceBufferFailsClosedAtCapacity);
  runTest(testOriginalEightProfilesCompleteWithSixtySecondHoldAndBoundedZero);
  runTest(testLongestSixMeterProfileFitsTimeAndEvidenceBounds);
  runTest(testParserNoiseIsPreservedButDoesNotReplaceFreshFeedback);
  runTest(testFlagStopAndTransportFailuresStopAndAttemptZero);
  runTest(testClockWatchdogDisconnectAndSignalFailuresUseZeroPath);
  runTest(testWatchdogAndSessionDeadlineCannotBeSplitAcrossIo);
  runTest(testPrearmCountsDistinctReceiptsNotFrames);
  runTest(testInitialAndPrearmIoAreExactZeroBeforeAnyReadOrMotion);
  runTest(testActiveSessionExceptionCannotEscapeWithoutExactZero);
  if (g_failures != 0) {
    std::fprintf(stderr, "%d failure(s) across %d test case(s)\n", g_failures,
                 g_test_cases_run);
    return 1;
  }
  std::printf(
      "PASS: %d raw-profile test cases; straight-6.0 evidence=%llu bytes duration=%lld ns buffer_margin=%llu bytes\n",
      g_test_cases_run,
      static_cast<unsigned long long>(g_longest_profile_evidence_bytes),
      static_cast<long long>(g_longest_profile_duration_ns),
      static_cast<unsigned long long>(
          wheeltec::kRawProfileEvidenceBufferCapacityBytes -
          g_longest_profile_evidence_bytes));
  return 0;
}
