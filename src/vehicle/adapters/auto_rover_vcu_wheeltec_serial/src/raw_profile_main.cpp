#include "auto_rover_vcu_wheeltec_serial/raw_profile.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

constexpr const char* kSchema =
    "auto_rover.wheeltec.raw_profile_capture.v1";
volatile std::sig_atomic_t g_stop_requested = 0;

struct CommandLineOptions {
  bool show_help{false};
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  bool actuation_opt_in{false};
  bool raw_raised_bench_opt_in{false};
  wheeltec::RawProfile profile{wheeltec::RawProfile::kDisabled};
  std::string operator_confirmation_token;
  std::string passive_evidence_token;
  std::string exact_zero_evidence_token;
  std::string device_path;
  std::uint64_t expected_major{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_minor{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_owner_uid{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_group_gid{wheeltec::kUnspecifiedPhysicalIdentity};
  std::string expected_usb_vid;
  std::string expected_usb_pid;
  std::string expected_usb_serial;
  std::string output_path;
};

class ExclusiveNdjsonSink {
 public:
  ~ExclusiveNdjsonSink() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool openNew(const std::string& path) {
    if (fd_ >= 0) {
      errno = EALREADY;
      return false;
    }
    do {
      fd_ = ::open(path.c_str(),
                   O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                   0600);
    } while (fd_ < 0 && errno == EINTR);
    return fd_ >= 0;
  }

  bool writeLine(const std::string& line) {
    std::string record = line;
    record.push_back('\n');
    return writeBlock(record.data(), record.size());
  }

  bool writeBlock(const char* data, std::size_t size) {
    if (fd_ < 0 || failed_ || (data == nullptr && size != 0U)) {
      return false;
    }
    std::size_t offset = 0U;
    while (offset < size) {
      const ssize_t written = ::write(fd_, data + offset, size - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
      } else if (written < 0 && errno == EINTR) {
        continue;
      } else {
        failed_ = true;
        return false;
      }
    }
    return true;
  }

  bool finish() {
    if (fd_ < 0) {
      return false;
    }
    bool succeeded = !failed_;
    int status = -1;
    do {
      status = ::fsync(fd_);
    } while (status < 0 && errno == EINTR);
    succeeded = succeeded && status == 0;
    const int close_status = ::close(fd_);
    fd_ = -1;
    return succeeded && close_status == 0;
  }

 private:
  int fd_{-1};
  bool failed_{false};
};

void signalHandler(int) { g_stop_requested = 1; }

void printUsage(const char* program) {
  std::fprintf(
      stderr,
      "Usage: %s --unverified-protocol-acknowledged "
      "--physical-device-opt-in --actuation-opt-in "
      "--raw-raised-bench-opt-in --operator-confirmation %s "
      "--profile PROFILE --passive-evidence-token "
      "passive-capture-sha256:<64-lowercase-hex> "
      "--exact-zero-evidence-token "
      "exact-zero-sha256:<64-lowercase-hex> "
      "--device /dev/... --expected-major N --expected-minor N "
      "--expected-owner-uid N --expected-group-gid N "
      "--expected-usb-vid HEX --expected-usb-pid HEX "
      "--expected-usb-serial SERIAL --output /absolute/new-file.ndjson\n\n"
      "Exactly one fixed PROFILE is required per invocation; every target "
      "hold is 60 s. The complete closed catalog is printed below.\n\n"
      "UNVERIFIED ROS-free experimental raised-bench data capture. It is "
      "not installed and is not connected to Phase-1. Commands use a fixed "
      "0.20 m/s^2 center-command ramp and remain <=6.0 m/s. Feedback "
      "magnitude, tracking, "
      "left/right difference, and post-zero tail are recorded, not online "
      "acceptance gates. Raw chunks and parser resynchronization errors are "
      "also recorded without refreshing feedback freshness. Valid-frame "
      "finite semantics, FlagStop, feedback freshness, "
      "100 ms command watchdog, strict wire slew, clock, transport, partial "
      "write, evidence, and caught-signal failures stop and attempt a bounded "
      "exact-zero write. SIGKILL/SIGSTOP cannot be handled. A successful host "
      "write is not a VCU acknowledgement. After physical open, session "
      "evidence uses a pre-reserved 32 MiB memory buffer; it is flushed only "
      "after bounded zero and serial close, so a crash may lose the entire "
      "motion record and a missing summary is invalid evidence. No gate, "
      "device, profile, evidence, "
      "or output is selected by default.\n",
      program, wheeltec::kRawProfileOperatorConfirmationToken);
  std::fprintf(stderr, "\nFixed PROFILE catalog (%zu):\n",
               wheeltec::kRawProfileFixedProfileCount);
  for (std::size_t index = 0U;
       index < wheeltec::kRawProfileFixedProfileCount; ++index) {
    wheeltec::RawProfile profile = wheeltec::RawProfile::kDisabled;
    if (!wheeltec::rawProfileAt(index, &profile)) {
      std::fprintf(stderr, "  <internal catalog error at %zu>\n", index);
      continue;
    }
    std::fprintf(stderr, "  %s\n", wheeltec::rawProfileName(profile));
  }
}

bool takeValue(int argc, char** argv, int* index, const char** value) {
  if (index == nullptr || value == nullptr || *index + 1 >= argc) {
    return false;
  }
  ++(*index);
  *value = argv[*index];
  return true;
}

bool markOnce(bool* seen, const std::string& option, std::string* error) {
  if (seen == nullptr || error == nullptr) {
    return false;
  }
  if (*seen) {
    *error = "duplicate " + option;
    return false;
  }
  *seen = true;
  return true;
}

bool parseUnsigned(const char* text, std::uint64_t* value) {
  if (text == nullptr || value == nullptr || text[0] == '\0' ||
      text[0] == '-') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *value = static_cast<std::uint64_t>(parsed);
  return true;
}

bool parseProfile(const std::string& text, wheeltec::RawProfile* profile) {
  return wheeltec::rawProfileFromName(text, profile);
}

bool parseCommandLine(int argc, char** argv, CommandLineOptions* options,
                      std::string* error) {
  if (options == nullptr || error == nullptr) {
    return false;
  }
  bool has_profile = false;
  bool has_operator = false;
  bool has_passive = false;
  bool has_zero = false;
  bool has_device = false;
  bool has_major = false;
  bool has_minor = false;
  bool has_owner = false;
  bool has_group = false;
  bool has_vid = false;
  bool has_pid = false;
  bool has_serial = false;
  bool has_output = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      options->show_help = true;
      continue;
    }
    bool* gate = nullptr;
    if (argument == "--unverified-protocol-acknowledged") {
      gate = &options->unverified_protocol_acknowledged;
    } else if (argument == "--physical-device-opt-in") {
      gate = &options->physical_device_opt_in;
    } else if (argument == "--actuation-opt-in") {
      gate = &options->actuation_opt_in;
    } else if (argument == "--raw-raised-bench-opt-in") {
      gate = &options->raw_raised_bench_opt_in;
    }
    if (gate != nullptr) {
      if (*gate) {
        *error = "duplicate " + argument;
        return false;
      }
      *gate = true;
      continue;
    }

    const char* value = nullptr;
    if (!takeValue(argc, argv, &index, &value)) {
      *error = "missing value after " + argument;
      return false;
    }
    if (argument == "--profile" &&
        markOnce(&has_profile, argument, error)) {
      if (!parseProfile(value, &options->profile)) {
        *error = "invalid --profile";
        return false;
      }
    } else if (argument == "--operator-confirmation" &&
               markOnce(&has_operator, argument, error)) {
      options->operator_confirmation_token = value;
    } else if (argument == "--passive-evidence-token" &&
               markOnce(&has_passive, argument, error)) {
      options->passive_evidence_token = value;
    } else if (argument == "--exact-zero-evidence-token" &&
               markOnce(&has_zero, argument, error)) {
      options->exact_zero_evidence_token = value;
    } else if (argument == "--device" &&
               markOnce(&has_device, argument, error)) {
      options->device_path = value;
    } else if (argument == "--expected-major" &&
               markOnce(&has_major, argument, error)) {
      if (!parseUnsigned(value, &options->expected_major)) {
        *error = "invalid --expected-major";
        return false;
      }
    } else if (argument == "--expected-minor" &&
               markOnce(&has_minor, argument, error)) {
      if (!parseUnsigned(value, &options->expected_minor)) {
        *error = "invalid --expected-minor";
        return false;
      }
    } else if (argument == "--expected-owner-uid" &&
               markOnce(&has_owner, argument, error)) {
      if (!parseUnsigned(value, &options->expected_owner_uid)) {
        *error = "invalid --expected-owner-uid";
        return false;
      }
    } else if (argument == "--expected-group-gid" &&
               markOnce(&has_group, argument, error)) {
      if (!parseUnsigned(value, &options->expected_group_gid)) {
        *error = "invalid --expected-group-gid";
        return false;
      }
    } else if (argument == "--expected-usb-vid" &&
               markOnce(&has_vid, argument, error)) {
      options->expected_usb_vid = value;
    } else if (argument == "--expected-usb-pid" &&
               markOnce(&has_pid, argument, error)) {
      options->expected_usb_pid = value;
    } else if (argument == "--expected-usb-serial" &&
               markOnce(&has_serial, argument, error)) {
      options->expected_usb_serial = value;
    } else if (argument == "--output" &&
               markOnce(&has_output, argument, error)) {
      options->output_path = value;
    } else {
      if (error->empty()) {
        *error = "unknown argument: " + argument;
      }
      return false;
    }
  }
  if (options->show_help) {
    return true;
  }
  if (!options->unverified_protocol_acknowledged ||
      !options->physical_device_opt_in || !options->actuation_opt_in ||
      !options->raw_raised_bench_opt_in || !has_profile || !has_operator ||
      !has_passive || !has_zero || !has_device || !has_major || !has_minor ||
      !has_owner || !has_group || !has_vid || !has_pid || !has_serial ||
      !has_output) {
    *error = "all four gates, one fixed profile, operator token, both evidence hashes, exact device identity, and output are required";
    return false;
  }
  if (options->operator_confirmation_token !=
      wheeltec::kRawProfileOperatorConfirmationToken) {
    *error = "operator confirmation token does not match";
    return false;
  }
  if (options->device_path.empty() || options->output_path.empty() ||
      options->output_path.front() != '/' ||
      options->output_path.find("..") != std::string::npos ||
      options->output_path.compare(0U, 5U, "/dev/") == 0 ||
      options->output_path.compare(0U, 6U, "/proc/") == 0 ||
      options->output_path.compare(0U, 5U, "/sys/") == 0) {
    *error = "output must be a safe absolute non-device path";
    return false;
  }
  return true;
}

std::string jsonString(const std::string& input) {
  std::ostringstream stream;
  stream << '"';
  for (const unsigned char byte : input) {
    if (byte == '"' || byte == '\\') {
      stream << '\\' << static_cast<char>(byte);
    } else if (byte >= 0x20U) {
      stream << static_cast<char>(byte);
    } else {
      stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
             << static_cast<unsigned int>(byte) << std::dec;
    }
  }
  stream << '"';
  return stream.str();
}

const char* transportStatusText(wheeltec::TransportStatus status) {
  switch (status) {
    case wheeltec::TransportStatus::kOk:
      return "ok";
    case wheeltec::TransportStatus::kWouldBlock:
      return "would_block";
    case wheeltec::TransportStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case wheeltec::TransportStatus::kDisconnected:
      return "disconnected";
    case wheeltec::TransportStatus::kDisabled:
      return "disabled";
    case wheeltec::TransportStatus::kInvalidArgument:
      return "invalid_argument";
    case wheeltec::TransportStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

std::string cliMetadataRecord(const CommandLineOptions& options) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"cli_preflight\""
         << ",\"protocol_status\":\"UNVERIFIED\""
         << ",\"profile_id\":"
         << jsonString(wheeltec::rawProfileName(options.profile))
         << ",\"four_gates_confirmed\":true"
         << ",\"raw_raised_bench_opt_in\":true"
         << ",\"operator_confirmation_token_matched\":true"
         << ",\"installed\":false"
         << ",\"active_session_filesystem_io\":false"
         << ",\"active_session_evidence_buffer_capacity_bytes\":"
         << wheeltec::kRawProfileEvidenceBufferCapacityBytes
         << ",\"session_evidence_flush_after_zero_and_serial_close\":true"
         << ",\"crash_may_lose_entire_buffered_session\":true"
         << ",\"device_path\":" << jsonString(options.device_path)
         << ",\"expected_identity\":{\"device_major\":"
         << options.expected_major << ",\"device_minor\":"
         << options.expected_minor << ",\"owner_uid\":"
         << options.expected_owner_uid << ",\"group_gid\":"
         << options.expected_group_gid << ",\"usb_vid\":"
         << jsonString(options.expected_usb_vid) << ",\"usb_pid\":"
         << jsonString(options.expected_usb_pid) << ",\"usb_serial\":"
         << jsonString(options.expected_usb_serial) << "}"
         << ",\"passive_evidence_token\":"
         << jsonString(options.passive_evidence_token)
         << ",\"exact_zero_evidence_token\":"
         << jsonString(options.exact_zero_evidence_token) << '}';
  return stream.str();
}

std::string openResultRecord(const wheeltec::PhysicalOpenResult& result,
                             wheeltec::RawProfile profile) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"open_result\",\"profile_id\":\""
         << wheeltec::rawProfileName(profile) << "\",\"status\":\""
         << transportStatusText(result.status) << "\",\"os_error\":"
         << result.os_error << '}';
  return stream.str();
}

std::string startupZeroRecord(
    const wheeltec::RawProfileImmediateZeroResult& result,
    wheeltec::RawProfile profile) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"startup_exact_zero_result\""
         << ",\"profile_id\":\"" << wheeltec::rawProfileName(profile)
         << "\",\"attempts\":" << result.attempts
         << ",\"exact_zero_frame\":true"
         << ",\"zero_host_write_completed\":"
         << (result.zero_host_write_completed ? "true" : "false")
         << ",\"vcu_acknowledgement\":false"
         << ",\"delivery_unconfirmed\":"
         << (result.delivery_unconfirmed ? "true" : "false")
         << ",\"terminal_transport_status\":\""
         << transportStatusText(result.terminal_transport_status)
         << "\",\"recording_order\":\"all_zero_attempts_completed_before_this_record\"}";
  return stream.str();
}

bool waitUntil(std::int64_t deadline_ns) {
  if (deadline_ns <= 0) {
    return false;
  }
  timespec deadline{};
  deadline.tv_sec = static_cast<time_t>(deadline_ns / 1000000000LL);
  deadline.tv_nsec = static_cast<long>(deadline_ns % 1000000000LL);
  int status = 0;
  do {
    status = ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline,
                               nullptr);
  } while (status == EINTR);
  return status == 0;
}

bool installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = signalHandler;
  ::sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  return ::sigaction(SIGINT, &action, nullptr) == 0 &&
         ::sigaction(SIGTERM, &action, nullptr) == 0 &&
         ::sigaction(SIGHUP, &action, nullptr) == 0 &&
         ::sigaction(SIGQUIT, &action, nullptr) == 0 &&
         ::sigaction(SIGTSTP, &action, nullptr) == 0;
}

wheeltec::RawProfileConfig makeConfig(const CommandLineOptions& options) {
  wheeltec::RawProfileConfig config;
  config.profile = options.profile;
  config.unverified_protocol_acknowledged =
      options.unverified_protocol_acknowledged;
  config.physical_device_opt_in = options.physical_device_opt_in;
  config.actuation_opt_in = options.actuation_opt_in;
  config.raw_raised_bench_opt_in = options.raw_raised_bench_opt_in;
  config.operator_confirmation_token = options.operator_confirmation_token;
  config.passive_evidence_token = options.passive_evidence_token;
  config.exact_zero_evidence_token = options.exact_zero_evidence_token;
  return config;
}

}  // namespace

int main(int argc, char** argv) {
  CommandLineOptions command_line;
  std::string parse_error;
  if (!parseCommandLine(argc, argv, &command_line, &parse_error)) {
    std::fprintf(stderr, "wheeltec_raw_profile_capture: %s\n",
                 parse_error.c_str());
    printUsage(argv[0]);
    return 2;
  }
  if (command_line.show_help) {
    printUsage(argv[0]);
    return 0;
  }
  const wheeltec::RawProfileConfig config = makeConfig(command_line);
  const wheeltec::RawProfileStatus preflight =
      wheeltec::rawProfilePreflightStatus(config);
  if (preflight != wheeltec::RawProfileStatus::kCompleted) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: preflight refused before physical open (%s)\n",
                 wheeltec::rawProfileStatusName(preflight));
    return 2;
  }

  wheeltec::PhysicalDeviceOptions physical;
  physical.device_path = command_line.device_path;
  physical.unverified_protocol_acknowledged =
      command_line.unverified_protocol_acknowledged;
  physical.physical_device_opt_in = command_line.physical_device_opt_in;
  physical.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  physical.actuation_opt_in = command_line.actuation_opt_in;
  physical.expected_device_major = command_line.expected_major;
  physical.expected_device_minor = command_line.expected_minor;
  physical.expected_owner_uid = command_line.expected_owner_uid;
  physical.expected_group_gid = command_line.expected_group_gid;
  physical.expected_usb_vendor_id = command_line.expected_usb_vid;
  physical.expected_usb_product_id = command_line.expected_usb_pid;
  physical.expected_usb_serial = command_line.expected_usb_serial;
  if (!wheeltec::physicalDeviceOptionsAreComplete(physical)) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: physical identity or actuation gates are incomplete\n");
    return 2;
  }
  if (!installSignalHandlers()) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: cannot install bounded-stop signal handlers\n");
    return 3;
  }
  wheeltec::RawProfileEvidenceBuffer evidence;
  if (!evidence.prepare()) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: cannot reserve the 32 MiB bounded evidence buffer before physical open\n");
    return 3;
  }
  ExclusiveNdjsonSink sink;
  if (!sink.openNew(command_line.output_path)) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: cannot create exclusive output: %s\n",
                 std::strerror(errno));
    return 3;
  }
  if (!sink.writeLine(cliMetadataRecord(command_line))) {
    sink.finish();
    return 3;
  }

  wheeltec::RawProfileOperations operations;
  operations.monotonic_now_ns = []() { return wheeltec::monotonicNowNs(); };
  operations.wait_until_monotonic_ns = waitUntil;
  operations.record_json_line = [&evidence](const std::string& line) {
    return evidence.appendLine(line);
  };
  operations.stop_requested = []() { return g_stop_requested != 0; };
  const wheeltec::RawProfileEncodeResult preencoded_zero =
      wheeltec::encodeRawProfileCommand(command_line.profile, 0);
  if (!preencoded_zero.ok()) {
    sink.finish();
    return 3;
  }

  wheeltec::PhysicalOpenResult opened = wheeltec::openPhysicalSerial(physical);
  wheeltec::RawProfileImmediateZeroResult startup_zero;
  if (opened.status == wheeltec::TransportStatus::kOk &&
      opened.transport) {
    startup_zero = wheeltec::writeRawProfileImmediateZeroNoRecord(
        opened.transport.get(), preencoded_zero.frame, operations);
  }
  const std::string open_result_record =
      openResultRecord(opened, command_line.profile);
  if (!evidence.appendLine(open_result_record)) {
    opened.transport.reset();
    sink.finish();
    return 3;
  }
  if (opened.status != wheeltec::TransportStatus::kOk ||
      !opened.transport) {
    const std::string& buffered = evidence.contents();
    const bool evidence_ok =
        sink.writeBlock(buffered.data(), buffered.size());
    const bool output_ok = sink.finish();
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: physical open refused (%s, errno=%d)%s\n",
                 transportStatusText(opened.status), opened.os_error,
                 output_ok ? "" : "; output persistence also failed");
    return evidence_ok && output_ok ? 4 : 3;
  }
  if (!evidence.appendLine(
          startupZeroRecord(startup_zero, command_line.profile))) {
    opened.transport.reset();
    sink.finish();
    return 3;
  }
  if (!startup_zero.zero_host_write_completed) {
    opened.transport.reset();
    wheeltec::RawProfileResult failed;
    failed.status = startup_zero.terminal_transport_status ==
                            wheeltec::TransportStatus::kDisconnected
                        ? wheeltec::RawProfileStatus::kDisconnected
                        : wheeltec::RawProfileStatus::kWriteFailed;
    failed.delivery_unconfirmed = true;
    const std::string& buffered = evidence.contents();
    const bool evidence_ok =
        sink.writeBlock(buffered.data(), buffered.size());
    const bool summary_ok =
        sink.writeLine(wheeltec::rawProfileSummaryRecordJson(failed));
    const bool output_ok = sink.finish();
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: startup exact-zero host write failed; no read or profile command was attempted\n");
    return evidence_ok && summary_ok && output_ok ? 5 : 3;
  }
  wheeltec::WheeltecRawProfileSession session;
  const wheeltec::RawProfileResult result =
      session.run(opened.transport.get(), config, operations);
  opened.transport.reset();

  const std::string& buffered_evidence = evidence.contents();
  const bool evidence_ok = sink.writeBlock(
      buffered_evidence.data(), buffered_evidence.size());
  const bool summary_ok =
      sink.writeLine(wheeltec::rawProfileSummaryRecordJson(result));
  const bool finish_ok = sink.finish();
  if (!evidence_ok || !summary_ok || !finish_ok) {
    std::fprintf(stderr,
                 "wheeltec_raw_profile_capture: post-zero evidence persistence failed\n");
    return 3;
  }
  std::fprintf(
      stderr,
      "wheeltec_raw_profile_capture: profile=%s status=%s rx_frames=%llu normal_host_writes=%llu nonzero_host_writes=%llu final_zero=%s vcu_ack_available=false delivery_unconfirmed=%s\n",
      wheeltec::rawProfileName(config.profile),
      wheeltec::rawProfileStatusName(result.status),
      static_cast<unsigned long long>(
          result.statistics.valid_feedback_frames),
      static_cast<unsigned long long>(
          result.statistics.normal_tx_host_writes_completed),
      static_cast<unsigned long long>(
          result.statistics.nonzero_frame_host_writes_completed),
      result.zero_host_write_completed ? "host_write_complete" : "FAILED",
      result.delivery_unconfirmed ? "true" : "false");
  if (result.status == wheeltec::RawProfileStatus::kInterrupted) {
    return 130;
  }
  return result.completed() ? 0 : 5;
}
