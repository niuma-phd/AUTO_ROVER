#include "auto_rover_vcu_wheeltec_serial/bench.hpp"
#include "auto_rover_vcu_wheeltec_serial/feedback_capture.hpp"
#include "auto_rover_vcu_wheeltec_serial/physical_activation.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <new>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

constexpr const char* kBenchSchema =
    "auto_rover.wheeltec.bench_characterization.v1";
constexpr double kMinimumDurationSeconds = 0.5;
constexpr double kMaximumExactZeroDurationSeconds = 30.0;
constexpr double kMaximumHoldSeconds = 5.0;
constexpr double kMaximumRaisedWheelStartHoldSeconds = 60.0;
constexpr std::size_t kMaximumBufferedEvidenceBytes = 16U * 1024U * 1024U;
volatile std::sig_atomic_t g_stop_requested = 0;

struct CommandLineOptions {
  bool show_help{false};
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  bool actuation_opt_in{false};
  bool allow_raised_wheel_start_asymmetry{false};
  bool record_uncalibrated_motion_feedback{false};
  wheeltec::BenchMode mode{wheeltec::BenchMode::kDisabled};
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
  double exact_zero_duration_seconds{0.0};
  double target_speed_mps{0.0};
  double hold_seconds{0.0};
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
    if (fd_ < 0 || failed_) {
      return false;
    }
    std::string record = line;
    record.push_back('\n');
    return writeBlock(record.data(), record.size());
  }

  bool writeBlock(const char* data, std::size_t size) {
    if ((data == nullptr && size != 0U) || fd_ < 0 || failed_) {
      return false;
    }
    std::size_t offset = 0U;
    while (offset < size) {
      const ssize_t count =
          ::write(fd_, data + offset, size - offset);
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      failed_ = true;
      return false;
    }
    return true;
  }

  bool finish() {
    if (fd_ < 0) {
      return false;
    }
    bool succeeded = !failed_;
    if (succeeded) {
      int sync_status = -1;
      do {
        sync_status = ::fsync(fd_);
      } while (sync_status < 0 && errno == EINTR);
      succeeded = sync_status == 0;
    }
    const int close_status = ::close(fd_);
    fd_ = -1;
    return succeeded && close_status == 0;
  }

 private:
  int fd_{-1};
  bool failed_{false};
};

class BoundedEvidenceBuffer {
 public:
  bool prepare() {
    try {
      data_.reserve(kMaximumBufferedEvidenceBytes);
    } catch (const std::bad_alloc&) {
      failed_ = true;
      return false;
    }
    return true;
  }

  bool appendLine(const std::string& line) {
    if (failed_ || line.size() >= kMaximumBufferedEvidenceBytes ||
        data_.size() >
            kMaximumBufferedEvidenceBytes - line.size() - 1U) {
      failed_ = true;
      return false;
    }
    try {
      data_.append(line);
      data_.push_back('\n');
    } catch (const std::bad_alloc&) {
      failed_ = true;
      return false;
    }
    return true;
  }

  bool flushTo(ExclusiveNdjsonSink* sink) const {
    return sink != nullptr && sink->writeBlock(data_.data(), data_.size());
  }

  bool failed() const { return failed_; }

 private:
  std::string data_;
  bool failed_{false};
};

void signalHandler(int) { g_stop_requested = 1; }

void printUsage(const char* program) {
  std::fprintf(
      stderr,
      "Usage: %s --unverified-protocol-acknowledged "
      "--physical-device-opt-in --actuation-opt-in "
      "--operator-confirmation %s --mode exact-zero|straight-ramp "
      "--device /dev/... --expected-major N --expected-minor N "
      "--expected-owner-uid N --expected-group-gid N "
      "--expected-usb-vid HEX --expected-usb-pid HEX "
      "--expected-usb-serial SERIAL --output /absolute/new-file.ndjson "
      "MODE_OPTIONS\n\n"
      "exact-zero MODE_OPTIONS:\n"
      "  --passive-evidence-token %s<64-lowercase-hex-sha256>\n"
      "  --duration-s SECONDS                 bounded %.1f..%.0f s\n\n"
      "straight-ramp MODE_OPTIONS:\n"
      "  --passive-evidence-token %s<64-lowercase-hex-sha256>\n"
      "  --exact-zero-evidence-token %s<64-lowercase-hex-sha256>\n"
      "  --target-speed-mps SPEED             >0 and <=%.2f\n"
      "  --hold-s SECONDS                     >0 and <=%.0f s by default\n"
      "  --allow-raised-wheel-start-asymmetry explicit raised-wheel-only opt-in; permits <=%.0f s hold\n"
      "  --record-uncalibrated-motion-feedback requires raised-wheel opt-in; records tracking without acceptance\n\n"
      "    feedback emergency ceiling 1.000 m/s; command/wire cap remains 0.500 m/s\n\n"
      "ROS-free, raised-bench characterization only. No device, mode, gate, "
      "speed, evidence SHA, or operator confirmation is selected by "
      "default. The candidate protocol remains UNVERIFIED. The tool does "
      "preserve the binary composite control-allow/inhibit feedback byte, "
      "but does not treat it as actuator enable, a specific fault, an ACK, "
      "or a command echo and does not connect to the phase-1 vehicle loop. "
      "Evidence SHAs are operator-attested; this "
      "tool does not open or validate their source files. Ctrl-C, SIGTERM, "
      "SIGHUP, SIGQUIT, or SIGTSTP each revokes authorization and attempts the same bounded "
      "exact-zero host write sequence; SIGKILL and SIGSTOP cannot be handled. "
      "A full host write is not a VCU acknowledgement.\n",
      program, wheeltec::kBenchOperatorConfirmationToken,
      wheeltec::kPassiveEvidenceTokenPrefix,
      kMinimumDurationSeconds, kMaximumExactZeroDurationSeconds,
      wheeltec::kPassiveEvidenceTokenPrefix,
      wheeltec::kExactZeroEvidenceTokenPrefix,
      wheeltec::kBenchMaximumForwardSpeedMps, kMaximumHoldSeconds,
      kMaximumRaisedWheelStartHoldSeconds);
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

bool parsePositiveDouble(const char* text, double* value) {
  if (text == nullptr || value == nullptr || text[0] == '\0') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' ||
      !std::isfinite(parsed) || parsed <= 0.0) {
    return false;
  }
  *value = parsed;
  return true;
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
  if (*seen) {
    *error = "duplicate " + option;
    return false;
  }
  *seen = true;
  return true;
}

bool parseCommandLine(int argc, char** argv, CommandLineOptions* options,
                      std::string* error) {
  if (options == nullptr || error == nullptr) {
    return false;
  }
  bool has_mode = false;
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
  bool has_duration = false;
  bool has_target = false;
  bool has_hold = false;
  bool has_output = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      options->show_help = true;
      continue;
    }
    if (argument == "--unverified-protocol-acknowledged") {
      if (options->unverified_protocol_acknowledged) {
        *error = "duplicate " + argument;
        return false;
      }
      options->unverified_protocol_acknowledged = true;
      continue;
    }
    if (argument == "--physical-device-opt-in") {
      if (options->physical_device_opt_in) {
        *error = "duplicate " + argument;
        return false;
      }
      options->physical_device_opt_in = true;
      continue;
    }
    if (argument == "--actuation-opt-in") {
      if (options->actuation_opt_in) {
        *error = "duplicate " + argument;
        return false;
      }
      options->actuation_opt_in = true;
      continue;
    }
    if (argument == "--allow-raised-wheel-start-asymmetry") {
      if (options->allow_raised_wheel_start_asymmetry) {
        *error = "duplicate " + argument;
        return false;
      }
      options->allow_raised_wheel_start_asymmetry = true;
      continue;
    }
    if (argument == "--record-uncalibrated-motion-feedback") {
      if (options->record_uncalibrated_motion_feedback) {
        *error = "duplicate " + argument;
        return false;
      }
      options->record_uncalibrated_motion_feedback = true;
      continue;
    }
    const char* value = nullptr;
    if (!takeValue(argc, argv, &index, &value)) {
      *error = "missing value after " + argument;
      return false;
    }
    if (argument == "--mode" &&
        markOnce(&has_mode, argument, error)) {
      const std::string mode(value);
      if (mode == "exact-zero") {
        options->mode = wheeltec::BenchMode::kExactZero;
      } else if (mode == "straight-ramp") {
        options->mode = wheeltec::BenchMode::kStraightRamp;
      } else {
        *error = "invalid --mode";
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
    } else if (argument == "--duration-s" &&
               markOnce(&has_duration, argument, error)) {
      if (!parsePositiveDouble(value,
                               &options->exact_zero_duration_seconds)) {
        *error = "invalid --duration-s";
        return false;
      }
    } else if (argument == "--target-speed-mps" &&
               markOnce(&has_target, argument, error)) {
      if (!parsePositiveDouble(value, &options->target_speed_mps)) {
        *error = "invalid --target-speed-mps";
        return false;
      }
    } else if (argument == "--hold-s" &&
               markOnce(&has_hold, argument, error)) {
      if (!parsePositiveDouble(value, &options->hold_seconds)) {
        *error = "invalid --hold-s";
        return false;
      }
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
      !has_mode || !has_operator || !has_device || !has_major ||
      !has_minor || !has_owner || !has_group || !has_vid || !has_pid ||
      !has_serial || !has_output) {
    *error = "all three gates, operator confirmation, mode, explicit device identity, and output are required";
    return false;
  }
  if (options->operator_confirmation_token !=
      wheeltec::kBenchOperatorConfirmationToken) {
    *error = "operator confirmation token does not match";
    return false;
  }
  if (options->mode == wheeltec::BenchMode::kExactZero) {
    if (!has_duration || !has_passive || has_target || has_hold || has_zero ||
        options->allow_raised_wheel_start_asymmetry ||
        options->record_uncalibrated_motion_feedback ||
        options->exact_zero_duration_seconds < kMinimumDurationSeconds ||
        options->exact_zero_duration_seconds >
            kMaximumExactZeroDurationSeconds) {
      *error = "exact-zero requires an operator-attested passive capture SHA and bounded --duration-s; ramp-only options are forbidden";
      return false;
    }
  } else {
    const double maximum_hold_seconds =
        options->allow_raised_wheel_start_asymmetry
            ? kMaximumRaisedWheelStartHoldSeconds
            : kMaximumHoldSeconds;
    if (!has_passive || !has_zero || !has_target || !has_hold ||
        has_duration ||
        options->target_speed_mps >
            wheeltec::kBenchMaximumForwardSpeedMps ||
        options->hold_seconds > maximum_hold_seconds ||
        (options->record_uncalibrated_motion_feedback &&
         !options->allow_raised_wheel_start_asymmetry)) {
      *error = "straight-ramp requires both evidence tokens, target speed <=0.50, and a hold <=5 s by default or <=60 s with explicit raised-wheel asymmetry opt-in; --duration-s is forbidden";
      return false;
    }
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
    switch (byte) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (byte < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4)
                 << std::setfill('0') << static_cast<unsigned int>(byte)
                 << std::dec;
        } else {
          stream << static_cast<char>(byte);
        }
        break;
    }
  }
  stream << '"';
  return stream.str();
}

std::string cliMetadataRecord(const CommandLineOptions& options) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"cli_preflight\","
         << "\"protocol_status\":\"UNVERIFIED\","
         << "\"mode\":" << jsonString(wheeltec::benchModeName(options.mode))
         << ",\"three_gates_confirmed\":true,"
         << "\"allow_raised_wheel_start_asymmetry\":"
         << (options.allow_raised_wheel_start_asymmetry ? "true" : "false")
         << ','
         << "\"record_uncalibrated_motion_feedback\":"
         << (options.record_uncalibrated_motion_feedback ? "true" : "false")
         << ','
         << "\"operator_confirmation_token_matched\":true,"
         << "\"device_path\":" << jsonString(options.device_path)
         << ",\"expected_identity\":{\"device_major\":"
         << options.expected_major << ",\"device_minor\":"
         << options.expected_minor << ",\"owner_uid\":"
         << options.expected_owner_uid << ",\"group_gid\":"
         << options.expected_group_gid << ",\"usb_vid\":"
         << jsonString(options.expected_usb_vid) << ",\"usb_pid\":"
         << jsonString(options.expected_usb_pid) << ",\"usb_serial\":"
         << jsonString(options.expected_usb_serial) << "},"
         << "\"passive_evidence_token\":"
         << jsonString(options.passive_evidence_token) << ','
         << "\"exact_zero_evidence_token\":"
         << jsonString(options.exact_zero_evidence_token) << '}';
  return stream.str();
}

std::string openResultRecord(const wheeltec::PhysicalOpenResult& result) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"open_result\",\"status\":\""
         << wheeltec::transportStatusName(result.status)
         << "\",\"os_error\":" << result.os_error << '}';
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

wheeltec::BenchConfig makeBenchConfig(const CommandLineOptions& options) {
  wheeltec::BenchConfig config;
  config.mode = options.mode;
  config.unverified_protocol_acknowledged =
      options.unverified_protocol_acknowledged;
  config.physical_device_opt_in = options.physical_device_opt_in;
  config.actuation_opt_in = options.actuation_opt_in;
  config.allow_raised_wheel_start_asymmetry =
      options.allow_raised_wheel_start_asymmetry;
  config.record_uncalibrated_motion_feedback =
      options.record_uncalibrated_motion_feedback;
  config.operator_confirmation_token = options.operator_confirmation_token;
  config.passive_evidence_token = options.passive_evidence_token;
  config.exact_zero_evidence_token = options.exact_zero_evidence_token;
  config.target_speed_mps = options.target_speed_mps;
  config.exact_zero_duration_ns = static_cast<std::int64_t>(
      options.exact_zero_duration_seconds * 1.0e9);
  config.hold_duration_ns =
      static_cast<std::int64_t>(options.hold_seconds * 1.0e9);
  return config;
}

}  // namespace

int main(int argc, char** argv) {
  CommandLineOptions command_line;
  std::string parse_error;
  if (!parseCommandLine(argc, argv, &command_line, &parse_error)) {
    std::fprintf(stderr, "wheeltec_bench_characterize: %s\n",
                 parse_error.c_str());
    printUsage(argv[0]);
    return 2;
  }
  if (command_line.show_help) {
    printUsage(argv[0]);
    return 0;
  }
  const wheeltec::BenchConfig bench_config = makeBenchConfig(command_line);
  const wheeltec::BenchStatus preflight_status =
      wheeltec::benchPreflightStatus(bench_config);
  if (preflight_status != wheeltec::BenchStatus::kCompleted) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: preflight refused before physical open (%s)\n",
                 wheeltec::benchStatusName(preflight_status));
    return 2;
  }

  wheeltec::PhysicalDeviceOptions physical_options;
  physical_options.device_path = command_line.device_path;
  physical_options.unverified_protocol_acknowledged =
      command_line.unverified_protocol_acknowledged;
  physical_options.physical_device_opt_in =
      command_line.physical_device_opt_in;
  physical_options.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  physical_options.actuation_opt_in = command_line.actuation_opt_in;
  physical_options.expected_device_major = command_line.expected_major;
  physical_options.expected_device_minor = command_line.expected_minor;
  physical_options.expected_owner_uid = command_line.expected_owner_uid;
  physical_options.expected_group_gid = command_line.expected_group_gid;
  physical_options.expected_usb_vendor_id = command_line.expected_usb_vid;
  physical_options.expected_usb_product_id = command_line.expected_usb_pid;
  physical_options.expected_usb_serial = command_line.expected_usb_serial;
  if (!wheeltec::physicalDeviceOptionsAreComplete(physical_options)) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: physical actuation identity or three-gate options are incomplete\n");
    return 2;
  }
  if (!wheeltec::kPhysicalActuationReleaseEnabled) {
    std::fprintf(
        stderr,
        "wheeltec_bench_characterize: physical actuation is release-frozen before device preparation/open\n");
    return 2;
  }
  if (!installSignalHandlers()) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot install bounded-shutdown signal handlers\n");
    return 3;
  }
  BoundedEvidenceBuffer session_evidence;
  if (!session_evidence.prepare()) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot reserve bounded in-memory session evidence before physical open\n");
    return 3;
  }

  ExclusiveNdjsonSink sink;
  if (!sink.openNew(command_line.output_path)) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot create exclusive output: %s\n",
                 std::strerror(errno));
    return 3;
  }
  if (!sink.writeLine(cliMetadataRecord(command_line))) {
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot persist CLI preflight evidence\n");
    return 3;
  }

  // Allocate every session callback/buffer and pre-encode the guard frame
  // before acquiring the physical command channel.
  wheeltec::BenchOperations operations;
  operations.monotonic_now_ns = []() { return wheeltec::monotonicNowNs(); };
  operations.wait_until_monotonic_ns = waitUntil;
  operations.record_json_line = [&session_evidence](const std::string& line) {
    return session_evidence.appendLine(line);
  };
  operations.stop_requested = []() { return g_stop_requested != 0; };
  wheeltec::WheeltecBenchSession session;
  wheeltec::PhysicalActivationConfig activation_config;
  activation_config.codec_limits.max_forward_speed_mps =
      wheeltec::kBenchMaximumForwardSpeedMps;
  activation_config.codec_limits.max_abs_curvature_inv_m =
      wheeltec::kPhase1MaximumAbsCurvatureInvM;
  wheeltec::PhysicalActivationOperations activation_operations;
  activation_operations.monotonic_now_ns = operations.monotonic_now_ns;
  activation_operations.wait_until_monotonic_ns =
      operations.wait_until_monotonic_ns;
  wheeltec::PreparedPhysicalActivation activation(
      activation_config, activation_operations);
  if (!activation.prepared()) {
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot prepare physical exact-zero activation guard\n");
    return 3;
  }
  wheeltec::PreparedPhysicalSerialOpen prepared_open(physical_options);

  wheeltec::PhysicalOpenResult opened =
      prepared_open.open();
  if (opened.status != wheeltec::TransportStatus::kOk ||
      !opened.transport) {
    if (!sink.writeLine(openResultRecord(opened))) {
      sink.finish();
      std::fprintf(stderr,
                   "wheeltec_bench_characterize: cannot persist failed serial open result\n");
      return 3;
    }
    const bool output_ok = sink.finish();
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: physical actuation open refused (%s, errno=%d)%s\n",
                 wheeltec::transportStatusName(opened.status), opened.os_error,
                 output_ok ? "" : "; output persistence also failed");
    return output_ok ? 4 : 3;
  }

  // The successful-open boundary contains no evidence callback or read: its
  // first protocol I/O is the prepared zero-only parser resynchronization,
  // followed by the exact-zero command candidate.
  const wheeltec::PhysicalActivationResult activation_result =
      activation.activate(opened.transport.get());
  if (!activation_result.succeeded()) {
    opened.transport.reset();
    const bool record_ok = sink.writeLine(openResultRecord(opened));
    const bool output_ok = sink.finish();
    std::fprintf(
        stderr,
                 "wheeltec_bench_characterize: physical zero-only activation failed (%s, poisoned=%s, unconfirmed=%s)%s\n",
        wheeltec::physicalActivationStatusName(activation_result.status),
        activation_result.write_stream_poisoned ? "true" : "false",
        activation_result.delivery_unconfirmed ? "true" : "false",
        record_ok && output_ok ? "" : "; output persistence also failed");
    return record_ok && output_ok ? 5 : 3;
  }
  if (!sink.writeLine(openResultRecord(opened))) {
    opened.transport.reset();
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: cannot persist serial open result after zero-only activation\n");
    return 3;
  }

  const wheeltec::BenchResult result =
      session.run(opened.transport.get(), bench_config, operations);
  opened.transport.reset();
  bool evidence_flushed = session_evidence.flushTo(&sink);
  if (session_evidence.failed()) {
    evidence_flushed =
        sink.writeLine(wheeltec::benchSummaryRecordJson(result)) &&
        evidence_flushed;
  }
  if (!evidence_flushed) {
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: post-zero session evidence flush failed\n");
    return 3;
  }
  if (!sink.finish()) {
    std::fprintf(stderr,
                 "wheeltec_bench_characterize: output persistence failed\n");
    return 3;
  }
  std::fprintf(
      stderr,
      "wheeltec_bench_characterize: status=%s rx_frames=%llu host_writes=%llu nonzero_host_writes=%llu final_zero=%s vcu_ack_available=false delivery_unconfirmed=%s\n",
      wheeltec::benchStatusName(result.status),
      static_cast<unsigned long long>(
          result.statistics.valid_feedback_frames),
      static_cast<unsigned long long>(
          result.statistics.tx_host_writes_completed),
      static_cast<unsigned long long>(
          result.statistics.nonzero_frame_host_writes_completed),
      result.zero_host_write_completed ? "host_write_complete" : "FAILED",
      result.delivery_unconfirmed ? "true" : "false");
  if (result.status == wheeltec::BenchStatus::kInterrupted) {
    return 130;
  }
  return result.completed() ? 0 : 5;
}
