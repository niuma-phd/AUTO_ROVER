#include "auto_rover_vcu_wheeltec_serial/feedback_capture.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

constexpr const char* kCaptureSchema =
    "auto_rover.wheeltec.feedback_capture.v1";
constexpr double kMinimumDurationSeconds = 0.1;
constexpr double kMaximumDurationSeconds = 3600.0;

struct CommandLineOptions {
  bool show_help{false};
  bool feedback_only_opt_in{false};
  bool unverified_protocol_acknowledged{false};
  std::string device_path;
  std::uint64_t expected_major{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_minor{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_owner_uid{wheeltec::kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_group_gid{wheeltec::kUnspecifiedPhysicalIdentity};
  std::string expected_usb_vid;
  std::string expected_usb_pid;
  std::string expected_usb_serial;
  double duration_seconds{0.0};
  std::int64_t read_timeout_ns{100000000};
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
    std::size_t offset = 0U;
    while (offset < record.size()) {
      const ssize_t count =
          ::write(fd_, record.data() + offset, record.size() - offset);
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

void printUsage(const char* program) {
  std::fprintf(
      stderr,
      "Usage: %s --feedback-only-opt-in "
      "--unverified-protocol-acknowledged --device /dev/... "
      "--expected-major N --expected-minor N --expected-owner-uid N "
      "--expected-group-gid N --expected-usb-vid HEX "
      "--expected-usb-pid HEX --expected-usb-serial SERIAL "
      "--duration-s SECONDS --output /absolute/new-file.ndjson "
      "[--read-timeout-ms MILLISECONDS]\n"
      "\n"
      "Receive-only UNVERIFIED Wheeltec candidate feedback capture. The "
      "tool opens the explicitly named character device O_RDONLY, never "
      "constructs the actuation adapter, and never writes to the serial "
      "transport. Feedback byte 1 is preserved as a binary composite "
      "control-allow/inhibit value, not an ACK, specific fault, or command "
      "echo. Output must be a new file. Duration is bounded to %.1f..%.0f "
      "seconds.\n",
      program, kMinimumDurationSeconds, kMaximumDurationSeconds);
}

bool parseUnsigned(const char* text, std::uint64_t* value) {
  if (text == nullptr || value == nullptr || text[0] == '\0' || text[0] == '-') {
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
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed) ||
      parsed <= 0.0) {
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

bool parseCommandLine(int argc, char** argv, CommandLineOptions* options,
                      std::string* error) {
  if (options == nullptr || error == nullptr) {
    return false;
  }
  bool has_device = false;
  bool has_major = false;
  bool has_minor = false;
  bool has_owner = false;
  bool has_group = false;
  bool has_vid = false;
  bool has_pid = false;
  bool has_serial = false;
  bool has_duration = false;
  bool has_output = false;
  bool has_read_timeout = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      options->show_help = true;
      continue;
    }
    if (argument == "--feedback-only-opt-in") {
      if (options->feedback_only_opt_in) {
        *error = "duplicate --feedback-only-opt-in";
        return false;
      }
      options->feedback_only_opt_in = true;
      continue;
    }
    if (argument == "--unverified-protocol-acknowledged") {
      if (options->unverified_protocol_acknowledged) {
        *error = "duplicate --unverified-protocol-acknowledged";
        return false;
      }
      options->unverified_protocol_acknowledged = true;
      continue;
    }
    const char* value = nullptr;
    if (!takeValue(argc, argv, &index, &value)) {
      *error = "missing value after " + argument;
      return false;
    }
    if (argument == "--device" && !has_device) {
      options->device_path = value;
      has_device = true;
    } else if (argument == "--expected-major" && !has_major) {
      has_major = parseUnsigned(value, &options->expected_major);
      if (!has_major) {
        *error = "invalid --expected-major";
        return false;
      }
    } else if (argument == "--expected-minor" && !has_minor) {
      has_minor = parseUnsigned(value, &options->expected_minor);
      if (!has_minor) {
        *error = "invalid --expected-minor";
        return false;
      }
    } else if (argument == "--expected-owner-uid" && !has_owner) {
      has_owner = parseUnsigned(value, &options->expected_owner_uid);
      if (!has_owner) {
        *error = "invalid --expected-owner-uid";
        return false;
      }
    } else if (argument == "--expected-group-gid" && !has_group) {
      has_group = parseUnsigned(value, &options->expected_group_gid);
      if (!has_group) {
        *error = "invalid --expected-group-gid";
        return false;
      }
    } else if (argument == "--expected-usb-vid" && !has_vid) {
      options->expected_usb_vid = value;
      has_vid = true;
    } else if (argument == "--expected-usb-pid" && !has_pid) {
      options->expected_usb_pid = value;
      has_pid = true;
    } else if (argument == "--expected-usb-serial" && !has_serial) {
      options->expected_usb_serial = value;
      has_serial = true;
    } else if (argument == "--duration-s" && !has_duration) {
      has_duration = parsePositiveDouble(value, &options->duration_seconds);
      if (!has_duration) {
        *error = "invalid --duration-s";
        return false;
      }
    } else if (argument == "--read-timeout-ms" && !has_read_timeout) {
      double timeout_ms = 0.0;
      has_read_timeout = parsePositiveDouble(value, &timeout_ms);
      if (!has_read_timeout || timeout_ms > 1000.0) {
        *error = "invalid --read-timeout-ms (must be >0 and <=1000)";
        return false;
      }
      options->read_timeout_ns =
          static_cast<std::int64_t>(timeout_ms * 1.0e6);
      if (options->read_timeout_ns <= 0) {
        *error = "--read-timeout-ms is below clock resolution";
        return false;
      }
    } else if (argument == "--output" && !has_output) {
      options->output_path = value;
      has_output = true;
    } else {
      *error = "unknown or duplicate argument: " + argument;
      return false;
    }
  }
  if (options->show_help) {
    return true;
  }
  if (!options->feedback_only_opt_in ||
      !options->unverified_protocol_acknowledged || !has_device ||
      !has_major || !has_minor || !has_owner || !has_group || !has_vid ||
      !has_pid || !has_serial || !has_duration || !has_output) {
    *error = "all feedback-only gates, identity fields, duration, and output are required";
    return false;
  }
  if (options->duration_seconds < kMinimumDurationSeconds ||
      options->duration_seconds > kMaximumDurationSeconds) {
    *error = "--duration-s is outside the bounded capture range";
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
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(byte) << std::dec;
        } else {
          stream << static_cast<char>(byte);
        }
        break;
    }
  }
  stream << '"';
  return stream.str();
}

std::string metadataRecord(const CommandLineOptions& options) {
  std::ostringstream stream;
  stream << "{\"schema\":" << jsonString(kCaptureSchema)
         << ",\"record_type\":\"metadata\","
         << "\"protocol_status\":\"UNVERIFIED\","
         << "\"feedback_only\":true,\"actuation_enabled\":false,"
         << "\"feedback_byte1_semantics\":\"binary_composite_current_cycle_control_allow_or_inhibit\","
         << "\"feedback_byte1_is_vcu_ack\":false,"
         << "\"feedback_byte1_is_specific_fault\":false,"
         << "\"feedback_byte1_is_command_echo\":false,"
         << "\"serial_open_mode\":\"O_RDONLY\",\"device_path\":"
         << jsonString(options.device_path)
         << ",\"expected_identity\":{\"device_major\":"
         << options.expected_major << ",\"device_minor\":"
         << options.expected_minor << ",\"owner_uid\":"
         << options.expected_owner_uid << ",\"group_gid\":"
         << options.expected_group_gid << ",\"usb_vid\":"
         << jsonString(options.expected_usb_vid) << ",\"usb_pid\":"
         << jsonString(options.expected_usb_pid) << ",\"usb_serial\":"
         << jsonString(options.expected_usb_serial)
         << "},\"requested_duration_s\":" << std::setprecision(17)
         << options.duration_seconds << ",\"read_timeout_ns\":"
         << options.read_timeout_ns << '}';
  return stream.str();
}

std::string openResultRecord(const wheeltec::PhysicalOpenResult& result) {
  std::ostringstream stream;
  stream << "{\"schema\":" << jsonString(kCaptureSchema)
         << ",\"record_type\":\"open_result\",\"status\":"
         << jsonString(wheeltec::transportStatusName(result.status))
         << ",\"os_error\":" << result.os_error << '}';
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  CommandLineOptions command_line;
  std::string parse_error;
  if (!parseCommandLine(argc, argv, &command_line, &parse_error)) {
    std::fprintf(stderr, "wheeltec_feedback_capture: %s\n",
                 parse_error.c_str());
    printUsage(argv[0]);
    return 2;
  }
  if (command_line.show_help) {
    printUsage(argv[0]);
    return 0;
  }

  wheeltec::PhysicalDeviceOptions physical_options;
  physical_options.device_path = command_line.device_path;
  physical_options.unverified_protocol_acknowledged =
      command_line.unverified_protocol_acknowledged;
  physical_options.physical_device_opt_in =
      command_line.feedback_only_opt_in;
  physical_options.access_mode = wheeltec::PhysicalAccessMode::kFeedbackOnly;
  physical_options.actuation_opt_in = false;
  physical_options.expected_device_major = command_line.expected_major;
  physical_options.expected_device_minor = command_line.expected_minor;
  physical_options.expected_owner_uid = command_line.expected_owner_uid;
  physical_options.expected_group_gid = command_line.expected_group_gid;
  physical_options.expected_usb_vendor_id = command_line.expected_usb_vid;
  physical_options.expected_usb_product_id = command_line.expected_usb_pid;
  physical_options.expected_usb_serial = command_line.expected_usb_serial;
  if (!wheeltec::physicalDeviceOptionsAreComplete(physical_options)) {
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: physical identity or feedback-only gates are invalid\n");
    return 2;
  }

  ExclusiveNdjsonSink sink;
  if (!sink.openNew(command_line.output_path)) {
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: cannot create exclusive output: %s\n",
                 std::strerror(errno));
    return 3;
  }
  if (!sink.writeLine(metadataRecord(command_line))) {
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: cannot write capture metadata\n");
    return 3;
  }

  wheeltec::PhysicalOpenResult opened =
      wheeltec::openPhysicalSerial(physical_options);
  if (opened.status != wheeltec::TransportStatus::kOk ||
      !opened.transport) {
    const bool output_ok = sink.writeLine(openResultRecord(opened)) &&
                           sink.finish();
    if (!output_ok) {
      std::fprintf(stderr,
                   "wheeltec_feedback_capture: failed to persist serial open failure\n");
      return 3;
    }
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: receive-only serial open failed (%s, errno=%d)\n",
                 wheeltec::transportStatusName(opened.status), opened.os_error);
    return 4;
  }
  if (!sink.writeLine(openResultRecord(opened))) {
    sink.finish();
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: cannot record serial open result\n");
    return 3;
  }

  wheeltec::FeedbackCaptureConfig capture_config;
  capture_config.duration_ns = static_cast<std::int64_t>(
      command_line.duration_seconds * 1.0e9);
  capture_config.read_timeout_ns = command_line.read_timeout_ns;
  wheeltec::FeedbackCaptureOperations operations;
  operations.monotonic_now_ns = []() { return wheeltec::monotonicNowNs(); };
  operations.record_json_line = [&sink](const std::string& line) {
    return sink.writeLine(line);
  };
  wheeltec::FeedbackCaptureSession session;
  const wheeltec::FeedbackCaptureResult capture_result =
      session.run(opened.transport.get(), capture_config, operations);
  if (!sink.finish()) {
    std::fprintf(stderr,
                 "wheeltec_feedback_capture: output persistence failed\n");
    return 3;
  }
  if (!capture_result.completed()) {
    std::fprintf(stderr, "wheeltec_feedback_capture: capture ended with %s\n",
                 wheeltec::feedbackCaptureStatusName(capture_result.status));
    return 5;
  }
  std::fprintf(stderr,
               "wheeltec_feedback_capture: completed receive-only capture; frames=%llu bytes=%llu timeouts=%llu\n",
               static_cast<unsigned long long>(
                   capture_result.statistics.valid_frames),
               static_cast<unsigned long long>(
                   capture_result.statistics.raw_bytes),
               static_cast<unsigned long long>(
                   capture_result.statistics.read_timeouts));
  return 0;
}
