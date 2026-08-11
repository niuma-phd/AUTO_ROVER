#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <climits>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <new>
#include <poll.h>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

bool isDisconnectError(int error_number) noexcept {
  return error_number == EBADF || error_number == EPIPE ||
         error_number == ENODEV || error_number == ENXIO ||
         error_number == EIO;
}

int waitForFd(int fd, short requested_events,
              std::int64_t absolute_deadline_ns) noexcept {
  for (;;) {
    const std::int64_t now_ns = monotonicNowNs();
    if (now_ns >= absolute_deadline_ns) {
      return 0;
    }
    const std::int64_t remaining_ns = absolute_deadline_ns - now_ns;
    std::int64_t timeout_ms = remaining_ns / 1000000;
    if (remaining_ns % 1000000 != 0) {
      ++timeout_ms;
    }
    if (timeout_ms > INT_MAX) {
      timeout_ms = INT_MAX;
    }
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = requested_events;
    const int poll_result =
        ::poll(&descriptor, 1U, static_cast<int>(timeout_ms));
    if (poll_result > 0) {
      if ((descriptor.revents & requested_events) != 0) {
        return 1;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        errno = EPIPE;
        return -1;
      }
      continue;
    }
    if (poll_result == 0) {
      return 0;
    }
    if (errno != EINTR) {
      return -1;
    }
  }
}

IoResult ioFailure(TransportStatus status, std::size_t transferred,
                   int os_error) noexcept {
  return {status, transferred, os_error,
          status == TransportStatus::kDisconnected || transferred != 0U};
}

IoResult writeAllWithSystemCalls(
    int fd, const std::uint8_t* data, std::size_t size,
    std::int64_t absolute_deadline_ns) noexcept {
  if (fd < 0 || (data == nullptr && size != 0U)) {
    return ioFailure(TransportStatus::kInvalidArgument, 0U, EINVAL);
  }
  if (size == 0U) {
    return {TransportStatus::kOk, 0U, 0, false};
  }

  std::size_t transferred = 0U;
  while (transferred < size) {
    if (monotonicNowNs() >= absolute_deadline_ns) {
      return ioFailure(TransportStatus::kDeadlineExceeded, transferred,
                       ETIMEDOUT);
    }
    const ssize_t write_result =
        ::write(fd, data + transferred, size - transferred);
    if (write_result > 0) {
      const std::size_t progress = static_cast<std::size_t>(write_result);
      if (progress > size - transferred) {
        return ioFailure(TransportStatus::kIoError, transferred, EIO);
      }
      transferred += progress;
      continue;
    }
    if (write_result == 0) {
      return ioFailure(TransportStatus::kDisconnected, transferred, EPIPE);
    }

    const int write_error = errno;
    if (write_error == EINTR) {
      continue;
    }
    if (write_error == EAGAIN || write_error == EWOULDBLOCK) {
      const int wait_result =
          waitForFd(fd, POLLOUT, absolute_deadline_ns);
      if (wait_result > 0) {
        continue;
      }
      if (wait_result == 0) {
        return ioFailure(TransportStatus::kDeadlineExceeded, transferred,
                         ETIMEDOUT);
      }
      const int wait_error = errno;
      if (isDisconnectError(wait_error)) {
        return ioFailure(TransportStatus::kDisconnected, transferred,
                         wait_error);
      }
      return ioFailure(TransportStatus::kIoError, transferred, wait_error);
    }
    if (isDisconnectError(write_error)) {
      return ioFailure(TransportStatus::kDisconnected, transferred,
                       write_error);
    }
    return ioFailure(TransportStatus::kIoError, transferred, write_error);
  }
  return {TransportStatus::kOk, transferred, 0, false};
}

bool physicalPathIsConstrained(const std::string& path) noexcept {
  if (path.size() <= 5U || path[0U] != '/' || path[1U] != 'd' ||
      path[2U] != 'e' || path[3U] != 'v' || path[4U] != '/' ||
      path.back() == '/') {
    return false;
  }
  std::size_t component_begin = 5U;
  for (std::size_t index = component_begin; index <= path.size(); ++index) {
    if (index < path.size() && path[index] == '\0') {
      return false;
    }
    if (index != path.size() && path[index] != '/') {
      continue;
    }
    const std::size_t component_size = index - component_begin;
    if (component_size == 0U ||
        (component_size == 1U && path[component_begin] == '.') ||
        (component_size == 2U && path[component_begin] == '.' &&
         path[component_begin + 1U] == '.')) {
      return false;
    }
    component_begin = index + 1U;
  }
  return true;
}

PhysicalFileIdentity identityFromStat(const struct stat& metadata) noexcept {
  PhysicalFileIdentity identity;
  identity.is_symlink = S_ISLNK(metadata.st_mode);
  identity.is_character_device = S_ISCHR(metadata.st_mode);
  identity.filesystem_device =
      static_cast<std::uint64_t>(metadata.st_dev);
  identity.inode = static_cast<std::uint64_t>(metadata.st_ino);
  identity.device_major =
      static_cast<std::uint64_t>(::major(metadata.st_rdev));
  identity.device_minor =
      static_cast<std::uint64_t>(::minor(metadata.st_rdev));
  identity.owner_uid = static_cast<std::uint64_t>(metadata.st_uid);
  identity.group_gid = static_cast<std::uint64_t>(metadata.st_gid);
  identity.permission_bits =
      static_cast<std::uint32_t>(metadata.st_mode & 07777);
  return identity;
}

int lstatWithoutInterruption(const char* path,
                             struct stat* metadata) noexcept {
  int status = -1;
  do {
    status = ::lstat(path, metadata);
  } while (status < 0 && errno == EINTR);
  return status;
}

bool copyPhysicalPath(const std::string& path,
                      std::array<char, PATH_MAX>* copied) noexcept {
  if (copied == nullptr || !physicalPathIsConstrained(path) ||
      path.size() >= copied->size()) {
    errno = EINVAL;
    return false;
  }
  std::memcpy(copied->data(), path.data(), path.size());
  (*copied)[path.size()] = '\0';
  return true;
}

bool readPathIdentityWithoutSymlinks(
    const std::array<char, PATH_MAX>& path,
    PhysicalFileIdentity* identity) noexcept {
  if (identity == nullptr) {
    errno = EINVAL;
    return false;
  }
  const std::size_t path_size = ::strnlen(path.data(), path.size());
  if (path_size <= 5U || path_size >= path.size()) {
    errno = EINVAL;
    return false;
  }

  std::array<char, PATH_MAX> current = path;
  struct stat metadata {};
  current[4U] = '\0';
  if (lstatWithoutInterruption(current.data(), &metadata) != 0 ||
      S_ISLNK(metadata.st_mode) || !S_ISDIR(metadata.st_mode)) {
    if (errno == 0) {
      errno = ELOOP;
    }
    return false;
  }
  current[4U] = '/';

  std::size_t component_begin = 5U;
  for (std::size_t index = component_begin; index <= path_size; ++index) {
    if (index != path_size && current[index] != '/') {
      continue;
    }
    const std::size_t component_size = index - component_begin;
    if (component_size == 0U ||
        (component_size == 1U && current[component_begin] == '.') ||
        (component_size == 2U && current[component_begin] == '.' &&
         current[component_begin + 1U] == '.')) {
      errno = EINVAL;
      return false;
    }
    const bool final_component = index == path_size;
    const char saved = current[index];
    current[index] = '\0';
    if (lstatWithoutInterruption(current.data(), &metadata) != 0) {
      current[index] = saved;
      return false;
    }
    current[index] = saved;
    if (S_ISLNK(metadata.st_mode)) {
      errno = ELOOP;
      return false;
    }
    if (final_component) {
      *identity = identityFromStat(metadata);
      return true;
    }
    if (!S_ISDIR(metadata.st_mode)) {
      errno = ENOTDIR;
      return false;
    }
    component_begin = index + 1U;
  }
  errno = EINVAL;
  return false;
}

bool readPhysicalPathIdentity(
    const std::array<char, PATH_MAX>& path,
    PhysicalPathIdentityReader injected_reader,
    void* injected_context,
    PhysicalFileIdentity* identity) noexcept {
  if (injected_reader == nullptr) {
    return readPathIdentityWithoutSymlinks(path, identity);
  }
  try {
    return injected_reader(path.data(), identity, injected_context);
  } catch (...) {
    errno = EIO;
    return false;
  }
}

class ScopedFd final {
 public:
  explicit ScopedFd(int fd) noexcept : fd_(fd) {}
  ~ScopedFd() noexcept {
    if (fd_ >= 0) {
      // A close reported as EINTR has unspecified descriptor state; retrying
      // could close an unrelated descriptor that another thread just reused.
      (void)::close(fd_);
    }
  }

  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;

  int get() const noexcept { return fd_; }
  int release() noexcept {
    const int released = fd_;
    fd_ = -1;
    return released;
  }

 private:
  int fd_{-1};
};

bool validHexIdentifier(const std::string& value) {
  if (value.size() != 4U) {
    return false;
  }
  for (const char character : value) {
    if (std::isxdigit(static_cast<unsigned char>(character)) == 0) {
      return false;
    }
  }
  return true;
}

std::string lowerHexIdentifier(const std::string& value) {
  std::string lowered = value;
  for (char& character : lowered) {
    character = static_cast<char>(
        std::tolower(static_cast<unsigned char>(character)));
  }
  return lowered;
}

void trimAsciiWhitespace(std::string* value) {
  if (value == nullptr) {
    return;
  }
  std::size_t begin = 0U;
  while (begin < value->size() &&
         std::isspace(static_cast<unsigned char>((*value)[begin])) != 0) {
    ++begin;
  }
  std::size_t end = value->size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>((*value)[end - 1U])) != 0) {
    --end;
  }
  *value = value->substr(begin, end - begin);
}

bool validExpectedSerial(const std::string& serial) {
  if (serial.empty() || serial.size() > 128U ||
      std::isspace(static_cast<unsigned char>(serial.front())) != 0 ||
      std::isspace(static_cast<unsigned char>(serial.back())) != 0) {
    return false;
  }
  for (const char character : serial) {
    const unsigned char byte = static_cast<unsigned char>(character);
    if (byte < 0x20U || byte > 0x7EU) {
      return false;
    }
  }
  return true;
}

bool readBoundedTextFileNoFollow(const std::string& path,
                                 std::string* value) {
  if (value == nullptr) {
    errno = EINVAL;
    return false;
  }
  int fd = -1;
  do {
    fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  } while (fd < 0 && errno == EINTR);
  if (fd < 0) {
    return false;
  }

  struct stat metadata {};
  int stat_status = -1;
  do {
    stat_status = ::fstat(fd, &metadata);
  } while (stat_status < 0 && errno == EINTR);
  if (stat_status != 0 || !S_ISREG(metadata.st_mode)) {
    const int saved_error = stat_status == 0 ? EINVAL : errno;
    ::close(fd);
    errno = saved_error;
    return false;
  }

  constexpr std::size_t kMaximumAttributeBytes = 256U;
  char buffer[kMaximumAttributeBytes + 1U]{};
  std::size_t used = 0U;
  while (used < sizeof(buffer)) {
    const ssize_t count = ::read(fd, buffer + used, sizeof(buffer) - used);
    if (count > 0) {
      used += static_cast<std::size_t>(count);
      continue;
    }
    if (count == 0) {
      break;
    }
    if (errno != EINTR) {
      const int saved_error = errno;
      ::close(fd);
      errno = saved_error;
      return false;
    }
  }
  const int close_status = ::close(fd);
  if (close_status != 0 || used > kMaximumAttributeBytes) {
    if (close_status == 0) {
      errno = EOVERFLOW;
    }
    return false;
  }
  value->assign(buffer, used);
  trimAsciiWhitespace(value);
  return !value->empty();
}

bool resolveSysfsDevicePath(const std::string& device_link,
                            std::string* resolved_path) {
  if (resolved_path == nullptr ||
      device_link.compare(0U, 14U, "/sys/dev/char/") != 0 ||
      device_link.find("..") != std::string::npos) {
    errno = EINVAL;
    return false;
  }
  char buffer[PATH_MAX]{};
  if (::realpath(device_link.c_str(), buffer) == nullptr) {
    return false;
  }
  const std::string resolved(buffer);
  if (resolved != "/sys/devices" &&
      resolved.compare(0U, 13U, "/sys/devices/") != 0) {
    errno = EPERM;
    return false;
  }
  *resolved_path = resolved;
  return true;
}

bool sameFileIdentity(const PhysicalFileIdentity& first,
                      const PhysicalFileIdentity& second) {
  return first.is_symlink == second.is_symlink &&
         first.is_character_device == second.is_character_device &&
         first.filesystem_device == second.filesystem_device &&
         first.inode == second.inode &&
         first.device_major == second.device_major &&
         first.device_minor == second.device_minor &&
         first.owner_uid == second.owner_uid &&
         first.group_gid == second.group_gid &&
         first.permission_bits == second.permission_bits;
}

bool fileIdentityMatchesOptions(const PhysicalDeviceOptions& options,
                                const PhysicalFileIdentity& identity) {
  constexpr std::uint32_t kRequiredOwnerPermissions = 0600U;
  constexpr std::uint32_t kAllowedPermissions = 0660U;
  return !identity.is_symlink && identity.is_character_device &&
         identity.device_major == options.expected_device_major &&
         identity.device_minor == options.expected_device_minor &&
         identity.owner_uid == options.expected_owner_uid &&
         identity.group_gid == options.expected_group_gid &&
         (identity.permission_bits & kRequiredOwnerPermissions) ==
             kRequiredOwnerPermissions &&
         (identity.permission_bits & ~kAllowedPermissions) == 0U;
}

bool physicalAccessIsAuthorized(const PhysicalDeviceOptions& options) {
  if (!options.unverified_protocol_acknowledged ||
      !options.physical_device_opt_in) {
    return false;
  }
  switch (options.access_mode) {
    case PhysicalAccessMode::kFeedbackOnly:
      return true;
    case PhysicalAccessMode::kActuation:
      return kPhysicalActuationReleaseEnabled && options.actuation_opt_in;
    case PhysicalAccessMode::kDisabled:
    default:
      return false;
  }
}

TransportStatus statusForOpenError(int error_number) {
  return isDisconnectError(error_number) || error_number == ENOENT
             ? TransportStatus::kDisconnected
             : TransportStatus::kIoError;
}

bool serialSettingsAre115200EightNOne(const termios& settings) {
  bool no_hardware_flow_control = true;
#ifdef CRTSCTS
  no_hardware_flow_control = (settings.c_cflag & CRTSCTS) == 0;
#endif
  return ::cfgetispeed(&settings) == B115200 &&
         ::cfgetospeed(&settings) == B115200 &&
         (settings.c_cflag & CSIZE) == CS8 &&
         (settings.c_cflag & (PARENB | CSTOPB)) == 0 &&
         (settings.c_cflag & (CLOCAL | CREAD)) == (CLOCAL | CREAD) &&
         no_hardware_flow_control && settings.c_cc[VMIN] == 0 &&
         settings.c_cc[VTIME] == 0;
}

}  // namespace

bool configureSerial115200EightNOne(int fd) noexcept {
  if (fd < 0) {
    errno = EINVAL;
    return false;
  }
  termios settings{};
  int get_status = -1;
  do {
    get_status = ::tcgetattr(fd, &settings);
  } while (get_status < 0 && errno == EINTR);
  if (get_status != 0) {
    return false;
  }
  ::cfmakeraw(&settings);
  if (::cfsetispeed(&settings, B115200) != 0 ||
      ::cfsetospeed(&settings, B115200) != 0) {
    return false;
  }
  settings.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
  settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
#ifdef CRTSCTS
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  int set_status = -1;
  do {
    set_status = ::tcsetattr(fd, TCSANOW, &settings);
  } while (set_status < 0 && errno == EINTR);
  if (set_status != 0) {
    return false;
  }

  termios observed{};
  do {
    get_status = ::tcgetattr(fd, &observed);
  } while (get_status < 0 && errno == EINTR);
  if (get_status != 0) {
    return false;
  }
  if (!serialSettingsAre115200EightNOne(observed)) {
    errno = EIO;
    return false;
  }
  return true;
}

bool physicalDeviceOptionsAreComplete(
    const PhysicalDeviceOptions& options) {
  if (!physicalAccessIsAuthorized(options) ||
      !physicalPathIsConstrained(options.device_path) ||
      options.expected_device_major == kUnspecifiedPhysicalIdentity ||
      options.expected_device_minor == kUnspecifiedPhysicalIdentity ||
      options.expected_owner_uid == kUnspecifiedPhysicalIdentity ||
      options.expected_group_gid == kUnspecifiedPhysicalIdentity ||
      options.expected_device_major >
          std::numeric_limits<unsigned int>::max() ||
      options.expected_device_minor >
          std::numeric_limits<unsigned int>::max() ||
      options.expected_owner_uid > std::numeric_limits<uid_t>::max() ||
      options.expected_group_gid > std::numeric_limits<gid_t>::max() ||
      !validHexIdentifier(options.expected_usb_vendor_id) ||
      !validHexIdentifier(options.expected_usb_product_id) ||
      !validExpectedSerial(options.expected_usb_serial)) {
    return false;
  }
  const dev_t combined = ::makedev(
      static_cast<unsigned int>(options.expected_device_major),
      static_cast<unsigned int>(options.expected_device_minor));
  return static_cast<std::uint64_t>(::major(combined)) ==
             options.expected_device_major &&
         static_cast<std::uint64_t>(::minor(combined)) ==
             options.expected_device_minor;
}

bool validatePhysicalDeviceFileIdentity(
    const PhysicalDeviceOptions& options,
    const PhysicalFileIdentity& before_open,
    const PhysicalFileIdentity& opened_fd,
    const PhysicalFileIdentity& after_open,
    bool is_tty) {
  return physicalDeviceOptionsAreComplete(options) && is_tty &&
         fileIdentityMatchesOptions(options, before_open) &&
         fileIdentityMatchesOptions(options, opened_fd) &&
         fileIdentityMatchesOptions(options, after_open) &&
         sameFileIdentity(before_open, opened_fd) &&
         sameFileIdentity(opened_fd, after_open);
}

bool readUsbDeviceIdentityFromAncestors(
    const std::string& sysfs_device_path,
    const SysfsTextReader& reader,
    UsbDeviceIdentity* identity) {
  constexpr std::size_t kMaximumParentDepth = 12U;
  if (identity == nullptr || !reader || sysfs_device_path.empty() ||
      (sysfs_device_path.compare(0U, 14U, "/sys/dev/char/") != 0 &&
       sysfs_device_path.compare(0U, 13U, "/sys/devices/") != 0) ||
      sysfs_device_path.find("..") != std::string::npos ||
      sysfs_device_path.back() == '/') {
    return false;
  }

  std::string current = sysfs_device_path;
  for (std::size_t depth = 0U; depth <= kMaximumParentDepth; ++depth) {
    UsbDeviceIdentity candidate;
    const bool has_vendor =
        reader(current + "/idVendor", &candidate.vendor_id);
    const bool has_product =
        reader(current + "/idProduct", &candidate.product_id);
    const bool has_serial = reader(current + "/serial", &candidate.serial);
    if (has_vendor && has_product && has_serial) {
      trimAsciiWhitespace(&candidate.vendor_id);
      trimAsciiWhitespace(&candidate.product_id);
      trimAsciiWhitespace(&candidate.serial);
      if (candidate.vendor_id.empty() || candidate.product_id.empty() ||
          candidate.serial.empty()) {
        return false;
      }
      *identity = candidate;
      return true;
    }
    if (current == "/sys/devices" || current == "/sys") {
      break;
    }
    const std::size_t separator = current.find_last_of('/');
    if (separator == std::string::npos || separator == 0U) {
      break;
    }
    current.erase(separator);
  }
  return false;
}

bool usbDeviceIdentityMatches(const PhysicalDeviceOptions& options,
                              const UsbDeviceIdentity& identity) {
  std::string vendor = identity.vendor_id;
  std::string product = identity.product_id;
  std::string serial = identity.serial;
  trimAsciiWhitespace(&vendor);
  trimAsciiWhitespace(&product);
  trimAsciiWhitespace(&serial);
  return validHexIdentifier(options.expected_usb_vendor_id) &&
         validHexIdentifier(options.expected_usb_product_id) &&
         validHexIdentifier(vendor) && validHexIdentifier(product) &&
         validExpectedSerial(options.expected_usb_serial) &&
         validExpectedSerial(serial) &&
         lowerHexIdentifier(vendor) ==
             lowerHexIdentifier(options.expected_usb_vendor_id) &&
         lowerHexIdentifier(product) ==
             lowerHexIdentifier(options.expected_usb_product_id) &&
         serial == options.expected_usb_serial;
}

std::int64_t monotonicNowNs() noexcept {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    return 0;
  }
  return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL +
         static_cast<std::int64_t>(value.tv_nsec);
}

IoResult writeAllWithOperations(int fd, const std::uint8_t* data,
                                std::size_t size,
                                std::int64_t absolute_deadline_ns,
                                const WriteOperations& operations) {
  if (fd < 0 || (data == nullptr && size != 0U) ||
      !operations.monotonic_now_ns || !operations.write_bytes ||
      !operations.wait_writable) {
    return ioFailure(TransportStatus::kInvalidArgument, 0U, EINVAL);
  }
  if (size == 0U) {
    return {TransportStatus::kOk, 0U, 0, false};
  }

  std::size_t transferred = 0U;
  while (transferred < size) {
    if (operations.monotonic_now_ns() >= absolute_deadline_ns) {
      return ioFailure(TransportStatus::kDeadlineExceeded, transferred,
                       ETIMEDOUT);
    }
    const ssize_t write_result =
        operations.write_bytes(fd, data + transferred, size - transferred);
    if (write_result > 0) {
      const std::size_t progress = static_cast<std::size_t>(write_result);
      if (progress > size - transferred) {
        return ioFailure(TransportStatus::kIoError, transferred, EIO);
      }
      transferred += progress;
      continue;
    }
    if (write_result == 0) {
      return ioFailure(TransportStatus::kDisconnected, transferred, EPIPE);
    }

    const int write_error = errno;
    if (write_error == EINTR) {
      continue;
    }
    if (write_error == EAGAIN || write_error == EWOULDBLOCK) {
      const int wait_result =
          operations.wait_writable(fd, absolute_deadline_ns);
      if (wait_result > 0) {
        continue;
      }
      if (wait_result == 0) {
        return ioFailure(TransportStatus::kDeadlineExceeded, transferred,
                         ETIMEDOUT);
      }
      const int wait_error = errno;
      if (wait_error == EINTR) {
        continue;
      }
      if (isDisconnectError(wait_error)) {
        return ioFailure(TransportStatus::kDisconnected, transferred,
                         wait_error);
      }
      return ioFailure(TransportStatus::kIoError, transferred, wait_error);
    }
    if (isDisconnectError(write_error)) {
      return ioFailure(TransportStatus::kDisconnected, transferred,
                       write_error);
    }
    return ioFailure(TransportStatus::kIoError, transferred, write_error);
  }
  return {TransportStatus::kOk, transferred, 0, false};
}

PosixFdTransport::PosixFdTransport(int fd, bool owns_fd,
                                   std::uint64_t generation,
                                   bool writes_enabled) noexcept
    : fd_(fd),
      owns_fd_(owns_fd),
      connected_(false),
      writes_enabled_(writes_enabled),
      generation_(generation) {
  if (fd_ < 0) {
    return;
  }
  int flags = -1;
  do {
    flags = ::fcntl(fd_, F_GETFL);
  } while (flags < 0 && errno == EINTR);
  if (flags < 0) {
    markDisconnected();
    return;
  }
  if ((flags & O_NONBLOCK) == 0) {
    int set_result = -1;
    do {
      set_result = ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    } while (set_result < 0 && errno == EINTR);
    if (set_result < 0) {
      markDisconnected();
      return;
    }
  }
  connected_ = true;
}

PosixFdTransport::~PosixFdTransport() noexcept {
  if (owns_fd_ && fd_ >= 0) {
    ::close(fd_);
  }
}

bool PosixFdTransport::isConnected() const { return connected_; }

std::uint64_t PosixFdTransport::connectionGeneration() const {
  return generation_;
}

IoResult PosixFdTransport::writeAll(const std::uint8_t* data,
                                    std::size_t size,
                                    std::int64_t absolute_deadline_ns) noexcept {
  if (!connected_) {
    return ioFailure(TransportStatus::kDisconnected, 0U, ENOTCONN);
  }
  if (!writes_enabled_) {
    return ioFailure(TransportStatus::kDisabled, 0U, EACCES);
  }
  IoResult result = writeAllWithSystemCalls(
      fd_, data, size, absolute_deadline_ns);
  if (result.status == TransportStatus::kDisconnected) {
    markDisconnected();
  }
  return result;
}

IoResult PosixFdTransport::readSome(std::uint8_t* data, std::size_t capacity,
                                    std::int64_t absolute_deadline_ns) noexcept {
  if (!connected_) {
    return ioFailure(TransportStatus::kDisconnected, 0U, ENOTCONN);
  }
  if (data == nullptr && capacity != 0U) {
    return ioFailure(TransportStatus::kInvalidArgument, 0U, EINVAL);
  }
  if (capacity == 0U) {
    return {TransportStatus::kOk, 0U, 0, false};
  }
  for (;;) {
    if (monotonicNowNs() >= absolute_deadline_ns) {
      return ioFailure(TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT);
    }
    const ssize_t read_result = ::read(fd_, data, capacity);
    if (read_result > 0) {
      return {TransportStatus::kOk,
              static_cast<std::size_t>(read_result), 0, false};
    }
    const int read_error = read_result == 0 ? EAGAIN : errno;
    if (read_error == EINTR) {
      continue;
    }
    if (read_error == EAGAIN || read_error == EWOULDBLOCK) {
      const int wait_result = waitForFd(fd_, POLLIN, absolute_deadline_ns);
      if (wait_result > 0) {
        continue;
      }
      if (wait_result == 0) {
        return ioFailure(TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT);
      }
      const int wait_error = errno;
      if (wait_error == EINTR) {
        continue;
      }
      if (isDisconnectError(wait_error)) {
        markDisconnected();
        return ioFailure(TransportStatus::kDisconnected, 0U, wait_error);
      }
      return ioFailure(TransportStatus::kIoError, 0U, wait_error);
    }
    if (isDisconnectError(read_error)) {
      markDisconnected();
      return ioFailure(TransportStatus::kDisconnected, 0U, read_error);
    }
    return ioFailure(TransportStatus::kIoError, 0U, read_error);
  }
}

bool PosixFdTransport::adoptOwnedFdNoexcept(int fd) noexcept {
  if (fd < 0 || fd_ >= 0 || owns_fd_ || connected_) {
    return false;
  }
  int flags = -1;
  do {
    flags = ::fcntl(fd, F_GETFL);
  } while (flags < 0 && errno == EINTR);
  if (flags < 0) {
    return false;
  }
  if ((flags & O_NONBLOCK) == 0) {
    int set_result = -1;
    do {
      set_result = ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    } while (set_result < 0 && errno == EINTR);
    if (set_result < 0) {
      return false;
    }
  }
  fd_ = fd;
  owns_fd_ = true;
  connected_ = true;
  return true;
}

void PosixFdTransport::markDisconnected() noexcept {
  connected_ = false;
  if (owns_fd_ && fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

struct PreparedPhysicalSerialOpen::Implementation {
  std::array<char, PATH_MAX> device_path{};
  PhysicalDeviceOptions pinned_options;
  PhysicalFileIdentity before_open{};
  PhysicalPathIdentityReader read_path_identity{nullptr};
  void* path_identity_context{nullptr};
  std::unique_ptr<PosixFdTransport> transport;
  int requested_access{O_RDONLY};
  bool consumed{false};
};

PreparedPhysicalSerialOpen::PreparedPhysicalSerialOpen(
    const PhysicalDeviceOptions& options) noexcept {
  prepare(options, nullptr);
}

PreparedPhysicalSerialOpen::PreparedPhysicalSerialOpen(
    const PhysicalDeviceOptions& options,
    const PhysicalPreparationOperations& operations) noexcept {
  prepare(options, &operations);
}

PreparedPhysicalSerialOpen::~PreparedPhysicalSerialOpen() noexcept = default;

void PreparedPhysicalSerialOpen::prepare(
    const PhysicalDeviceOptions& options,
    const PhysicalPreparationOperations* operations) noexcept {
  implementation_.reset();
  preparation_status_ = TransportStatus::kIoError;
  preparation_error_ = 0;
  try {
    if (!physicalAccessIsAuthorized(options)) {
      preparation_status_ = TransportStatus::kDisabled;
      return;
    }
    if (!physicalDeviceOptionsAreComplete(options)) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EINVAL;
      return;
    }
    if (operations != nullptr &&
        (!operations->resolve_sysfs_device_path ||
         !operations->read_sysfs_text)) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EINVAL;
      return;
    }

    std::array<char, PATH_MAX> copied_path{};
    if (!copyPhysicalPath(options.device_path, &copied_path)) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EINVAL;
      return;
    }
    PhysicalFileIdentity before_open;
    const PhysicalPathIdentityReader path_identity_reader =
        operations == nullptr ? nullptr : operations->read_path_identity;
    void* const path_identity_context =
        operations == nullptr ? nullptr : operations->path_identity_context;
    errno = 0;
    if (!readPhysicalPathIdentity(copied_path, path_identity_reader,
                                  path_identity_context, &before_open)) {
      const int path_error = errno == 0 ? EIO : errno;
      preparation_status_ =
          path_error == ELOOP || path_error == ENOTDIR ||
                  path_error == EINVAL
              ? TransportStatus::kInvalidArgument
              : statusForOpenError(path_error);
      preparation_error_ = path_error;
      return;
    }
    if (!fileIdentityMatchesOptions(options, before_open)) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EPERM;
      return;
    }

    const std::string sysfs_device_link =
        "/sys/dev/char/" +
        std::to_string(options.expected_device_major) + ":" +
        std::to_string(options.expected_device_minor) + "/device";
    std::string resolved_sysfs_device_path;
    bool resolved = false;
    if (operations == nullptr) {
      resolved = resolveSysfsDevicePath(sysfs_device_link,
                                        &resolved_sysfs_device_path);
    } else {
      resolved = operations->resolve_sysfs_device_path(
          sysfs_device_link, &resolved_sysfs_device_path);
    }
    if (!resolved) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EPERM;
      return;
    }

    UsbDeviceIdentity usb_identity;
    bool identity_read = false;
    if (operations == nullptr) {
      const SysfsTextReader reader =
          [](const std::string& path, std::string* value) {
            return readBoundedTextFileNoFollow(path, value);
          };
      identity_read = readUsbDeviceIdentityFromAncestors(
          resolved_sysfs_device_path, reader, &usb_identity);
    } else {
      identity_read = readUsbDeviceIdentityFromAncestors(
          resolved_sysfs_device_path, operations->read_sysfs_text,
          &usb_identity);
    }
    if (!identity_read || !usbDeviceIdentityMatches(options, usb_identity)) {
      preparation_status_ = TransportStatus::kInvalidArgument;
      preparation_error_ = EPERM;
      return;
    }

    std::unique_ptr<Implementation> prepared(
        new (std::nothrow) Implementation());
    if (!prepared) {
      preparation_status_ = TransportStatus::kIoError;
      preparation_error_ = ENOMEM;
      return;
    }
    const bool writes_enabled =
        options.access_mode == PhysicalAccessMode::kActuation;
    prepared->transport.reset(new (std::nothrow) PosixFdTransport(
        -1, false, 1U, writes_enabled));
    if (!prepared->transport) {
      preparation_status_ = TransportStatus::kIoError;
      preparation_error_ = ENOMEM;
      return;
    }
    prepared->device_path = copied_path;
    prepared->pinned_options = options;
    prepared->before_open = before_open;
    prepared->read_path_identity = path_identity_reader;
    prepared->path_identity_context = path_identity_context;
    prepared->requested_access =
        options.access_mode == PhysicalAccessMode::kFeedbackOnly ? O_RDONLY
                                                                 : O_RDWR;
    implementation_ = std::move(prepared);
    preparation_status_ = TransportStatus::kOk;
    preparation_error_ = 0;
  } catch (const std::bad_alloc&) {
    implementation_.reset();
    preparation_status_ = TransportStatus::kIoError;
    preparation_error_ = ENOMEM;
  } catch (...) {
    implementation_.reset();
    preparation_status_ = TransportStatus::kIoError;
    preparation_error_ = EIO;
  }
}

bool PreparedPhysicalSerialOpen::prepared() const noexcept {
  return preparation_status_ == TransportStatus::kOk &&
         implementation_ != nullptr && implementation_->transport != nullptr;
}

TransportStatus PreparedPhysicalSerialOpen::preparationStatus()
    const noexcept {
  return preparation_status_;
}

int PreparedPhysicalSerialOpen::preparationError() const noexcept {
  return preparation_error_;
}

PhysicalOpenResult PreparedPhysicalSerialOpen::open() noexcept {
  PhysicalOpenResult result;
  if (implementation_ != nullptr && implementation_->consumed) {
    result.status = TransportStatus::kInvalidArgument;
    result.os_error = EALREADY;
    return result;
  }
  if (!prepared()) {
    result.status = preparation_status_;
    result.os_error = preparation_error_;
    return result;
  }
  implementation_->consumed = true;

  int raw_fd = -1;
  do {
    raw_fd = ::open(implementation_->device_path.data(),
                    implementation_->requested_access | O_NOCTTY |
                        O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
  } while (raw_fd < 0 && errno == EINTR);
  if (raw_fd < 0) {
    const int open_error = errno == 0 ? EIO : errno;
    result.status = statusForOpenError(open_error);
    result.os_error = open_error;
    return result;
  }
  ScopedFd opened_fd(raw_fd);

  struct stat opened_metadata {};
  int stat_status = -1;
  do {
    stat_status = ::fstat(opened_fd.get(), &opened_metadata);
  } while (stat_status < 0 && errno == EINTR);
  if (stat_status != 0) {
    const int stat_error = errno == 0 ? EIO : errno;
    result.status = statusForOpenError(stat_error);
    result.os_error = stat_error;
    return result;
  }
  const PhysicalFileIdentity observed_fd =
      identityFromStat(opened_metadata);

  errno = 0;
  const bool is_tty = ::isatty(opened_fd.get()) == 1;
  const int tty_error = errno;

  PhysicalFileIdentity after_open;
  errno = 0;
  if (!readPhysicalPathIdentity(
          implementation_->device_path,
          implementation_->read_path_identity,
          implementation_->path_identity_context, &after_open)) {
    const int path_error = errno == 0 ? EIO : errno;
    result.status =
        path_error == ELOOP || path_error == ENOTDIR ||
                path_error == EINVAL
            ? TransportStatus::kInvalidArgument
            : statusForOpenError(path_error);
    result.os_error = path_error;
    return result;
  }

  if (!validatePhysicalDeviceFileIdentity(
          implementation_->pinned_options,
          implementation_->before_open, observed_fd, after_open, is_tty)) {
    result.status = TransportStatus::kInvalidArgument;
    result.os_error = is_tty ? EPERM : (tty_error == 0 ? ENOTTY : tty_error);
    return result;
  }

  int lock_status = -1;
  do {
    lock_status = ::flock(opened_fd.get(), LOCK_EX | LOCK_NB);
  } while (lock_status < 0 && errno == EINTR);
  if (lock_status != 0) {
    result.status = TransportStatus::kIoError;
    result.os_error = errno == 0 ? EBUSY : errno;
    return result;
  }

  int exclusive_status = -1;
  do {
    exclusive_status = ::ioctl(opened_fd.get(), TIOCEXCL);
  } while (exclusive_status < 0 && errno == EINTR);
  if (exclusive_status != 0) {
    result.status = TransportStatus::kIoError;
    result.os_error = errno == 0 ? EIO : errno;
    return result;
  }
  int exclusive_enabled = 0;
  do {
    exclusive_status =
        ::ioctl(opened_fd.get(), TIOCGEXCL, &exclusive_enabled);
  } while (exclusive_status < 0 && errno == EINTR);
  if (exclusive_status != 0 || exclusive_enabled == 0) {
    result.status = TransportStatus::kIoError;
    result.os_error = exclusive_status == 0 || errno == 0 ? EIO : errno;
    return result;
  }

  if (!configureSerial115200EightNOne(opened_fd.get())) {
    result.status = TransportStatus::kIoError;
    result.os_error = errno == 0 ? EIO : errno;
    return result;
  }
  if (!implementation_->transport->adoptOwnedFdNoexcept(opened_fd.get())) {
    result.status = TransportStatus::kIoError;
    result.os_error = errno == 0 ? EIO : errno;
    return result;
  }
  (void)opened_fd.release();
  result.transport.reset(implementation_->transport.release());
  result.status = TransportStatus::kOk;
  result.os_error = 0;
  return result;
}

PhysicalOpenResult openPhysicalSerial(const PhysicalDeviceOptions& options) {
  PreparedPhysicalSerialOpen prepared(options);
  return prepared.open();
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
