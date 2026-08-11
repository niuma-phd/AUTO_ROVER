#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <sys/types.h>

namespace auto_rover {
namespace wheeltec_serial {

enum class TransportStatus : std::uint8_t {
  kOk = 0,
  kWouldBlock,
  kDeadlineExceeded,
  kDisconnected,
  kDisabled,
  kInvalidArgument,
  kIoError,
};

struct IoResult {
  TransportStatus status{TransportStatus::kIoError};
  std::size_t transferred{0U};
  int os_error{0};
  bool delivery_unconfirmed{false};
};

struct WriteOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<ssize_t(int, const std::uint8_t*, std::size_t)> write_bytes;
  std::function<int(int, std::int64_t)> wait_writable;
};

std::int64_t monotonicNowNs() noexcept;
bool configureSerial115200EightNOne(int fd) noexcept;
IoResult writeAllWithOperations(int fd, const std::uint8_t* data,
                                std::size_t size,
                                std::int64_t absolute_deadline_ns,
                                const WriteOperations& operations);

class ByteTransport {
 public:
  virtual ~ByteTransport() = default;
  // Implementations must report all write progress in IoResult.  The adapter
  // catches an exception as an unknown-delivery stream poison, but a
  // transport must not throw after hiding a recoverable progress count.
  virtual bool isConnected() const = 0;
  virtual std::uint64_t connectionGeneration() const = 0;
  virtual IoResult writeAll(const std::uint8_t* data, std::size_t size,
                            std::int64_t absolute_deadline_ns) = 0;
  virtual IoResult readSome(std::uint8_t* data, std::size_t capacity,
                            std::int64_t absolute_deadline_ns) = 0;
};

class PreparedPhysicalSerialOpen;

class PosixFdTransport final : public ByteTransport {
 public:
  PosixFdTransport(int fd, bool owns_fd, std::uint64_t generation,
                   bool writes_enabled = false) noexcept;
  ~PosixFdTransport() noexcept override;

  PosixFdTransport(const PosixFdTransport&) = delete;
  PosixFdTransport& operator=(const PosixFdTransport&) = delete;

  bool isConnected() const override;
  std::uint64_t connectionGeneration() const override;
  IoResult writeAll(const std::uint8_t* data, std::size_t size,
                    std::int64_t absolute_deadline_ns) noexcept override;
  IoResult readSome(std::uint8_t* data, std::size_t capacity,
                    std::int64_t absolute_deadline_ns) noexcept override;

 private:
  friend class PreparedPhysicalSerialOpen;

  // Transfers ownership only after the descriptor has been fully validated.
  // On false, the caller retains ownership and must close the descriptor.
  bool adoptOwnedFdNoexcept(int fd) noexcept;
  void markDisconnected() noexcept;

  int fd_{-1};
  bool owns_fd_{false};
  bool connected_{false};
  bool writes_enabled_{true};
  std::uint64_t generation_{0U};
};

enum class PhysicalAccessMode : std::uint8_t {
  kDisabled = 0,
  kFeedbackOnly,
  kActuation,
};

// Phase-1 release freeze: no runtime parameter, launch file, or caller option
// can enable physical writes.  Changing this constant requires a new safety
// ADR plus firmware-watchdog and parser-resynchronization acceptance evidence.
constexpr bool kPhysicalActuationReleaseEnabled = false;

constexpr std::uint64_t kUnspecifiedPhysicalIdentity =
    static_cast<std::uint64_t>(-1);

struct PhysicalDeviceOptions {
  std::string device_path;
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  PhysicalAccessMode access_mode{PhysicalAccessMode::kDisabled};
  bool actuation_opt_in{false};
  std::uint64_t expected_device_major{kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_device_minor{kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_owner_uid{kUnspecifiedPhysicalIdentity};
  std::uint64_t expected_group_gid{kUnspecifiedPhysicalIdentity};
  std::string expected_usb_vendor_id;
  std::string expected_usb_product_id;
  std::string expected_usb_serial;
};

struct PhysicalFileIdentity {
  bool is_symlink{false};
  bool is_character_device{false};
  std::uint64_t filesystem_device{0U};
  std::uint64_t inode{0U};
  std::uint64_t device_major{0U};
  std::uint64_t device_minor{0U};
  std::uint64_t owner_uid{0U};
  std::uint64_t group_gid{0U};
  std::uint32_t permission_bits{0U};
};

struct UsbDeviceIdentity {
  std::string vendor_id;
  std::string product_id;
  std::string serial;
};

using SysfsTextReader =
    std::function<bool(const std::string&, std::string*)>;

using SysfsPathResolver =
    std::function<bool(const std::string&, std::string*)>;

// Preparation hooks exist so identity pinning can be tested without a physical
// USB device.  They are called only during preparation, before raw ::open.
struct PhysicalPreparationOperations {
  SysfsPathResolver resolve_sysfs_device_path;
  SysfsTextReader read_sysfs_text;
};

struct PhysicalOpenResult {
  TransportStatus status{TransportStatus::kDisabled};
  int os_error{0};
  std::unique_ptr<ByteTransport> transport;
};

// Two-phase opening keeps all allocation and sysfs/string work before the
// externally significant device open.  open() is one-shot and noexcept: after
// raw ::open it uses only fixed storage and system calls, and an fd guard owns
// the descriptor until noexcept transport adoption succeeds.
class PreparedPhysicalSerialOpen final {
 public:
  explicit PreparedPhysicalSerialOpen(
      const PhysicalDeviceOptions& options) noexcept;
  PreparedPhysicalSerialOpen(
      const PhysicalDeviceOptions& options,
      const PhysicalPreparationOperations& operations) noexcept;
  ~PreparedPhysicalSerialOpen() noexcept;

  PreparedPhysicalSerialOpen(const PreparedPhysicalSerialOpen&) = delete;
  PreparedPhysicalSerialOpen& operator=(
      const PreparedPhysicalSerialOpen&) = delete;
  PreparedPhysicalSerialOpen(PreparedPhysicalSerialOpen&&) = delete;
  PreparedPhysicalSerialOpen& operator=(
      PreparedPhysicalSerialOpen&&) = delete;

  bool prepared() const noexcept;
  TransportStatus preparationStatus() const noexcept;
  int preparationError() const noexcept;
  PhysicalOpenResult open() noexcept;

 private:
  struct Implementation;

  void prepare(const PhysicalDeviceOptions& options,
               const PhysicalPreparationOperations* operations) noexcept;

  std::unique_ptr<Implementation> implementation_;
  TransportStatus preparation_status_{TransportStatus::kIoError};
  int preparation_error_{0};
};

bool physicalDeviceOptionsAreComplete(const PhysicalDeviceOptions& options);
bool validatePhysicalDeviceFileIdentity(
    const PhysicalDeviceOptions& options,
    const PhysicalFileIdentity& before_open,
    const PhysicalFileIdentity& opened_fd,
    const PhysicalFileIdentity& after_open,
    bool is_tty);
bool readUsbDeviceIdentityFromAncestors(
    const std::string& sysfs_device_path,
    const SysfsTextReader& reader,
    UsbDeviceIdentity* identity);
bool usbDeviceIdentityMatches(const PhysicalDeviceOptions& options,
                              const UsbDeviceIdentity& identity);
PhysicalOpenResult openPhysicalSerial(const PhysicalDeviceOptions& options);

}  // namespace wheeltec_serial
}  // namespace auto_rover
