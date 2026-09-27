#include "motorevo_ros2/meow_usb_abi.hpp"
#include "motorevo_ros2/transport.hpp"

#include <algorithm>
#include <array>
#include <dlfcn.h>
#include <mutex>
#include <stdexcept>
#include <string>

namespace motorevo_ros2
{
namespace
{

uint8_t length_to_dlc(std::size_t size)
{
  if (size <= 8) {return static_cast<uint8_t>(size);}
  if (size == 12) {return 9;}
  if (size == 16) {return 10;}
  if (size == 20) {return 11;}
  if (size == 24) {return 12;}
  if (size == 32) {return 13;}
  if (size == 48) {return 14;}
  if (size == 64) {return 15;}
  throw std::invalid_argument("invalid CAN FD payload length");
}

std::size_t dlc_to_length(uint8_t dlc)
{
  static constexpr std::array<std::size_t, 16> lengths =
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  if (dlc >= lengths.size()) {throw std::runtime_error("invalid DLC from USB2FDCAN");}
  return lengths[dlc];
}

uint8_t bitrate_code(uint32_t bitrate)
{
  switch (bitrate) {
    case 500000: return 0;
    case 1000000: return 1;
    case 2000000: return 2;
    case 4000000: return 3;
    case 5000000: return 4;
    default: throw std::invalid_argument("Meow USB2FDCAN supports 500k/1M/2M/4M/5M only");
  }
}

class MeowUsbTransport final : public Transport
{
public:
  ~MeowUsbTransport() override {close();}

  void open(const TransportConfig & config) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    close_unlocked();
    library_ = dlopen(config.library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library_) {
      throw std::runtime_error(
              "cannot load Meow USB2FDCAN SDK '" + config.library_path + "': " + dlerror());
    }
    try {
      open_fn_ = load_symbol<OpenFn>("openUSBCAN");
      close_fn_ = load_symbol<CloseFn>("closeUSBCAN");
      send_fn_ = load_symbol<SendFn>("sendUSBCAN");
      read_fn_ = load_symbol<ReadFn>("readUSBCAN");
      config_fn_ = load_symbol<ConfigFn>("configUSBCAN");
      handle_ = open_fn_(config.device.c_str());
      if (handle_ < 0) {
        throw std::runtime_error("cannot open Meow USB2FDCAN device " + config.device);
      }
      const uint8_t type = config.can_fd ?
        (config.bitrate_switch ? kMeowFdCanBrs : kMeowFdCan) : kMeowClassicCan;
      const int rc = config_fn_(
        handle_, config.channel, type, bitrate_code(config.nominal_bitrate),
        bitrate_code(config.data_bitrate));
      if (rc < 0) {
        throw std::runtime_error("configUSBCAN failed with code " + std::to_string(rc));
      }
      channel_ = config.channel;
    } catch (...) {
      close_unlocked();
      throw;
    }
  }

  void close() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    close_unlocked();
  }

  void send(const Frame & frame) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_open();
    MeowFrameInfo info{
      frame.id, static_cast<uint8_t>(frame.extended ? kMeowExtended : kMeowStandard),
      length_to_dlc(frame.size)};
    auto data = frame.data;
    const int rc = send_fn_(handle_, channel_, &info, data.data());
    if (rc < 0) {
      throw std::runtime_error("sendUSBCAN failed with code " + std::to_string(rc));
    }
  }

  bool receive(Frame & frame, int timeout_us) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_open();
    uint8_t rx_channel = 0;
    MeowFrameInfo info{};
    std::array<uint8_t, 64> data{};
    const int rc = read_fn_(handle_, &rx_channel, &info, data.data(), timeout_us);
    if (rc == -1) {return false;}
    if (rc < -1) {
      throw std::runtime_error("readUSBCAN failed with code " + std::to_string(rc));
    }
    if (rx_channel != channel_) {return false;}
    frame = {};
    frame.id = info.can_id;
    frame.extended = info.frame_type == kMeowExtended;
    frame.size = dlc_to_length(info.data_length);
    frame.fd = frame.size > 8;
    std::copy_n(data.begin(), frame.size, frame.data.begin());
    return true;
  }

  bool is_open() const noexcept override {return handle_ >= 0;}

private:
  using OpenFn = int32_t (*)(const char *);
  using CloseFn = int32_t (*)(int32_t);
  using SendFn = int32_t (*)(int32_t, uint8_t, MeowFrameInfo *, uint8_t *);
  using ReadFn = int32_t (*)(int32_t, uint8_t *, MeowFrameInfo *, uint8_t *, int32_t);
  using ConfigFn = int32_t (*)(int32_t, uint8_t, uint8_t, uint8_t, uint8_t);

  template<typename T>
  T load_symbol(const char * name)
  {
    dlerror();
    void * symbol = dlsym(library_, name);
    if (const char * error = dlerror()) {
      throw std::runtime_error(std::string("missing SDK symbol ") + name + ": " + error);
    }
    return reinterpret_cast<T>(symbol);
  }

  void require_open() const
  {
    if (handle_ < 0) {throw std::runtime_error("USB2FDCAN transport is not open");}
  }

  void close_unlocked() noexcept
  {
    if (handle_ >= 0 && close_fn_) {close_fn_(handle_);}
    handle_ = -1;
    if (library_) {dlclose(library_);}
    library_ = nullptr;
    open_fn_ = nullptr;
    close_fn_ = nullptr;
    send_fn_ = nullptr;
    read_fn_ = nullptr;
    config_fn_ = nullptr;
  }

  mutable std::mutex mutex_;
  void * library_{nullptr};
  int32_t handle_{-1};
  uint8_t channel_{1};
  OpenFn open_fn_{nullptr};
  CloseFn close_fn_{nullptr};
  SendFn send_fn_{nullptr};
  ReadFn read_fn_{nullptr};
  ConfigFn config_fn_{nullptr};
};

}  // namespace

std::unique_ptr<Transport> make_meow_usb_transport()
{
  return std::make_unique<MeowUsbTransport>();
}

}  // namespace motorevo_ros2
