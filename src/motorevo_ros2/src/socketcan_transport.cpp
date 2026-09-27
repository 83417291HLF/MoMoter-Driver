#include "motorevo_ros2/transport.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>

namespace motorevo_ros2
{

std::unique_ptr<Transport> make_meow_usb_transport();

namespace
{

#ifdef __linux__
class SocketCanTransport final : public Transport
{
public:
  ~SocketCanTransport() override {close();}

  void open(const TransportConfig & config) override
  {
    close();
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {throw_system("socket(PF_CAN)");}
    try {
      int enable_fd = 1;
      if (config.can_fd &&
        setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
      {
        throw_system("setsockopt(CAN_RAW_FD_FRAMES)");
      }
      ifreq request{};
      if (config.device.size() >= IFNAMSIZ) {
        throw std::invalid_argument("SocketCAN interface name is too long");
      }
      std::strncpy(request.ifr_name, config.device.c_str(), IFNAMSIZ - 1);
      if (ioctl(socket_, SIOCGIFINDEX, &request) < 0) {throw_system("SIOCGIFINDEX");}
      sockaddr_can address{};
      address.can_family = AF_CAN;
      address.can_ifindex = request.ifr_ifindex;
      if (bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        throw_system("bind(SocketCAN)");
      }
      const int flags = fcntl(socket_, F_GETFL, 0);
      if (flags < 0 || fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw_system("fcntl(O_NONBLOCK)");
      }
    } catch (...) {
      close();
      throw;
    }
  }

  void close() noexcept override
  {
    if (socket_ >= 0) {::close(socket_);}
    socket_ = -1;
  }

  void send(const Frame & frame) override
  {
    require_open();
    if (frame.fd) {
      canfd_frame raw{};
      raw.can_id = frame.id | (frame.extended ? CAN_EFF_FLAG : 0u);
      raw.len = static_cast<__u8>(frame.size);
      raw.flags = frame.bitrate_switch ? CANFD_BRS : 0;
      std::copy_n(frame.data.begin(), frame.size, raw.data);
      if (::write(socket_, &raw, CANFD_MTU) != CANFD_MTU) {throw_system("write(CAN FD)");}
    } else {
      if (frame.size > CAN_MAX_DLEN) {throw std::invalid_argument("classic CAN payload exceeds 8 bytes");}
      can_frame raw{};
      raw.can_id = frame.id | (frame.extended ? CAN_EFF_FLAG : 0u);
      raw.can_dlc = static_cast<__u8>(frame.size);
      std::copy_n(frame.data.begin(), frame.size, raw.data);
      if (::write(socket_, &raw, CAN_MTU) != CAN_MTU) {throw_system("write(CAN)");}
    }
  }

  bool receive(Frame & frame, int timeout_us) override
  {
    require_open();
    if (timeout_us > 0) {
      pollfd descriptor{socket_, POLLIN, 0};
      const int timeout_ms = std::max(1, (timeout_us + 999) / 1000);
      const int rc = ::poll(&descriptor, 1, timeout_ms);
      if (rc == 0) {return false;}
      if (rc < 0) {throw_system("poll(SocketCAN)");}
    }
    canfd_frame raw{};
    const ssize_t bytes = ::read(socket_, &raw, CANFD_MTU);
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {return false;}
    if (bytes != CAN_MTU && bytes != CANFD_MTU) {
      if (bytes < 0) {throw_system("read(SocketCAN)");}
      throw std::runtime_error("unexpected SocketCAN frame size");
    }
    frame = {};
    frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
    frame.id = raw.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    frame.fd = bytes == CANFD_MTU;
    frame.bitrate_switch = frame.fd && ((raw.flags & CANFD_BRS) != 0);
    frame.size = raw.len;
    std::copy_n(raw.data, frame.size, frame.data.begin());
    return true;
  }

  bool is_open() const noexcept override {return socket_ >= 0;}

private:
  [[noreturn]] static void throw_system(const char * operation)
  {
    throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
  }
  void require_open() const
  {
    if (socket_ < 0) {throw std::runtime_error("SocketCAN transport is not open");}
  }
  int socket_{-1};
};
#endif

}  // namespace

std::unique_ptr<Transport> make_transport(const std::string & backend)
{
  if (backend == "meow_usb") {return make_meow_usb_transport();}
#ifdef __linux__
  if (backend == "socketcan") {return std::make_unique<SocketCanTransport>();}
#endif
  throw std::invalid_argument("unsupported transport backend: " + backend);
}

}  // namespace motorevo_ros2

