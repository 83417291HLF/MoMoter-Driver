#pragma once

#include "motorevo_ros2/types.hpp"

#include <memory>
#include <string>

namespace motorevo_ros2
{

class Transport
{
public:
  virtual ~Transport() = default;
  virtual void open(const TransportConfig & config) = 0;
  virtual void close() noexcept = 0;
  virtual void send(const Frame & frame) = 0;
  virtual bool receive(Frame & frame, int timeout_us) = 0;
  virtual bool is_open() const noexcept = 0;
};

std::unique_ptr<Transport> make_transport(const std::string & backend);

}  // namespace motorevo_ros2

