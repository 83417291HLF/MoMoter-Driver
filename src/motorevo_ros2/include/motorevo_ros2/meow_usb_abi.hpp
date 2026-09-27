#pragma once

#include <cstdint>

namespace motorevo_ros2
{

#pragma pack(push, 1)
struct MeowFrameInfo
{
  uint32_t can_id;
  uint8_t frame_type;
  uint8_t data_length;
};
#pragma pack(pop)

enum : uint8_t
{
  kMeowStandard = 0,
  kMeowExtended = 1,
  kMeowClassicCan = 0,
  kMeowFdCan = 1,
  kMeowFdCanBrs = 2,
};

}  // namespace motorevo_ros2
