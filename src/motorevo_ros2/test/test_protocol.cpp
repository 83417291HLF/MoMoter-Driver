#include "motorevo_ros2/protocol.hpp"

#include <gtest/gtest.h>

#include <vector>

using motorevo_ros2::MotorCommand;
using motorevo_ros2::Protocol;
using motorevo_ros2::StateCommand;

TEST(Protocol, PacksDocumentedNeutralMitCommand)
{
  Protocol protocol;
  const auto bytes = protocol.pack_mit(MotorCommand{});
  const std::array<uint8_t, 8> expected{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  EXPECT_EQ(bytes, expected);
}

TEST(Protocol, PlacesCommandsByMotorIdInBroadcastFrame)
{
  Protocol protocol;
  MotorCommand command;
  command.id = 2;
  command.position = 12.5;
  const auto frame = protocol.make_broadcast_control({command});
  EXPECT_EQ(frame.id, 0x20u);
  EXPECT_EQ(frame.size, 64u);
  EXPECT_TRUE(frame.fd);
  EXPECT_EQ(frame.data[8], 0xFF);
  EXPECT_EQ(frame.data[9], 0xFF);
}

TEST(Protocol, BroadcastStateOnlyTargetsSelectedMotors)
{
  Protocol protocol;
  const auto frame = protocol.make_broadcast_state({2, 7}, StateCommand::kEnable);
  EXPECT_EQ(frame.id, 0x10u);
  EXPECT_EQ(frame.data[7], static_cast<uint8_t>(StateCommand::kQuery));
  EXPECT_EQ(frame.data[15], static_cast<uint8_t>(StateCommand::kEnable));
  EXPECT_EQ(frame.data[55], static_cast<uint8_t>(StateCommand::kEnable));
}

TEST(Protocol, DecodesFeedbackAndStatus)
{
  Protocol protocol;
  motorevo_ros2::Frame frame;
  frame.id = 1;
  frame.size = 8;
  frame.data = {0x80, 0x00, 0x7F, 0xF7, 0xFF, 65, 0x00, 0x03};
  const auto state = protocol.unpack_feedback(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->id, 1);
  EXPECT_NEAR(state->position, 0.0, 0.001);
  EXPECT_NEAR(state->velocity, 0.0, 0.01);
  EXPECT_NEAR(state->effort, 0.0, 0.05);
  EXPECT_DOUBLE_EQ(state->temperature, 25.0);
  EXPECT_TRUE(state->enabled);
  EXPECT_EQ(state->status, 0x0003u);
}

TEST(Protocol, RejectsInvalidFeedback)
{
  Protocol protocol;
  motorevo_ros2::Frame frame;
  frame.id = 1;
  frame.size = 7;
  EXPECT_FALSE(protocol.unpack_feedback(frame).has_value());
}
