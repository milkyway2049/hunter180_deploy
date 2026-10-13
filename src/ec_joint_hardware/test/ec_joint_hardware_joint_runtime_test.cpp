#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "ec_joint_hardware/joint_runtime.hpp"
#include "ec_joint_hardware/types.hpp"
#include "ec_joint_hardware/utils.hpp"

namespace encos::ec_joint_hardware {
namespace {

TEST(JointRuntimeTest, SameInterfacesIgnoresOrderAndRejectsDuplicates) {
  EXPECT_TRUE(SameInterfaces({"velocity", "position", "current"}, {"position", "velocity", "current"}));
  EXPECT_FALSE(SameInterfaces({"current", "current"}, {"velocity", "current"}));
  EXPECT_FALSE(SameInterfaces({"current"}, {"effort"}));
}

TEST(JointRuntimeTest, ResolvesEveryDefinedModeFromItsCompleteInterfaceSet) {
  for (const auto& [expected_mode, interfaces] : kModeDefinitions) {
    auto claimed = interfaces;
    std::reverse(claimed.begin(), claimed.end());

    const auto resolved = JointRuntime::ResolveMode(claimed);

    ASSERT_TRUE(resolved.has_value());
    EXPECT_EQ(*resolved, expected_mode);
  }
}

TEST(JointRuntimeTest, RejectsAnUnknownOrDuplicateInterfaceSet) {
  EXPECT_EQ(JointRuntime::ResolveMode({"kp", "current"}), std::nullopt);
  EXPECT_EQ(JointRuntime::ResolveMode({"current", "current"}), std::nullopt);
}

TEST(JointRuntimeTest, AppliesOnlyThisJointsAddedAndRemovedInterfaceKeys) {
  JointRuntime runtime("left_joint");
  runtime.SetMode(ModeDefinition::Position);

  const auto resolved =
      runtime.ResolveMode({"right_joint/current", "left_joint/effort", "left_joint/effort"},
                          {"right_joint/effort", "left_joint/position", "left_joint/velocity", "left_joint/current"});

  ASSERT_TRUE(resolved.has_value());
  EXPECT_EQ(*resolved, ModeDefinition::Torque);
}

TEST(JointRuntimeTest, DistinguishesAReleasedClaimFromAnInvalidClaim) {
  JointRuntime runtime("test_joint");
  runtime.SetMode(ModeDefinition::Torque);

  const auto released = runtime.ResolveMode({}, {"test_joint/effort"});
  ASSERT_TRUE(released.has_value());
  EXPECT_EQ(*released, ModeDefinition::None);

  const auto invalid = runtime.ResolveMode({"test_joint/current"}, {});
  EXPECT_EQ(invalid, std::nullopt);
}

}  // namespace
}  // namespace encos::ec_joint_hardware
