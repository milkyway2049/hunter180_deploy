#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "ec_joint_hardware/utils.hpp"

namespace encos::ec_joint_hardware {
namespace {

// RAII
// 临时配置文件：写入失败立即抛异常（避免读到上次运行残留的文件），析构时清理
class ScopedConfigFile {
 public:
  explicit ScopedConfigFile(const nlohmann::json& config)
      : path_(std::filesystem::temp_directory_path() / "ec_joint_hardware_config_test.json") {
    std::ofstream output(path_);
    if (!(output << config.dump(2))) {
      throw std::runtime_error("failed to write test config file");
    }
  }

  ~ScopedConfigFile() { std::filesystem::remove(path_); }

  std::string path() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

nlohmann::json MinimalConfig(nlohmann::json joint = {
                                 {"name", "test_joint"},
                                 {"type", "RJoint"},
                                 {"config", nlohmann::json::object()},
                             }) {
  return {
      {"adapters", {{{"type", "Fake"}, {"name", "test_adapter"}}}},
      {"joints", {std::move(joint)}},
  };
}

}  // namespace

TEST(JointHardwareConfigTest, ParsesNumberedAdapterSelections) {
  const std::unordered_map<std::string, std::string> parameters = {
      {"adapter_type_1", "Ethercat"}, {"interface_name_1", "enp86s0"},    {"adapter_type_2", "Fake"},
      {"interface_name_2", "eth0"},   {"unrelated_parameter", "ignored"},
  };

  const auto selections = ParseAdapterSelections(parameters);

  ASSERT_EQ(selections.size(), 2U);
  EXPECT_EQ(selections.at(1).type, "Ethercat");
  EXPECT_EQ(selections.at(1).interface_name, "enp86s0");
  EXPECT_EQ(selections.at(2).type, "Fake");
  EXPECT_EQ(selections.at(2).interface_name, "eth0");
}

TEST(JointHardwareConfigTest, RejectsInvalidNumberedAdapterSelections) {
  for (const auto& parameters : {
           std::unordered_map<std::string, std::string>{},
           std::unordered_map<std::string, std::string>{{"adapter_type_1", "Fake"}},
           std::unordered_map<std::string, std::string>{{"interface_name_1", "test_adapter"}},
           std::unordered_map<std::string, std::string>{{"adapter_type_1", ""}, {"interface_name_1", "test_adapter"}},
           std::unordered_map<std::string, std::string>{{"adapter_type_invalid", "Fake"},
                                                        {"interface_name_invalid", "test_adapter"}},
       }) {
    EXPECT_THROW((void)ParseAdapterSelections(parameters), std::runtime_error);
  }
}

TEST(JointHardwareConfigTest, ParsesAdapterDtoAndPreservesRawSdkJointJson) {
  const nlohmann::json joint = {
      {"name", "future_sdk_joint"},
      {"type", "FutureSdkJoint"},
      {"config", {{"opaque_sdk_field", 42}}},
  };
  const ScopedConfigFile config_file(MinimalConfig(joint));
  const auto config = LoadJointHardwareConfig(config_file.path(), {});

  ASSERT_EQ(config.adapters.size(), 1U);
  EXPECT_EQ(config.adapters.front().type, "Fake");
  EXPECT_EQ(config.adapters.front().name, "test_adapter");
  ASSERT_EQ(config.joints.size(), 1U);
  EXPECT_EQ(config.joints.front(), joint);
}

TEST(JointHardwareConfigTest, DoesNotDuplicateSdkJointValidation) {
  const nlohmann::json joint = {
      {"name", "sdk_owned_joint"},
      {"type", "RJoint"},
      {"config", {{"maxSpd", 0}, {"maxTor", -1}, {"sdk_extension", true}}},
  };

  const ScopedConfigFile config_file(MinimalConfig(joint));
  EXPECT_NO_THROW((void)LoadJointHardwareConfig(config_file.path(), {}));
}

TEST(JointHardwareConfigTest, RequiresRootAdapterAndJointArrays) {
  {
    auto config = MinimalConfig();
    config.erase("adapters");
    const ScopedConfigFile config_file(config);
    EXPECT_THROW((void)LoadJointHardwareConfig(config_file.path(), {}), std::runtime_error);
  }
  {
    auto config = MinimalConfig();
    config["joints"] = nlohmann::json::object();
    const ScopedConfigFile config_file(config);
    EXPECT_THROW((void)LoadJointHardwareConfig(config_file.path(), {}), std::runtime_error);
  }
}

TEST(JointHardwareConfigTest, AdapterDtoUsesRequiredNlohmannFields) {
  auto config = MinimalConfig();
  config["adapters"][0].erase("type");
  const ScopedConfigFile config_file(config);
  EXPECT_THROW((void)LoadJointHardwareConfig(config_file.path(), {}), std::runtime_error);
}

TEST(JointHardwareConfigTest, ResolvesNumberedAdapterMarkersAndPreservesNumericFields) {
  const nlohmann::json joint = {
      {"name", "test_joint"},
      {"type", "RJoint"},
      {"config",
       {{"motorParams",
         {{"adapterId", "enx-replace-me-2"},
          {"adapterType", "plugin-replace-me-2"},
          {"busId", 65536},
          {"motorId", 3},
          {"nested", nlohmann::json{{"adapterId", "enx-replace-me-1"}}}}},
        {"description", "prefix plugin-replace-me-2 suffix"},
        {"leading_marker", "plugin-replace-me-2 suffix"}}},
  };
  auto config = MinimalConfig(joint);
  config["adapters"] = {
      {{"type", "plugin-replace-me-1"}, {"name", "enx-replace-me-1"}},
      {{"type", "plugin-replace-me-2"}, {"name", "enx-replace-me-2"}},
  };
  const ScopedConfigFile config_file(config);
  const AdapterSelections selections = {
      {1, {"Ethercat", "enp86s0"}},
      {2, {"Fake", "eth0"}},
  };

  const auto loaded = LoadJointHardwareConfig(config_file.path(), selections);

  ASSERT_EQ(loaded.adapters.size(), 2U);
  EXPECT_EQ(loaded.adapters[0].type, "Ethercat");
  EXPECT_EQ(loaded.adapters[0].name, "enp86s0");
  EXPECT_EQ(loaded.adapters[1].type, "Fake");
  EXPECT_EQ(loaded.adapters[1].name, "eth0");
  const auto& motor_params = loaded.joints.front().at("config").at("motorParams");
  EXPECT_EQ(motor_params.at("adapterId"), "eth0");
  EXPECT_EQ(motor_params.at("adapterType"), "Fake");
  EXPECT_EQ(motor_params.at("busId"), 65536);
  EXPECT_EQ(motor_params.at("motorId"), 3);
  EXPECT_EQ(motor_params.at("nested").at("adapterId"), "enp86s0");
  EXPECT_EQ(loaded.joints.front().at("config").at("description"), "prefix plugin-replace-me-2 suffix");
  EXPECT_EQ(loaded.joints.front().at("config").at("leading_marker"), "plugin-replace-me-2 suffix");
}

TEST(JointHardwareConfigTest, RejectsMarkersReferencingUnknownAdapterIds) {
  auto config = MinimalConfig();
  config["adapters"][0]["type"] = "plugin-replace-me-2";
  config["adapters"][0]["name"] = "enx-replace-me-2";
  const ScopedConfigFile config_file(config);

  EXPECT_THROW((void)LoadJointHardwareConfig(config_file.path(), {{1, {"Fake", "test_adapter"}}}), std::runtime_error);
}

}  // namespace encos::ec_joint_hardware
