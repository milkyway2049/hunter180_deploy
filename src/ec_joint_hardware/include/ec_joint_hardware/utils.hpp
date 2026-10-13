/**
 * @file utils.hpp
 * @brief 声明 ec_joint_hardware 的配置、接口集合和日志辅助函数。
 */

#pragma once

#include <nlohmann/json_fwd.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ec_joint_hardware/types.hpp"
#include "motor/types.h"

namespace encos::ec_joint_hardware {

/**
 * @brief 将 `joint/interface` 键拆分为关节名和接口名。
 * @param[in] key ros2_control 接口键。
 * @return 指向 @p key 存储区的关节名和接口名视图；没有分隔符时接口名为空。
 */
std::pair<std::string_view, std::string_view> SplitInterfaceKey(const std::string& key);

/**
 * @brief 判断两个接口名称集合是否完全相同。
 * @param[in] lhs 第一个接口集合。
 * @param[in] rhs 第二个接口集合。
 * @return 两集合元素及重复次数相同则为 true；元素顺序不影响结果。
 */
bool SameInterfaces(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs);

/**
 * @brief 获取控制模式的稳定日志名称。
 * @param[in] mode 控制模式。
 * @return 以空字符结尾的静态字符串；未知枚举值返回 `unknown`。
 */
const char* ModeName(ModeDefinition mode);

/**
 * @brief 获取电机错误码的稳定日志名称。
 * @param[in] error joint_sdk 电机错误码。
 * @return 以空字符结尾的静态字符串；未知错误码返回 `unknown`。
 */
const char* MotorErrorName(encos::MotorError error);

/**
 * @brief 从 JSON 对象解析 adapter 配置。
 * @param[in] json 包含 `type` 和 `name` 字段的 JSON 对象。
 * @param[out] adapter 接收解析结果的 adapter 配置。
 * @throws nlohmann::json::exception 字段缺失或类型错误时抛出。
 */
void from_json(const nlohmann::json& json, AdapterConfig& adapter);

/**
 * @brief 从 hardware 参数解析编号 adapter 选择表。
 * @param[in] hardware_parameters ros2_control hardware 参数集合。
 * @return 以编号为键的 adapter 类型和接口名。
 * @throws std::runtime_error 没有完整 pair、编号非法或值为空时抛出。
 */
AdapterSelections ParseAdapterSelections(const std::unordered_map<std::string, std::string>& hardware_parameters);

/**
 * @brief 从文件加载硬件插件配置。
 * @param[in] config_file JSON 配置文件路径。
 * @param[in] adapter_selections 由 hardware 参数选择的 adapter 表。
 * @return 解析完成的 adapter 和关节配置。
 * @throws std::runtime_error 文件不可读、根类型错误或 JSON 内容无效时抛出。
 */
JointHardwareConfig LoadJointHardwareConfig(const std::string& config_file,
                                            const AdapterSelections& adapter_selections);

}  // namespace encos::ec_joint_hardware
