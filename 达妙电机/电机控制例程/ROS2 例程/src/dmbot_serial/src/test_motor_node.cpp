#include "dmbot_serial/dm_ros2_utils.hpp"
#include "dmbot_serial/protocol/damiao.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


namespace
{

// 不同控制模式下，单个电机需要的指令数值个数
constexpr std::size_t kMitCommandSize = 5;
constexpr std::size_t kPosVelCommandSize = 2;
constexpr std::size_t kVelCommandSize = 1;
constexpr std::size_t kPosForceCommandSize = 3;

// 根据控制模式返回单个电机需要的指令长度
std::size_t command_size_for_mode(damiao::Control_Mode mode)
{
  switch (mode) {
    case damiao::MIT_MODE:
      return kMitCommandSize;
    case damiao::POS_VEL_MODE:
      return kPosVelCommandSize;
    case damiao::VEL_MODE:
      return kVelCommandSize;
    case damiao::POS_FORCE_MODE:
      return kPosForceCommandSize;
    default:
      return 0;
  }
}

// 根据控制模式返回便于打印的指令格式说明
std::string command_format_for_mode(damiao::Control_Mode mode)
{
  switch (mode) {
    case damiao::MIT_MODE:
      return "[kp, kd, position, velocity, effort]";
    case damiao::POS_VEL_MODE:
      return "[position, velocity]";
    case damiao::VEL_MODE:
      return "[velocity]";
    case damiao::POS_FORCE_MODE:
      return "[position, velocity, effort]";
    default:
      return "未知模式";
  }
}

}  // namespace

class TestMotorNode : public rclcpp::Node
{
public:
  TestMotorNode()
  : Node("test_motor_node")
  {
    // 读取 CAN 接口和控制模式等基本参数
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    can_mode_ = dmbot_serial::parse_can_mode(declare_parameter<std::string>("can_mode", "canfd"));
    control_mode_ = dmbot_serial::parse_control_mode(declare_parameter<std::string>("control_mode", "pos_vel"));

    // 读取多电机数组参数
    const auto motor_types =
      declare_parameter<std::vector<std::string>>("motor_types", std::vector<std::string>{"DM4310"});
    const auto motor_can_ids =
      declare_parameter<std::vector<int64_t>>("motor_can_ids", std::vector<int64_t>{0x01});
    const auto motor_master_ids =
      declare_parameter<std::vector<int64_t>>("motor_master_ids", std::vector<int64_t>{0x11});

    // 三个数组必须等长，否则无法正确建立电机与 CAN ID 的对应关系
    if (motor_types.size() != motor_can_ids.size() ||
      motor_types.size() != motor_master_ids.size()) {
      throw std::invalid_argument(
        "motor_types、motor_can_ids、motor_master_ids 三个数组长度必须一致");
    }

    const auto motor_count = motor_can_ids.size();
    if (motor_count == 0) {
      throw std::invalid_argument("motor_can_ids 不能为空");
    }

    // 初始化每个电机的目标值和 MIT 参数
    const auto initial_position = declare_parameter<double>("initial_position", 0.0);
    const auto initial_velocity = declare_parameter<double>("initial_velocity", 0.0);
    const auto initial_effort = declare_parameter<double>("initial_effort", 0.0);
    const auto initial_kp = declare_parameter<double>("mit_kp", 0.0);
    const auto initial_kd = declare_parameter<double>("mit_kd", 0.0);
    max_send_rate_hz_ = declare_parameter<double>("send_rate_hz", 1000.0);

    motor_can_ids_.reserve(motor_count);
    motor_master_ids_.reserve(motor_count);
    motor_data_.reserve(motor_count);
    target_position_.assign(motor_count, initial_position);
    target_velocity_.assign(motor_count, initial_velocity);
    target_effort_.assign(motor_count, initial_effort);
    mit_kp_.assign(motor_count, initial_kp);
    mit_kd_.assign(motor_count, initial_kd);

    // 组装电机注册信息
    for (std::size_t i = 0; i < motor_count; ++i) {
      const auto motor_type = dmbot_serial::parse_motor_type(motor_types[i]);
      const auto can_id = static_cast<uint16_t>(motor_can_ids[i]);
      const auto master_id = static_cast<uint16_t>(motor_master_ids[i]);

      motor_can_ids_.push_back(can_id);
      motor_master_ids_.push_back(master_id);
      motor_data_.push_back(damiao::DmActData{
        .motorType = motor_type,
        .mode = control_mode_,
        .can_id = can_id,
        .mst_id = master_id,
      });
    }

    // 创建 CAN 电机控制对象，并注册本节点控制的所有电机
    motor_control_ = std::make_unique<damiao::Motor_Control>(can_interface_, &motor_data_, can_mode_);

    // 订阅目标指令话题，消息到达后先解析再保存
    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/target_command", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        update_command(*msg);
      });

    // 按 send_rate_hz 周期性向电机下发最新指令
    const auto send_period = std::chrono::duration<double>(1.0 / std::max(max_send_rate_hz_, 1.0));
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(send_period),
      [this]() {
        send_command();
      });

    // 每秒打印一次电机反馈信息
    feedback_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        print_feedback();
      });

    RCLCPP_INFO(
      get_logger(),
      "test_motor_node 已启动: motor_count=%zu, can_interface=%s, "
      "当前模式格式 %s，单个电机指令长度=%zu",
      motor_can_ids_.size(),
      can_interface_.c_str(),
      command_format_for_mode(control_mode_).c_str(),
      command_size_for_mode(control_mode_));
  }

  ~TestMotorNode() override
  {
    motor_control_.reset();
  }

private:
  // 根据电机下标获取对应的 Motor 对象
  std::shared_ptr<damiao::Motor> motor(std::size_t index)
  {
    if (!motor_control_ || index >= motor_can_ids_.size()) {
      return nullptr;
    }
    return motor_control_->getMotor(motor_can_ids_[index]);
  }

  // 解析并保存一次目标指令
  void update_command(const std_msgs::msg::Float64MultiArray& msg)
  {
    const auto motor_count = motor_can_ids_.size();
    const auto per_motor_size = command_size_for_mode(control_mode_);
    if (per_motor_size == 0) {
      RCLCPP_WARN(get_logger(), "不支持的控制模式，无法解析 target_command");
      return;
    }

    const auto expected_size = motor_count * per_motor_size;
    // 指令数组长度必须和“电机数量 x 单电机参数数量”完全一致
    if (msg.data.size() != expected_size) {
      RCLCPP_WARN(
        get_logger(),
        "target_command 需要 %zu 个数值（%zu 个电机 x %s），当前收到 %zu 个",
        expected_size,
        motor_count,
        command_format_for_mode(control_mode_).c_str(),
        msg.data.size());
      return;
    }

    // 按电机顺序从扁平数组中依次取出各自的目标值
    for (std::size_t i = 0; i < motor_count; ++i) {
      const auto offset = i * per_motor_size;
      switch (control_mode_) {
        case damiao::MIT_MODE:
          mit_kp_[i] = msg.data[offset + 0];
          mit_kd_[i] = msg.data[offset + 1];
          target_position_[i] = msg.data[offset + 2];
          target_velocity_[i] = msg.data[offset + 3];
          target_effort_[i] = msg.data[offset + 4];
          break;
        case damiao::POS_VEL_MODE:
          target_position_[i] = msg.data[offset + 0];
          target_velocity_[i] = msg.data[offset + 1];
          break;
        case damiao::VEL_MODE:
          target_velocity_[i] = msg.data[offset + 0];
          break;
        case damiao::POS_FORCE_MODE:
          target_position_[i] = msg.data[offset + 0];
          target_velocity_[i] = msg.data[offset + 1];
          target_effort_[i] = msg.data[offset + 2];
          break;
        default:
          RCLCPP_WARN(get_logger(), "不支持的控制模式");
          break;
      }
    }
  }

  // 周期向所有电机发送控制指令
  void send_command()
  {
    for (std::size_t i = 0; i < motor_can_ids_.size(); ++i) {
      auto motor_ptr = motor(i);
      if (!motor_ptr) {
        RCLCPP_WARN(
          get_logger(),
          "未找到第 %zu 个电机 can_id=0x%03X",
          i,
          static_cast<unsigned>(motor_can_ids_[i]));
        continue;
      }

      switch (control_mode_) {
        case damiao::MIT_MODE:
          motor_control_->control_mit(
            *motor_ptr,
            static_cast<float>(mit_kp_[i]),
            static_cast<float>(mit_kd_[i]),
            static_cast<float>(target_position_[i]),
            static_cast<float>(target_velocity_[i]),
            static_cast<float>(target_effort_[i]));
          break;
        case damiao::POS_VEL_MODE:
          motor_control_->control_pos_vel(
            *motor_ptr,
            static_cast<float>(target_position_[i]),
            static_cast<float>(target_velocity_[i]));
          break;
        case damiao::VEL_MODE:
          motor_control_->control_vel(*motor_ptr, static_cast<float>(target_velocity_[i]));
          break;
        case damiao::POS_FORCE_MODE:
          motor_control_->control_pos_force(
            *motor_ptr,
            static_cast<float>(target_position_[i]),
            static_cast<float>(target_velocity_[i]),
            static_cast<float>(target_effort_[i]));
          break;
        default:
          RCLCPP_WARN(get_logger(), "不支持的控制模式");
          break;
      }
    }
  }

  // 周期打印所有电机的反馈和目标指令
  void print_feedback()
  {
    for (std::size_t i = 0; i < motor_can_ids_.size(); ++i) {
      auto motor_ptr = motor(i);
      if (!motor_ptr) {
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "反馈[%zu] can_id=0x%03X pos=%.4f vel=%.4f effort=%.4f err=%d | "
        "指令 pos=%.4f vel=%.4f effort=%.4f kp=%.4f kd=%.4f",
        i,
        static_cast<unsigned>(motor_can_ids_[i]),
        motor_ptr->Get_Position(),
        motor_ptr->Get_Velocity(),
        motor_ptr->Get_tau(),
        motor_ptr->Get_err(),
        target_position_[i],
        target_velocity_[i],
        target_effort_[i],
        mit_kp_[i],
        mit_kd_[i]);
    }
  }

  // CAN 通信配置
  std::string can_interface_;
  damiao::Can_control_Mode can_mode_{damiao::canfd};
  damiao::Control_Mode control_mode_{damiao::POS_VEL_MODE};

  // 电机 ID 列表
  std::vector<uint16_t> motor_can_ids_;
  std::vector<uint16_t> motor_master_ids_;

  // 每个电机当前的目标指令
  std::vector<double> target_position_;
  std::vector<double> target_velocity_;
  std::vector<double> target_effort_;
  std::vector<double> mit_kp_;
  std::vector<double> mit_kd_;
  double max_send_rate_hz_{};

  // 控制对象及相关数据
  std::vector<damiao::DmActData> motor_data_;
  std::unique_ptr<damiao::Motor_Control> motor_control_;

  // ROS2 通信对象
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<TestMotorNode>());
  } catch (const std::exception& e) {
    std::cerr << "test_motor_node 启动失败: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
