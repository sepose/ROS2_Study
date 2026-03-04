#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"

#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace snt_robot_driver
{

class SNTRobotHW : public hardware_interface::SystemInterface
{
public:
  SNTRobotHW() = default;
  ~SNTRobotHW() override = default;

  // ---------------- on_init ----------------
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_init 작동 - 초기화 시작.");

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_init 실패 - HardwareInfo 로드 실패.");
      return CallbackReturn::ERROR;
    }

    num_joints_ = info_.joints.size();

    //
    position_.assign(num_joints_, 0.0);
    velocity_.assign(num_joints_, 0.0);
    effort_.assign(num_joints_, 0.0);
    command_position_.assign(num_joints_, 0.0);
    command_velocity_.assign(num_joints_, 0.0);
    command_effort_.assign(num_joints_, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), 
                "SNT로봇 on_init 작동 - 총 %zu개의 조인트가 초기화되었습니다.", num_joints_);
    return CallbackReturn::SUCCESS;
  }

  // ---------------- on_configure ----------------
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_configure 작동 - 하드웨어가 설정되었습니다.");
    return CallbackReturn::SUCCESS;
  }

  // ---------------- on_activate ----------------
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_activate 작동 - 제어가 시작됩니다.");
    active_ = true;
    return CallbackReturn::SUCCESS;
  }

  // ---------------- on_deactivate ----------------
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_deactivate 작동 - 제어가 중지되었습니다.");
    active_ = false;
    return CallbackReturn::SUCCESS;
  }

  // ---------------- export_state_interfaces ----------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 export_state_interfaces 작동 - 상태 인터페이스를 등록합니다.");

    // 즉, JSB가  어떤  값을  읽어가면  될지  정의해준다. 

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
      RCLCPP_INFO(rclcpp::get_logger("SNT_HW"),
                  "  → 조인트[%zu] %s: position, velocity 상태 인터페이스 등록 완료.",
                  i + 1, info_.joints[i].name.c_str());
    }
    return state_interfaces;
  }

  // ---------------- export_command_interfaces ----------------
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 export_command_interfaces 작동 - 명령 인터페이스를 등록합니다.");

    // 즉, JTC가 어디다가  명령 값을  쓰면  되는지  정의해준다.

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
      RCLCPP_INFO(rclcpp::get_logger("SNT_HW"),
                  "  → 조인트[%zu] %s: position 명령 인터페이스 등록 완료.",
                  i + 1, info_.joints[i].name.c_str());
    }
    return command_interfaces;
  }

  // ---------------- read ----------------
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) return hardware_interface::return_type::OK;
    for (size_t i = 0; i < num_joints_; ++i)
    {
      position_[i] = command_position_[i];
    }
    return hardware_interface::return_type::OK;
  }

// ---------------- write ----------------
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (!active_) return hardware_interface::return_type::OK;

    const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
    std::ostringstream ss;

    // 어딘가에  쓴다고  가정하고 그냥  값 print
    // rclcpp::get_logger 가  print역할을  하는 놈이다

    ss << "Joint Degrees: ";
    for (size_t i = 0; i < num_joints_; ++i)
    {
      double pos_deg = command_position_[i] * RAD_TO_DEG;
      
      // 조인트 이름과 도(°) 값만 딱 나오게 설정
      ss << "[" << info_.joints[i].name << "]: " 
         << std::fixed << std::setprecision(2) << pos_deg << "°  ";
    }

    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "%s", ss.str().c_str());

    return hardware_interface::return_type::OK;
  }

private:

  size_t num_joints_{0};
  bool active_{false};
  std::vector<double> position_, velocity_, effort_;
  std::vector<double> command_position_, command_velocity_, command_effort_;
};

}

PLUGINLIB_EXPORT_CLASS
(
  snt_robot_driver::SNTRobotHW, 
  hardware_interface::SystemInterface
)