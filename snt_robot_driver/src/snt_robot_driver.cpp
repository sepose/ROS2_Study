#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <array>
#include <fstream>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp" 
#include "realtime_tools/realtime_publisher.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace snt_robot_driver
{

    static constexpr int MAX_SAMPLES = 1000;

    class SNTRobotHW : public hardware_interface::SystemInterface
    {
    public:

        SNTRobotHW() = default;
        ~SNTRobotHW() override = default;

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        node_ = std::make_shared<rclcpp::Node>("snt_talker_node");
        publisher_ = node_->create_publisher<std_msgs::msg::String>("chatter", 10);
        rt_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(publisher_); 

        num_joints_ = info_.joints.size();

        position_.assign(num_joints_, 0.0);
        velocity_.assign(num_joints_, 0.0);
        command_position_.assign(num_joints_, 0.0);

        RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "On Init 완료.");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
        }
        return command_interfaces;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
    {
        if (!active_)
        {
            return hardware_interface::return_type::OK;
        }

        if (!is_full) 
        {
            struct timespec curr_time;
            clock_gettime(CLOCK_MONOTONIC, &curr_time);
            
            // 초 단위를 고려한 정밀한 나노초 차이 계산 (지터 측정의 핵심)
            long diff_nsec = (curr_time.tv_sec - last_time.tv_sec) * 1000000000L + 
                             (curr_time.tv_nsec - last_time.tv_nsec);
            
            jitter_samples[sample_idx] = static_cast<double>(diff_nsec);
            sample_idx++;

            if (sample_idx >= MAX_SAMPLES) {
                is_full = true;
            }
            last_time = curr_time;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
       
        for (size_t i = 0; i < num_joints_; ++i)
        {
            position_[i] = command_position_[i];
        }
        
        return hardware_interface::return_type::OK;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_configure 작동.");

        struct sched_param param;
        param.sched_priority = 90;
        RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "Priority 90으로 상승");
        pthread_setschedparam(pthread_self(),SCHED_FIFO,&param);
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_activate 작동.");
        
        clock_gettime(CLOCK_MONOTONIC, &last_time);
        sample_idx = 0;
        is_full = false;
        
        active_ = true;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        active_ = false;
        if (is_full) 
        {
            std::string filepath = "/tmp/snt_jitter_log.csv";
            std::ofstream file(filepath);

            if (file.is_open()) 
            {
                file << "sample_index,jitter_ns\n";
                double sum = 0;
                for (int i = 0; i < MAX_SAMPLES; ++i) 
                {
                    file << i << "," << std::fixed << std::setprecision(0) << jitter_samples[i] << "\n";
                    sum += jitter_samples[i];
                }
                file.close();
                RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "지터 리포트 저장 완료: %s", filepath.c_str());
                RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "평균 주기: %.2f ns", sum / MAX_SAMPLES);
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_deactivate 작동.");
            return CallbackReturn::SUCCESS;
    }

    private:

        double jitter_samples[MAX_SAMPLES]; 
        int sample_idx = 0;
        bool is_full = false;
        struct timespec last_time;

        size_t num_joints_{0};
        bool active_{false};
        std::vector<double> position_, velocity_, effort_;
        std::vector<double> command_position_;
    
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> rt_publisher_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    };

} // namespace snt_robot_driver

PLUGINLIB_EXPORT_CLASS(snt_robot_driver::SNTRobotHW, hardware_interface::SystemInterface)