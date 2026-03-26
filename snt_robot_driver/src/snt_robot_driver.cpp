#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <iomanip>
#include <array>
#include <fstream>
#include <pthread.h>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "test.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace snt_robot_driver
{

// Lock-free ring buffer for RT-safe data transfer
template <typename T, size_t Size>
class LockFreeBuffer 
{
public:
    LockFreeBuffer() : head_(0), tail_(0) {}
    
    bool push(const T & item) 
    {
        size_t next_head = (head_ + 1) % Size;
        if (next_head == tail_) return false; // full
        buffer_[head_] = item;
        head_ = next_head;
        return true;
    }
    
    bool pop(T & item) 
    {
        if (tail_ == head_) return false; // empty
        item = buffer_[tail_];
        tail_ = (tail_ + 1) % Size;
        return true;
    }
    
private:
    T buffer_[Size];
    std::atomic<size_t> head_, tail_;
};

static constexpr int MAX_SAMPLES = 2000;
static constexpr double MOVE_THRESHOLD = 1e-6; // 움직임 감지 임계값

class SNTRobotHW : public hardware_interface::SystemInterface
{
public:
    SNTRobotHW() = default;
    ~SNTRobotHW() override = default;

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        //--------------------- C 코드 -------------------- 
        int tnum = 0;
        tnum = test();
        if(tnum == 42)
        {
            RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "C를 통해서 작성된 함수가 실행됨");
        }
        //--------------------- C 코드 -------------------- 
        
        node_ = std::make_shared<rclcpp::Node>("snt_talker_node");
        publisher_ = node_->create_publisher<std_msgs::msg::String>("chatter", 10);
        rt_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(publisher_); 

        num_joints_ = info_.joints.size();
        position_.assign(num_joints_, 0.0);
        velocity_.assign(num_joints_, 0.0);
        command_position_.assign(num_joints_, 0.0);
        prev_command_position_.assign(num_joints_, 0.0);

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
        if (!active_) return hardware_interface::return_type::OK;

        const double RAD_TO_DEG = 180.0 / 3.14159265358979;

        // ------------------- RT-safe 메시지 생성 -------------------
        char msg_buf[128];
        int offset = 0;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            offset += snprintf(msg_buf + offset, sizeof(msg_buf) - offset,
                               "%.2f%s", command_position_[i] * RAD_TO_DEG,
                               (i < num_joints_ - 1 ? "/" : ""));
        }

        // RealtimePublisher 사용
        if (rt_publisher_->trylock())
        {
            rt_publisher_->msg_.data = std::string(msg_buf);
            rt_publisher_->unlockAndPublish();
        }

        // ------------------- MoveIt 명령 변화 감지 -------------------
        if (!trigger_started && !is_full)
        {
            for (size_t i = 0; i < num_joints_; ++i)
            {
                if (std::abs(command_position_[i] - prev_command_position_[i]) > MOVE_THRESHOLD)
                {
                    trigger_started = true;
                    sample_idx = 0;
                    clock_gettime(CLOCK_MONOTONIC, &last_time);
                    RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "MoveIt 실행 감지: 지터 기록 시작.");
                    break;
                }
            }
        }

        if (trigger_started && !is_full)
        {
            struct timespec curr_time;
            clock_gettime(CLOCK_MONOTONIC, &curr_time);
            
            long diff_nsec = (curr_time.tv_sec - last_time.tv_sec) * 1000000000L +
                             (curr_time.tv_nsec - last_time.tv_nsec);
            
            jitter_samples[sample_idx] = static_cast<double>(diff_nsec);
            sample_idx++;
            if (sample_idx >= MAX_SAMPLES)
            {
                is_full = true;
                trigger_started = false;
                RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "2000개 샘플 수집 완료");
            }
            last_time = curr_time;
        }

        prev_command_position_ = command_position_;
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
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "SNT로봇 on_activate 작동.");
        prev_command_position_ = command_position_;
        sample_idx = 0;
        is_full = false;
        trigger_started = false; 
        active_ = true;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        active_ = false;
        if (is_full) 
        {
            std::ofstream file("/tmp/snt_jitter_log.csv");
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
                RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "지터 리포트 저장 완료");
                RCLCPP_INFO(rclcpp::get_logger("SNT_HW"), "평균 주기: %.2f ns", sum / MAX_SAMPLES);
            }
        }
        return CallbackReturn::SUCCESS;
    }

private:
    double jitter_samples[MAX_SAMPLES]; 
    int sample_idx = 0;
    bool is_full = false;
    bool trigger_started = false;
    struct timespec last_time;

    size_t num_joints_{0};
    bool active_{false};
    std::vector<double> position_, velocity_, effort_;
    std::vector<double> command_position_;
    std::vector<double> prev_command_position_;

    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> rt_publisher_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

} // namespace snt_robot_driver

PLUGINLIB_EXPORT_CLASS(snt_robot_driver::SNTRobotHW, hardware_interface::SystemInterface)