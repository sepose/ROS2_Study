#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class FjtMoveItConnector : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;

  explicit FjtMoveItConnector() : Node("fjt_moveit_connector")
  {
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    current_state_.position.assign(6, 0.0); // 초기 위치 0으로 설정

    // 1. MoveIt에 현재 로봇 위치를 알려주는 퍼블리셔 (필수!)
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // 2. MoveIt의 명령을 받는 액션 서버
    server_ = rclcpp_action::create_server<FollowJT>(
      this, "/snt_arm_controller/follow_joint_trajectory",
      std::bind(&FjtMoveItConnector::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FjtMoveItConnector::handle_cancel, this, std::placeholders::_1),
      std::bind(&FjtMoveItConnector::handle_accepted, this, std::placeholders::_1)
    );

    // 30ms 간격으로 JointState 발행 (MoveIt이 로봇 위치를 놓치지 않게 함)
    timer_ = this->create_wall_timer(30ms, [this]() 
    {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = this->now();
      msg.name = joint_names_;
      msg.position = current_state_.position;
      js_pub_->publish(msg);
    });

    std::cout << "\033[1;32m[준비] MoveIt 연결 및 데이터 전수조사기 가동.\033[0m" << std::endl;
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() 
    {
      const auto goal = goal_handle->get_goal();
      const auto & traj = goal->trajectory;

      // [핵심] 수신된 모든 리스트 데이터 구조 출력
      std::cout << "\n\033[1;33m[데이터 수신] 계층 구조 분석 시작\033[0m" << std::endl;
      
      for (size_t i = 0; i < traj.points.size(); ++i) 
      {
        const auto & p = traj.points[i];
        std::cout << "\n\033[1;36mPoint [" << i << "]\033[0m" << std::endl;
        
        // P, V, A 데이터 전수 출력
        std::cout << "  ├─ Positions(리스트): ";
        for(auto val : p.positions) std::cout << val << " ";
        std::cout << "\n  ├─ Velocities(리스트): ";
        for(auto val : p.velocities) std::cout << val << " ";
        std::cout << "\n  └─ Accelerations(리스트): ";
        for(auto val : p.accelerations) std::cout << val << " ";
        std::cout << std::endl;

        // 실제 로봇 위치 업데이트 (시뮬레이션)
        {
          std::lock_guard<std::mutex> lk(mtx_);
          current_state_.position = p.positions;
        }
        std::this_thread::sleep_for(50ms); // 시각적인 확인을 위한 딜레이
      }

      auto res = std::make_shared<FollowJT::Result>();
      res->error_code = FollowJT::Result::SUCCESSFUL;
      goal_handle->succeed(res);
      std::cout << "\n\033[1;32m[완료] 모든 궤적 데이터 출력 및 실행 완료\033[0m" << std::endl;
    }).detach();
  }

  std::vector<std::string> joint_names_;
  sensor_msgs::msg::JointState current_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<FollowJT>::SharedPtr server_;
  std::mutex mtx_;
};

int main(int argc, char ** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FjtMoveItConnector>());
  rclcpp::shutdown();
  return 0;
}