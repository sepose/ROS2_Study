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
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class FjtRealTimeMonitor : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;

  explicit FjtRealTimeMonitor() : Node("fjt_realtime_monitor")
  {
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    
    // 초기 상태 초기화
    current_state_.positions.assign(6, 0.0);
    current_state_.velocities.assign(6, 0.0);
    current_state_.accelerations.assign(6, 0.0);

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    server_ = rclcpp_action::create_server<FollowJT>(
      this, "/snt_arm_controller/follow_joint_trajectory",
      std::bind(&FjtRealTimeMonitor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FjtRealTimeMonitor::handle_cancel, this, std::placeholders::_1),
      std::bind(&FjtRealTimeMonitor::handle_accepted, this, std::placeholders::_1)
    );

    // 20ms 간격으로 상태 업데이트 및 퍼블리시
    timer_ = this->create_wall_timer(20ms, std::bind(&FjtRealTimeMonitor::on_timer, this));
    
    std::cout << "\033[1;32m[SYSTEM] MoveIt-Native PVAT Monitor (Degree) Ready.\033[0m" << std::endl;
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    executing_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() {
      auto goal = goal_handle->get_goal();
      
      // 실행 전 궤적 정보 복사 (충돌 방지)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        traj_ = goal->trajectory;
      }

      std::cout << "\n\033[1;33m[WAIT] Trajectory Ready. Execute? (y/n): \033[0m" << std::flush;
      char input;
      std::cin >> input;

      if (input == 'y' || input == 'Y') {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          // 핵심: 사용자가 'y'를 입력한 시점을 T=0으로 설정하여 순간이동 방지
          start_time_ = this->now(); 
          
          double dur = (double)traj_.points.back().time_from_start.sec + 
                       (double)traj_.points.back().time_from_start.nanosec * 1e-9;
          end_time_ = start_time_ + rclcpp::Duration::from_seconds(dur);
          
          executing_ = true;
          first_print_ = true; 
        }
        
        // 궤적 종료 시점까지 대기
        while (rclcpp::ok()) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!executing_ || this->now() >= end_time_) break;
          }
          std::this_thread::sleep_for(10ms);
        }

        std::lock_guard<std::mutex> lk(mtx_);
        executing_ = false;
        current_state_ = traj_.points.back(); // 종점 고정
        
        auto res = std::make_shared<FollowJT::Result>();
        res->error_code = FollowJT::Result::SUCCESSFUL;
        goal_handle->succeed(res);
        std::cout << "\n\033[1;32m[DONE] Execution Finished Correctly.\033[0m" << std::endl;
      } else {
        goal_handle->canceled(std::make_shared<FollowJT::Result>());
      }
    }).detach();
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (executing_) {
      double t = (this->now() - start_time_).seconds();
      current_state_ = sample_pvat(traj_, t);
      print_realtime_dashboard();
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_names_;
    js.position = current_state_.positions;
    js.velocity = current_state_.velocities;
    js_pub_->publish(js);
  }

  void print_realtime_dashboard()
  {
    const double rad2deg = 180.0 / M_PI;
    if (!first_print_) std::cout << "\033[4A"; 
    first_print_ = false;

    std::cout << "\033[1;36m[LIVE MOVEIT PATH - DEGREE]\033[0m \033[K" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 2; ++col) {
        int i = row * 2 + col;
        double p = current_state_.positions[i] * rad2deg;
        double v = current_state_.velocities[i] * rad2deg;
        double a = current_state_.accelerations[i] * rad2deg;

        std::cout << "[" << i+1 << "] P:" << std::setw(7) << p 
                  << " V:" << std::setw(6) << v 
                  << " A:" << std::setw(6) << a;
        if (col == 0) std::cout << " | ";
      }
      std::cout << "\033[K" << std::endl;
    }
    std::cout << std::flush;
  }

  trajectory_msgs::msg::JointTrajectoryPoint sample_pvat(const trajectory_msgs::msg::JointTrajectory & traj, double t)
  {
    const auto & pts = traj.points;
    auto to_sec = [](const builtin_interfaces::msg::Duration & d) { 
      return (double)d.sec + (double)d.nanosec * 1e-9; 
    };
    
    if (t <= 0.0) return pts.front();
    if (t >= to_sec(pts.back().time_from_start)) return pts.back();

    // 현재 시간이 속한 구간(segment) 찾기
    size_t i = 0;
    for (; i < pts.size() - 1; ++i) {
        if (t <= to_sec(pts[i+1].time_from_start)) break;
    }

    double t0 = to_sec(pts[i].time_from_start);
    double t1 = to_sec(pts[i+1].time_from_start);
    double T = t1 - t0;
    double h = t - t0;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(6); p.velocities.resize(6); p.accelerations.resize(6);

    for (int k = 0; k < 6; ++k) {
      double p0 = pts[i].positions[k], p1 = pts[i+1].positions[k];
      double v0 = pts[i].velocities[k], v1 = pts[i+1].velocities[k];
      double a0 = pts[i].accelerations.empty() ? 0.0 : pts[i].accelerations[k];
      double a1 = pts[i+1].accelerations.empty() ? 0.0 : pts[i+1].accelerations[k];

      // Quintic Spline (5차 다항식) 보간: 위치, 속도, 가속도 연속성 보장
      double c0 = p0;
      double c1 = v0;
      double c2 = a0 * 0.5;
      double c3 = (10.0*(p1-p0) - (6.0*v0+4.0*v1)*T - (3.0*a0-a1)*T*T*0.5) / std::pow(T, 3);
      double c4 = (-15.0*(p1-p0) + (8.0*v0+7.0*v1)*T + (1.5*a0-a1)*T*T) / std::pow(T, 4);
      double c5 = (6.0*(p1-p0) - 3.0*(v0+v1)*T - 0.5*(a0-a1)*T*T) / std::pow(T, 5);

      p.positions[k] = c5*std::pow(h, 5) + c4*std::pow(h, 4) + c3*std::pow(h, 3) + c2*std::pow(h, 2) + c1*h + c0;
      p.velocities[k] = 5.0*c5*std::pow(h, 4) + 4.0*c4*std::pow(h, 3) + 3.0*c3*std::pow(h, 2) + 2.0*c2*h + c1;
      p.accelerations[k] = 20.0*c5*std::pow(h, 3) + 12.0*c4*std::pow(h, 2) + 6.0*c3*h + 2.0*c2;
    }
    return p;
  }

  std::vector<std::string> joint_names_;
  rclcpp_action::Server<FollowJT>::SharedPtr server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mtx_;
  bool executing_{false};
  bool first_print_{true};
  trajectory_msgs::msg::JointTrajectory traj_;
  trajectory_msgs::msg::JointTrajectoryPoint current_state_;
  rclcpp::Time start_time_, end_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FjtRealTimeMonitor>());
  rclcpp::shutdown();
  return 0;
}