/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>

#include <hsrb_base_controllers/command_subscriber.hpp>
#include <hsrb_base_controllers/controller_command_interface.hpp>
#include <hsrb_base_controllers/omni_base_control_method.hpp>
#include <hsrb_base_controllers/omni_base_joint_controller.hpp>
#include <hsrb_base_controllers/omni_base_odometry.hpp>

namespace hsrb_base_controllers {

/// 全方位台車速度コントローラクラス
class OmniBaseController
    : public controller_interface::ControllerInterface,
      public IControllerCommandInterface {
 public:
  OmniBaseController() = default;
  ~OmniBaseController() = default;

  // コントローラ初期化
  controller_interface::return_type init(const std::string& controller_name) override;

  // ros2_controlのインターフェース設定
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 台車ジョイント角速度を計算し更新
  controller_interface::return_type update() override;

  // configure時に呼ばれる関数
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;
  // activate時に呼ばれる関数
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  // deactivate時に呼ばれる関数
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // 指令を受付可能か返す
  bool IsAcceptable() override;

  // 入力速度指令をセットする
  void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) override;

  // 入力軌道指令を検証する
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;
  // 入力軌道指令をセットする
  void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) override;
  // 入力軌道をリセットする
  void ResetTrajectory() override;

 protected:
  // ControllerInterface::init以外の部分の初期化，テストのための分割
  bool InitImpl();

  // 軌道追従の許容幅
  joint_trajectory_controller::SegmentTolerances default_tolerances_;
  joint_trajectory_controller::SegmentTolerances active_tolerances_;
  // 軌道追従中のtolerancesのチェック
  // 追従を続ける場合は正の数を，追従を止める場合はcontrol_msgs/action/FollowJointTrajectoryのエラーコード(0 ~ -5)を返す
  int32_t CheckTorelances(const ControllerBaseState& state, bool before_last_point, double time_from_trajectory_end);

  // 入力速度指令サブスクライバ
  CommandVelocitySubscriber::Ptr velocity_subscriber_;
  // 入力軌道指令サブスクライバ
  CommandTrajectorySubscriber::Ptr trajectory_subscriber_;
  // 入力軌道Action指令サーバ
  TrajectoryActionServer::Ptr trajectory_action_;

  // Jointコントローラ
  OmniBaseJointController::Ptr joint_controller_;

  // 台車のホイールオドメトリ計算クラス
  BaseOdometry::Ptr base_odometry_;
  WheelOdometry::Ptr wheel_odometry_;

  // 台車の指令速度を計算するクラス
  OmniBaseVelocityControl::Ptr velocity_control_;
  OmniBaseTrajectoryControl::Ptr trajectory_control_;

  // 台車の状態の発行
  StatePublisher::Ptr joint_state_publisher_;
  StatePublisher::Ptr base_state_publisher_;

  // 前回update関数が呼ばれた時刻
  rclcpp::Time last_update_time_;
};

}  // namespace hsrb_base_controllers

#endif/*HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_*/
