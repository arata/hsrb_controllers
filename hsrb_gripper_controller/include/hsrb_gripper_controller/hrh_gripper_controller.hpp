/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_CONTROLLER_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperController
/// @brief High-Ratio-Hypoidグリッパコントローラ
class HrhGripperController : public controller_interface::ControllerInterface {
 public:
  using Ptr = std::shared_ptr<HrhGripperController>;

  HrhGripperController();

  controller_interface::return_type init(const std::string& controller_name) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /// 指令を受付可能か返す
  bool IsAcceptable();

  /// アクティブなゴールを中断する
  void PreemptActiveGoal();

  /// インターフェースへのアクセス
  double GetCurrentPosition() const;
  double GetCurrentVelocity() const;
  double GetCurrentTorque() const;
  bool GetCurrentGraspingFlag() const;

  double GetLeftSpringPosition() const;
  double GetRightSpringPosition() const;

  void SetComandPosition(double position);
  void SetGraspCommand(bool grasping_flag, double effort);

  std::string joint_name() const { return joint_name_; }

  /// 制御モードを変える
  /// @param[in] mode 制御モード
  void ChangeControlMode(IHrhGripperAction::Ptr action);

 protected:
  // ControllerInterface::init以外の部分の初期化，テストのための分割
  bool InitImpl();

 private:
  /// 制御モード指令バッファ
  realtime_tools::RealtimeBuffer<int32_t> command_control_mode_;

  /// 制御対象関節名
  std::string joint_name_;

  /// 左右の指関節名
  std::string left_spring_joint_;
  std::string right_spring_joint_;

  /// TODO(Takeshita) indexでなくポインタをもたせる？
  /// インターフェースのインデックス
  uint32_t current_position_index_;
  uint32_t current_velocity_index_;
  uint32_t current_effort_index_;
  uint32_t current_grasping_flag_index_;
  uint32_t current_left_spring_index_;
  uint32_t current_right_spring_index_;
  uint32_t command_position_index_;
  uint32_t command_effort_index_;
  uint32_t command_grasping_flag_index_;
  uint32_t command_drive_mode_index_;

  /// 全アクション
  std::vector<IHrhGripperAction::Ptr> actions_;

  /// 実行中のアクション
  IHrhGripperAction::Ptr active_action_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_CONTROLLER_HPP_
