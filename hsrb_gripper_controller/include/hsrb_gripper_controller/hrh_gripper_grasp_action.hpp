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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_GRASP_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_GRASP_ACTION_HPP_

#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperGraspAction
/// @brief Hrh握り込み制御アクションクラス
class HrhGripperGraspAction : public HrhGripperAction<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  /// コンストラクタ
  /// @param [in] controller 親コントローラ
  explicit HrhGripperGraspAction(HrhGripperController* controller);
  virtual ~HrhGripperGraspAction() = default;

  void Update(const rclcpp::Time& time) override;

 protected:
  /// アクションの初期化の実装
  //bool InitImpl(const rclcpp::Node::SharedPtr& node) override;
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  /// アクションの目標を更新する
  void UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) override;

  /// 指令値と状態
  std::mutex mutex_;
  double command_torque_;
  bool is_sent_start_grasping_;

  /// アクション成否判定
  void CheckForSuccess();

  /// ゴールトルクの許容誤差[Nm]
  double goal_tolerance_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_GRASP_ACTION_HPP_
