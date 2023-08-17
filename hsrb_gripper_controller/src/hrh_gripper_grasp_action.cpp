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
#include "hsrb_gripper_controller/hrh_gripper_grasp_action.hpp"

#include <hsrb_servomotor_protocol/exxx_common.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"

namespace {
// デフォルトトルク誤差閾値[Nm]
const double kDefaultTorqueGoalTolerance = 1.0;

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperGraspAction::HrhGripperGraspAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "~/grasp", hsrb_servomotor_protocol::kDriveModeHandGrasp),
      goal_tolerance_(kDefaultTorqueGoalTolerance),
      is_sent_start_grasping_(false) {}

/// 周期更新処理
void HrhGripperGraspAction::Update(const rclcpp::Time& time) {
  if (!IsActive()) {
    return;
  }
  bool grasping_flag = controller_->GetCurrentGraspingFlag();
  {
    std::lock_guard<std::mutex> guard(mutex_);
    bool start_grasping_flag;
    if (grasping_flag) {
      // 握り込み開始フラグON済み
      start_grasping_flag = false;
      is_sent_start_grasping_ = true;
    } else {
      if (is_sent_start_grasping_) {
        // 握り込み開始フラグ送信済みで握り込み完了
        start_grasping_flag = false;
      } else {
        // 握り込み開始フラグをまだ送っていない
        start_grasping_flag = true;
      }
    }
    controller_->SetGraspCommand(start_grasping_flag, command_torque_);
  }
  CheckForSuccess();
}

/// アクションの初期化の実装
// bool HrhGripperGraspAction::InitImpl(const rclcpp::Node::SharedPtr& node) {
bool HrhGripperGraspAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  goal_tolerance_ = GetPositiveParameter(node, "torque_goal_tolerance", kDefaultTorqueGoalTolerance);
  return true;
}

/// アクションの目標を更新する
void HrhGripperGraspAction::UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) {
  std::lock_guard<std::mutex> guard(mutex_);
  command_torque_ = goal.effort;
  is_sent_start_grasping_ = false;
}

void HrhGripperGraspAction::CheckForSuccess() {
  bool grasping_flag = controller_->GetCurrentGraspingFlag();
  // コントロールテーブル上に握り込み開始フラグをセットすると握り込み開始、
  // Stallするとフラグがリセットされる仕様(指令と状態確認のフィールドが同一)
  // 握り込み開始フラグを送信済みで、かつ現在の握り込みフラグがリセットされているなら握り込み完了である。
  bool has_completed;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    has_completed = is_sent_start_grasping_ && !grasping_flag;
  }
  if (has_completed) {
    auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
    result->stalled = true;
    result->effort = controller_->GetCurrentTorque();;

    // stallして平衡状態になった時に指令値と現在値を比較
    bool is_succeeded;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      is_succeeded = fabs(command_torque_ - result->effort) < goal_tolerance_;
    }

    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (is_succeeded) {
      active_goal->gh_->succeed(result);
    } else {
      active_goal->gh_->abort(result);
    }
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

}  // namespace hsrb_gripper_controller

