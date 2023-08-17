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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_

#include <limits>
#include <memory>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperFollowTrajectoryAction
/// @brief Hrh軌道追従アクションクラス
class HrhGripperFollowTrajectoryAction : public HrhGripperAction<control_msgs::action::FollowJointTrajectory> {
 public:
  /// コンストラクタ
  /// @param [in] controller 親コントローラ
  explicit HrhGripperFollowTrajectoryAction(HrhGripperController* controller);
  virtual ~HrhGripperFollowTrajectoryAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

  private:
    // added
    joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method_{joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION};

 protected:
  /// アクションの初期化の実装
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// ゴールが受け入れ可能かをチェックする
  bool ValidateGoal(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// アクションの目標を更新する
  void UpdateActionImpl(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;

  /// デフォルトのゴール位置の許容誤差[rad]
  double default_goal_tolerance_;
  /// デフォルトのゴール到達時刻の許容誤差[s]
  double default_goal_time_tolerance_;

  /// 軌道指令がトピックで届いた際のコールバック
  /// @param [in] msg 軌道
  void TrajectoryCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  /// 軌道指令受信
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;

  /// 軌道を保持する
  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  /// @struct GoalCondition
  /// @brief ゴール条件
  struct GoalCondition {
    /// 指令位置[rad]
    double position;
    /// 目標到達時刻
    rclcpp::Time expected_arrival_time;
    /// 軌道追従をやめる時刻
    rclcpp::Time abort_time;
    /// ゴール位置の許容誤差
    double goal_tolerance;
  };
  /// ゴール条件のバッファ
  realtime_tools::RealtimeBuffer<GoalCondition> goal_condition_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
