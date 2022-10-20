/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_
#define HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hsrb_base_controllers/controller_command_interface.hpp>
#include <hsrb_base_controllers/omni_base_state.hpp>

namespace hsrb_base_controllers {

/// 入力指令クラス
class CommandSubscriber : private boost::noncopyable {
 public:
  CommandSubscriber(const rclcpp::Node::SharedPtr& node,
                    IControllerCommandInterface* controller);
  virtual ~CommandSubscriber() {}

 protected:
  rclcpp::Node::SharedPtr node_;
  IControllerCommandInterface* controller_;
};

/// 入力速度指令クラス
class CommandVelocitySubscriber : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<CommandVelocitySubscriber>;

  CommandVelocitySubscriber(const rclcpp::Node::SharedPtr& node,
                            IControllerCommandInterface* controller);
  virtual ~CommandVelocitySubscriber() {}

 private:
  // 入力指令速度コールバック
  void CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 入力速度サブスクライバ
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
};

/// 入力軌道指令クラス
class CommandTrajectorySubscriber : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<CommandTrajectorySubscriber>;

  CommandTrajectorySubscriber(const rclcpp::Node::SharedPtr& node,
                              IControllerCommandInterface* controller);
  virtual ~CommandTrajectorySubscriber() {}

 private:
  // 入力指令軌道コールバック
  void CommandTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // 入力軌道サブスクライバ
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
};


/// 入力軌道アクション指令クラス
// TODO(Takeshita) toleranceの扱い周りがros1の頃より劣化しているので要検討
//                 原因はfollow_trajectory_controllerの実装に寄せているから
//                 velocity使っていない，アクションゴールのtoleranceを無視
class TrajectoryActionServer : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<TrajectoryActionServer>;

  TrajectoryActionServer(const rclcpp::Node::SharedPtr& node,
                         const std::vector<std::string>& cordinates,
                         IControllerCommandInterface* controller);
  virtual ~TrajectoryActionServer() {}

  // アクションの結果の更新を行う
  void UpdateActionResult(int32_t error_code);
  // フィードバックの発行
  void SetFeedback(const ControllerBaseState& state, const rclcpp::Time& stamp);
  // 現在追従中のゴールをクリア
  void PreemptActiveGoal();

 private:
  // アクション状態更新周期
  double action_monitor_period_;
  // 台車関節名
  std::vector<std::string> cordinates_;
  // アクションのGoalHandle
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> goal_handle_buffer_;

  // アクションサーバーとコールバック関数群
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);

  // アクション実行時のタイマー
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_*/
