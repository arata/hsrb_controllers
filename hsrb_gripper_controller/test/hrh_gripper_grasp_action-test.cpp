/*
Copyright (c) 2022 TOYOTA MOTOR CORPORATION
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
#include <gtest/gtest.h>

#include <hsrb_servomotor_protocol/exxx_common.hpp>

#include <hsrb_gripper_controller/hrh_gripper_grasp_action.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {

class GraspActionTest
    : public GripperActionTestBase<tmc_control_msgs::action::GripperApplyEffort, HrhGripperGraspAction> {
 public:
  GraspActionTest() : GripperActionTestBase("grasp") {}
  virtual ~GraspActionTest() = default;
};

TEST_F(GraspActionTest, ActionSucceeded) {
  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 握り込み開始
  hardware_->effort->set_current(0.0);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // 握り込み中
  hardware_->grasping_flag->set_current(true);
  action_server_->Update(node_->now());

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // 握り込み完了
  hardware_->effort->set_current(2.1);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

TEST_F(GraspActionTest, ActionAborted) {
  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 握り込み開始
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  // 握り込み中
  hardware_->grasping_flag->set_current(true);
  action_server_->Update(node_->now());

  // 握り込み完了
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(GraspActionTest, PreemptFromOutside) {
  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 握り込み開始
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // 外部からの割り込み
  action_server_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // 割り込みがあったので状態が変化しない
  hardware_->grasping_flag->set_current(true);
  action_server_->Update(node_->now());

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);
}

TEST_F(GraspActionTest, CancelGoal) {
  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 握り込み開始
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  // キャンセルを投げる
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GraspActionTest, GoalTorelance) {
  node_->set_parameter({rclcpp::Parameter("torque_goal_tolerance", 1.2)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 握り込み開始
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  // 握り込み中
  hardware_->grasping_flag->set_current(true);
  action_server_->Update(node_->now());

  // 握り込み完了
  hardware_->grasping_flag->set_current(false);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

TEST_F(GraspActionTest, TargetMode) {
  EXPECT_EQ(action_server_->target_mode(), hsrb_servomotor_protocol::kDriveModeHandGrasp);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
