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

#include <hsrb_gripper_controller/hrh_gripper_apply_force_action.hpp>

#include "utils.hpp"


namespace hsrb_gripper_controller {

class ApplyForceActionTest
    : public GripperActionTestBase<tmc_control_msgs::action::GripperApplyEffort, HrhGripperApplyForceAction> {
 public:
  ApplyForceActionTest() : GripperActionTestBase("apply_force") {}
  virtual ~ApplyForceActionTest() = default;
};

TEST_F(ApplyForceActionTest, ActionSucceeded) {
  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // デフォルトのゲインと差分から導出
  const auto last_command = hardware_->position->command();
  EXPECT_NEAR(last_command, 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  // do_control_stopがfalseなので，アクション完了後も位置を更新し続ける
  action_server_->Update(node_->now());
  EXPECT_GT(std::abs(hardware_->position->command() - last_command), kEpsilon);
}

TEST_F(ApplyForceActionTest, ControlStopAfterCompletion) {
  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = true;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // デフォルトのゲインと差分から導出
  const auto last_command = hardware_->position->command();
  EXPECT_NEAR(last_command, 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  // do_control_stopがtrueなので，アクション完了後は位置を更新しない
  action_server_->Update(node_->now());
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, ActionAborted) {
  ActionType::Goal goal;
  goal.effort = 1.15;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(ApplyForceActionTest, PreemptFromOutside) {
  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  const auto last_command = hardware_->position->command();

  // 外部からの割り込み
  action_server_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // 割り込みがあったので,do_control_stopがfalseでも更新を止める
  action_server_->Update(node_->now());
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, CancelGoal) {
  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  const auto last_command = hardware_->position->command();

  // キャンセルを投げる
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // 割り込みがあったので,do_control_stopがfalseでも更新を止める
  action_server_->Update(node_->now());
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, ForceGoalTolerance) {
  node_->set_parameter({rclcpp::Parameter("force_goal_tolerance", 0.2)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.15;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

TEST_F(ApplyForceActionTest, StallVelocityThreshold) {
  node_->set_parameter({rclcpp::Parameter("stall_velocity_threshold", 0.03)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(500.0);
  hardware_->spring_r_position->set_current(500.0);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.03);
  action_server_->Update(node_->now());

  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(ApplyForceActionTest, StallTimeout) {
  node_->set_parameter({rclcpp::Parameter("stall_timeout", 2.2)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(500.0);
  hardware_->spring_r_position->set_current(500.0);
  action_server_->Update(node_->now());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));

  // 上のWaitForStatusで1秒待つので，残りを待てばいい
  std::this_thread::sleep_for(std::chrono::milliseconds(1250));
  action_server_->Update(node_->now());

  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(ApplyForceActionTest, ForceControlPgain) {
  node_->set_parameter({rclcpp::Parameter("force_control_pgain", 1.0)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // ゲインと差分から導出
  EXPECT_NEAR(hardware_->position->command(), 1.0 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceControlIgain) {
  node_->set_parameter({rclcpp::Parameter("force_control_igain", 1.0)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // ゲインと差分から導出
  EXPECT_NEAR(hardware_->position->command(), 0.1 * -0.05 + 1.0 * -0.05 + 0.4 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceControlDgain) {
  node_->set_parameter({rclcpp::Parameter("force_control_dgain", 1.0)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // ゲインと差分から導出
  EXPECT_NEAR(hardware_->position->command(), 0.1 * -0.05 + 0.15 * -0.05 + 1.0 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceIerrMax) {
  node_->set_parameter({rclcpp::Parameter("force_control_pgain", 0.0)});
  node_->set_parameter({rclcpp::Parameter("force_control_igain", 0.2)});
  node_->set_parameter({rclcpp::Parameter("force_control_dgain", 0.0)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 20.0;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(0.0);
  hardware_->spring_r_position->set_current(0.0);
  action_server_->Update(node_->now());

  EXPECT_NEAR(hardware_->position->command(), 0.2 * -0.15, kEpsilon);

  // キャンセルで終わらせておく
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // force_ierr_maxを調整して，反映されることを確認
  node_->set_parameter({rclcpp::Parameter("force_ierr_max", 0.05)});
  EXPECT_TRUE(action_server_->Init(node_));

  future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  action_server_->Update(node_->now());

  EXPECT_NEAR(hardware_->position->command(), 0.2 * -0.05, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceCalibDataPath) {
  node_->set_parameter({rclcpp::Parameter("force_calib_data_path", "test.yaml")});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  action_server_->Update(node_->now());

  // キャリブ結果がないときの指令値と変化があることだけをチェック
  // キャリブ結果が正しく反映されているかのテストはhrh_gripper_controller_apply_force_calculator-test.cpp
  EXPECT_GT(std::abs(hardware_->position->command() - 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0), kEpsilon);
}

TEST_F(ApplyForceActionTest, TargetMode) {
  EXPECT_EQ(action_server_->target_mode(), hsrb_servomotor_protocol::kDriveModeHandPosition);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
