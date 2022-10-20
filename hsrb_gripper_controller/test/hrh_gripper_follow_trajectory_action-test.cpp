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

#include <hsrb_gripper_controller/hrh_gripper_follow_trajectory_action.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {

class FollowTrajectoryActionTest
    : public GripperActionTestBase<control_msgs::action::FollowJointTrajectory, HrhGripperFollowTrajectoryAction> {
 public:
  FollowTrajectoryActionTest() : GripperActionTestBase("follow_joint_trajectory") {}
  virtual ~FollowTrajectoryActionTest() = default;

  void SetUp() override;
};

void FollowTrajectoryActionTest::SetUp() {
  GripperActionTestBase::SetUp();
  hardware_->position->set_current(0.5);
  hardware_->velocity->set_current(0.0);
}

TEST_F(FollowTrajectoryActionTest, ActionSucceeded) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 1回目のupdate時に軌道が確定するので，Updateを呼んだあとにpositionを目標位置付近に動かす
  action_server_->Update(node_->now());

  std::vector<double> command_positions;
  command_positions.push_back(hardware_->position->command());

  hardware_->position->set_current(1.04);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
    command_positions.push_back(hardware_->position->command());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  double previous_command = 0.5;
  for (double command : command_positions) {
    EXPECT_LE(command, 1.0);
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_positions.front(), command_positions.back());
}

TEST_F(FollowTrajectoryActionTest, GoalToleranceViolated) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 1回目のupdate時に軌道が確定するので，Updateを呼んだあとにpositionを目標位置付近に動かす
  action_server_->Update(node_->now());

  hardware_->position->set_current(1.06);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(FollowTrajectoryActionTest, PreemptFromOutside) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 30; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
  }
  double last_command = hardware_->position->command();

  // 外部からの割り込み
  action_server_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // 割り込みがあったので状態が変化しない
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowTrajectoryActionTest, CancelGoal) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 30; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
  }
  double last_command = hardware_->position->command();

  // キャンセルを投げる
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  // 割り込みがあったので状態が変化しない
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowTrajectoryActionTest, PositionGoalTolerance) {
  node_->set_parameter({rclcpp::Parameter("position_goal_tolerance", 0.07)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 1回目のupdate時に軌道が確定するので，Updateを呼んだあとにpositionを目標位置付近に動かす
  action_server_->Update(node_->now());

  hardware_->position->set_current(1.06);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

TEST_F(FollowTrajectoryActionTest, PositionGoalTimeTolerance) {
  node_->set_parameter({rclcpp::Parameter("position_goal_time_tolerance", 0.15)});
  EXPECT_TRUE(action_server_->Init(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // 1回目のupdate時に軌道が確定するので，Updateを呼んだあとにpositionを目標位置付近に動かす
  action_server_->Update(node_->now());

  hardware_->position->set_current(1.06);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
  }

  // golt time toleranceを超えたあとにUpdateを呼んでいないので，abortedが飛んでこないはず
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE(WaitForStatus<ActionType>({node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

TEST_F(FollowTrajectoryActionTest, InvalidJointName) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();
  goal.trajectory.joint_names = {"invalid"};

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowTrajectoryActionTest, InvalidJointNumbers) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();
  goal.trajectory.joint_names = {kHandJointName, kHandJointName};

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowTrajectoryActionTest, ZeroDurationTrajectory) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();
  goal.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowTrajectoryActionTest, PastTrajectory) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();
  goal.trajectory.header.stamp = node_->now() - rclcpp::Duration(2, 0);

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowTrajectoryActionTest, EmptyTrajectory) {
  ActionType::Goal goal;
  goal.trajectory = MakeTrajectory();
  goal.trajectory.points.clear();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowTrajectoryActionTest, AcceptTrajectoryTopic) {
  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS());

  publisher->publish(MakeTrajectory());

  std::vector<double> command_positions;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
    command_positions.push_back(hardware_->position->command());
  }

  double previous_command = 0.5;
  for (double command : command_positions) {
    EXPECT_LE(command, 1.0);
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_positions.front(), command_positions.back());
}

TEST_F(FollowTrajectoryActionTest, RejectTrajectoryTopic) {
  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS());

  auto trajectory = MakeTrajectory();
  trajectory.joint_names = {"invalid"};
  publisher->publish(trajectory);

  double last_command = hardware_->position->command();

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    action_server_->Update(node_->now());
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowTrajectoryActionTest, TargetMode) {
  EXPECT_EQ(action_server_->target_mode(), hsrb_servomotor_protocol::kDriveModeHandPosition);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
