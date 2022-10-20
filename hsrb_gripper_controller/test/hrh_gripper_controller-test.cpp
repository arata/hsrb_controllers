/*
Copyright (c) 2014 TOYOTA MOTOR CORPORATION
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

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <hsrb_servomotor_protocol/exxx_common.hpp>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {


class GripperControllerTest : public ::testing::Test {
 public:
  void SetUp() override {
    controller_ = std::make_shared<TestableHrhGripperController>();
    controller_node_ = controller_->get_node();

    controller_node_->declare_parameter<std::vector<std::string>>("joints", {kHandJointName});
    // WaitForでupdateが複数回呼ばれる影響を消すために，igainをゼロにしておく
    controller_node_->declare_parameter<double>("force_control_igain", 0.0);
    EXPECT_EQ(controller_->init(kControllerNodeName), controller_interface::return_type::OK);

    hardware_ = std::make_shared<HardwareStub>(kHandJointName);
    controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
    EXPECT_EQ(controller_->configure().label(), controller_interface::state_names::INACTIVE);
    EXPECT_EQ(controller_->activate().label(), controller_interface::state_names::ACTIVE);

    client_node_ = rclcpp::Node::make_shared(kClientNodeName);

    trajectory_publisher_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());

    apply_force_action_client_ = rclcpp_action::create_client<ApplayEffortAction>(
        client_node_, std::string(kControllerNodeName) + "/apply_force");
    grasp_action_client_ = rclcpp_action::create_client<ApplayEffortAction>(
        client_node_, std::string(kControllerNodeName) + "/grasp");
    follow_trajectory_action_client_ = rclcpp_action::create_client<FollowTrajectoryAction>(
        client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");

    EXPECT_TRUE(apply_force_action_client_->wait_for_action_server());
    EXPECT_TRUE(grasp_action_client_->wait_for_action_server());
    EXPECT_TRUE(follow_trajectory_action_client_->wait_for_action_server());
  }

 protected:
  TestableHrhGripperController::Ptr controller_;
  rclcpp::Node::SharedPtr controller_node_;
  HardwareStub::Ptr hardware_;

  rclcpp::Node::SharedPtr client_node_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  using ApplayEffortAction = tmc_control_msgs::action::GripperApplyEffort;
  rclcpp_action::Client<ApplayEffortAction>::SharedPtr apply_force_action_client_;
  rclcpp_action::Client<ApplayEffortAction>::SharedPtr grasp_action_client_;

  using FollowTrajectoryAction = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowTrajectoryAction>::SharedPtr follow_trajectory_action_client_;

  void SpinOnce(rclcpp::WallRate& rate) {
    rate.sleep();
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(controller_node_);
    EXPECT_EQ(controller_->update(), controller_interface::return_type::OK);
  }

  template <typename FutureT>
  void WaitFor(rclcpp::WallRate& rate, const typename std::shared_future<FutureT>& future) {
    while (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::microseconds(1)) ==
           rclcpp::FutureReturnCode::TIMEOUT) {
      SpinOnce(rate);
    }
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<ApplayEffortAction>> StartApplyForceAction() {
    ApplayEffortAction::Goal goal;
    goal.effort = 1.0;
    auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // apply_force開始を待つ
    while (rclcpp::ok()) {
      SpinOnce(rate);
      // 0.1 * -1.0が想定指令値
      if (std::abs(hardware_->position->command() + 0.1) < kEpsilon) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowTrajectoryAction>> StartFollowTrajectoryAction() {
    // 軌道追従中に偏差過大でabortedにならないように，軌道のゴール位置に近い値を入れておく
    hardware_->position->set_current(0.96);

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeTrajectory();
    auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(follow_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // 追従開始を待つ
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 0.96) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<ApplayEffortAction>> StartGraspAction() {
    ApplayEffortAction::Goal grasp_goal;
    grasp_goal.effort = 3.0;
    auto future_goal_handle = grasp_action_client_->async_send_goal(grasp_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // 握り込み開始を待つ
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->grasping_flag->bool_command()) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandGrasp);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  void SendTrajectoryTopic() {
    // 軌道追従中に偏差過大でabortedにならないように，軌道のゴール位置に近い値を入れておく
    hardware_->position->set_current(1.46);

    auto trajectory = MakeTrajectory();
    trajectory.points[0].positions[0] = 1.5;
    trajectory_publisher_->publish(trajectory);

    // 追従開始を待つ
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 1.46) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);
  }

  void PreemptWithApplyForceAction() {
    ApplayEffortAction::Goal goal;
    goal.effort = 1.0;
    goal.do_control_stop = false;

    auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    std::this_thread::sleep_for(std::chrono::milliseconds(2050));

    // 目標力とローパス後の力が同じ値になるようにセットしている
    hardware_->position->set_current(0.0);
    hardware_->spring_l_position->set_current(5.0);
    hardware_->spring_r_position->set_current(5.0);
    SpinOnce(rate);
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);

    // ゲインと差分から導出
    EXPECT_NEAR(hardware_->position->command(), 0.4 * 1.0, kEpsilon);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
        {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
  }

  void PreemptWithFollowTrajectoryAction() {
    // 軌道追従中に偏差過大でabortedにならないように，軌道のゴール位置に近い値を入れておく
    hardware_->position->set_current(0.96);

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeTrajectory();

    auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(follow_goal);
    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // テスト軌道は1.0へ0.5秒で移動
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);
    EXPECT_DOUBLE_EQ(hardware_->position->command(), 1.0);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE(WaitForStatus<FollowTrajectoryAction>(
        {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
  }

  void PreemptWithGraspAction() {
    // アクションを成功させるために，現在力と目標力を合わせておく
    hardware_->effort->set_current(3.0);

    ApplayEffortAction::Goal grasp_goal;
    grasp_goal.effort = 3.0;
    auto future_grasp_goal_handle = grasp_action_client_->async_send_goal(grasp_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_grasp_goal_handle);

    // 握り込み開始を待つ
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->grasping_flag->bool_command()) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandGrasp);

    // 握り込み中
    hardware_->grasping_flag->set_current(true);
    SpinOnce(rate);

    // 握り込み完了
    hardware_->grasping_flag->set_current(false);
    SpinOnce(rate);

    auto grasp_goal_handle = future_grasp_goal_handle.get();
    EXPECT_TRUE(grasp_goal_handle.get());
    EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
        {controller_node_, client_node_}, grasp_goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
  }

  void PreemptWithTrajectoryTopic() {
    // 軌道追従中に偏差過大でabortedにならないように，軌道のゴール位置に近い値を入れておく
    hardware_->position->set_current(1.46);

    auto trajectory = MakeTrajectory();
    trajectory.points[0].positions[0] = 1.5;
    trajectory_publisher_->publish(trajectory);

    // テスト軌道は1.5へ0.5秒で移動
    rclcpp::WallRate rate(100.0);
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);
    EXPECT_DOUBLE_EQ(hardware_->position->command(), 1.5);
  }
};

TEST_F(GripperControllerTest, ApplyForceAction) {
  ApplayEffortAction::Goal goal;
  goal.effort = 1.0;
  goal.do_control_stop = false;

  auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

  rclcpp::WallRate rate(100.0);
  WaitFor(rate, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  // 目標力とローパス後の力が同じ値になるようにセットしている
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  SpinOnce(rate);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  // ゲインと差分から導出
  EXPECT_NEAR(hardware_->position->command(), 0.4 * 1.0, kEpsilon);

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);
}

TEST_F(GripperControllerTest, FollowTrajectoryAction) {
  // 軌道追従中に偏差過大でabortedにならないように，軌道のゴール位置に近い値を入れておく
  hardware_->position->set_current(0.96);

  FollowTrajectoryAction::Goal goal;
  goal.trajectory = MakeTrajectory();
  auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(goal);

  rclcpp::WallRate rate(100.0);
  WaitFor(rate, future_goal_handle);

  // テスト軌道は1.0へ0.5秒で移動
  std::vector<double> command_positions;
  for (int i = 0; i < 60; ++i) {
    SpinOnce(rate);
    command_positions.push_back(hardware_->position->command());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<FollowTrajectoryAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  double previous_command = 0.96;
  for (double command : command_positions) {
    EXPECT_LE(command, 1.0);
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_positions.front(), command_positions.back());

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandPosition);
}

TEST_F(GripperControllerTest, GraspAction) {
  // アクションを成功させるために，現在力と目標力を合わせておく
  hardware_->effort->set_current(3.0);
  hardware_->grasping_flag->set_current(false);

  ApplayEffortAction::Goal goal;
  goal.effort = 3.0;
  auto future_goal_handle = grasp_action_client_->async_send_goal(goal);

  rclcpp::WallRate rate(100.0);
  WaitFor(rate, future_goal_handle);

  while (rclcpp::ok()) {
    SpinOnce(rate);
    // 握り込み開始でbreak
    if (hardware_->grasping_flag->bool_command()) {
      EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);
      break;
    }
  }
  // 握り込み中
  hardware_->grasping_flag->set_current(true);
  SpinOnce(rate);

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // 握り込み完了
  hardware_->grasping_flag->set_current(false);
  SpinOnce(rate);

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), hsrb_servomotor_protocol::kDriveModeHandGrasp);
}

TEST_F(GripperControllerTest, ApplyForceAndFollowTrajectory) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, ApplyForceAndGrasp) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithGraspAction();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, ApplyForceAndSendTrajectoryTopic) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndApplyForce) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE(WaitForStatus<FollowTrajectoryAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndGrasp) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithGraspAction();

  EXPECT_TRUE(WaitForStatus<FollowTrajectoryAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndSendTrajectoryTopic) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE(WaitForStatus<FollowTrajectoryAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, GraspAndApplyForce) {
  auto goal_handle = StartGraspAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, GraspAndFollowTrajectory) {
  auto goal_handle = StartGraspAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, GraspAndSendTrajectoryTopic) {
  auto goal_handle = StartGraspAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE(WaitForStatus<ApplayEffortAction>(
      {controller_node_, client_node_}, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndApplyForce) {
  SendTrajectoryTopic();
  PreemptWithApplyForceAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndFollowTrajectory) {
  SendTrajectoryTopic();
  PreemptWithGraspAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndGrasp) {
  SendTrajectoryTopic();
  PreemptWithFollowTrajectoryAction();
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
