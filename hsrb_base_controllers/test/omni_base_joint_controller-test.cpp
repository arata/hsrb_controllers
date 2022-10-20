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
#include <fstream>
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hsrb_base_controllers/omni_base_joint_controller.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-5;

void spin_some(const rclcpp::Node::SharedPtr& node) {
  while (true) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

namespace hsrb_base_controllers {

class OmniBaseJointControllerTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr urdf_node_;
  OmniBaseJointController::Ptr controller_;
  HardwareStub hardware_;
};

void OmniBaseJointControllerTest::SetUp() {
  node_ = rclcpp::Node::make_shared("test_node");
  controller_ = std::make_shared<OmniBaseJointController>(node_);

  node_->declare_parameter("joints.steer", "base_roll_joint");
  node_->declare_parameter("joints.r_wheel", "base_r_drive_wheel_joint");
  node_->declare_parameter("joints.l_wheel", "base_l_drive_wheel_joint");

  DeclareRobotDescription(node_);
  node_->declare_parameter("parameter_connection_timeout", 0);

  urdf_node_ = rclcpp::Node::make_shared("urdf_node");
  DeclareRobotDescription(urdf_node_);
}

/// ステア軸名が用意されたインターフェースのリスト内に無いときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, BadSteerJointName) {
  node_->set_parameter({rclcpp::Parameter("joints.steer", "bad_joint_name")});
  EXPECT_FALSE(controller_->Init());
}

/// 左車輪軸名が用意されたインターフェースのリストと違うときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, BadLeftWheelJointName) {
  node_->set_parameter({rclcpp::Parameter("joints.l_wheel", "bad_joint_name")});
  EXPECT_FALSE(controller_->Init());
}

/// 右車輪軸名が用意されたインターフェースのリストと違うときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, BadRightWheelJointName) {
  node_->set_parameter({rclcpp::Parameter("joints.r_wheel", "bad_joint_name")});
  EXPECT_FALSE(controller_->Init());
}

/// ステア軸名の指定が無いときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, NoSteerJointName) {
  node_->undeclare_parameter("joints.steer");
  EXPECT_FALSE(controller_->Init());
}

/// 左車輪軸名の指定が無いときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, NoLeftWheelJointName) {
  node_->undeclare_parameter("joints.l_wheel");
  EXPECT_FALSE(controller_->Init());
}

/// 右車輪軸名の指定が無いときに初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, NoRightWheelJointName) {
  node_->undeclare_parameter("joints.r_wheel");
  EXPECT_FALSE(controller_->Init());
}

/// 不正なrobot_descriptionで初期化が失敗するか
TEST_F(OmniBaseJointControllerTest, InvalidRobotDescription) {
  node_->set_parameter({rclcpp::Parameter("robot_description", "invalid")});
  EXPECT_FALSE(controller_->Init());
}

/// 別のノードのロボットモデルを取得して初期化
TEST_F(OmniBaseJointControllerTest, RobotDescriptionFromAnotherNode) {
  node_->undeclare_parameter("robot_description");
  node_->declare_parameter("model_node_name", "urdf_node");

  // std::threadだとinterruptが存在しない
  auto spin_thread = boost::thread(std::bind(spin_some, urdf_node_));
  EXPECT_TRUE(controller_->Init());
  spin_thread.interrupt();
}

/// 指定されたノードにロボットモデルが存在しない
TEST_F(OmniBaseJointControllerTest, NoRobotDescriptionOnAnotherNode) {
  node_->undeclare_parameter("robot_description");
  node_->declare_parameter("model_node_name", "no_description_node");
  auto node = rclcpp::Node::make_shared("no_description_node");

  auto spin_thread = boost::thread(std::bind(spin_some, node));
  EXPECT_FALSE(controller_->Init());
  spin_thread.interrupt();
}

/// 別のノードを参照しようとするが，存在しないノードが指定されている
TEST_F(OmniBaseJointControllerTest, NoRobotDescriptionNode) {
  node_->undeclare_parameter("robot_description");
  EXPECT_FALSE(controller_->Init());
}

/// 台車サイズが取得できること
TEST_F(OmniBaseJointControllerTest, GetOmniBaseSize) {
  EXPECT_TRUE(controller_->Init());

  auto omnibase_size = controller_->omnibase_size();
  EXPECT_EQ(0.266, omnibase_size.tread);
  EXPECT_EQ(0.11, omnibase_size.caster_offset);
  EXPECT_EQ(0.04, omnibase_size.wheel_radius);
}

/// l_wheel_joint_nameが取得できること
TEST_F(OmniBaseJointControllerTest, GetLeftWhellJointName) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_EQ(controller_->l_wheel_joint_name(), "base_l_drive_wheel_joint");
}

/// r_wheel_joint_nameが取得できること
TEST_F(OmniBaseJointControllerTest, GetRightWhellJointName) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_EQ(controller_->r_wheel_joint_name(), "base_r_drive_wheel_joint");
}

/// steer_joint_nameが取得できること
TEST_F(OmniBaseJointControllerTest, GetSteerJointName) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_EQ(controller_->steer_joint_name(), "base_roll_joint");
}

/// 関節へ指令を投げるインターフェースの名前が取得できる
TEST_F(OmniBaseJointControllerTest, GetCommandInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->command_interface_names();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// 関節の状態を入力するインターフェースの名前が取得できる
TEST_F(OmniBaseJointControllerTest, GetStateInterfaceNames) {
  EXPECT_TRUE(controller_->Init());

  auto names = controller_->state_interface_names();
  EXPECT_EQ(names.size(), 6);
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_roll_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_l_drive_wheel_joint/velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "base_r_drive_wheel_joint/velocity"), names.end());
}

/// command_interfaceが不足してActivateに失敗する
TEST_F(OmniBaseJointControllerTest, CommandInterfaceShortage) {
  EXPECT_TRUE(controller_->Init());

  hardware_.command_interfaces.clear();
  EXPECT_FALSE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
}

/// state_interfaceが不足してActivateに失敗する
TEST_F(OmniBaseJointControllerTest, StateInterfaceShortage) {
  EXPECT_TRUE(controller_->Init());

  hardware_.state_interfaces.clear();
  EXPECT_FALSE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
}

/// 指令値が取得できること
TEST_F(OmniBaseJointControllerTest, GetJointCommand) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, Eigen::Vector3d(0.02, 0.03, 0.01));

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(1.40682, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(-0.406818, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(0.262727, joint_command(kJointIDSteer), kEpsilon);

  EXPECT_NEAR(1.40682, hardware_.r_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(-0.406818, hardware_.l_wheel_handle->command(), kEpsilon);
  EXPECT_NEAR(0.0262727, hardware_.steer_handle->command(), kEpsilon);
}

/// 指令値設定で旋回軸速度リミットがかかること
TEST_F(OmniBaseJointControllerTest, SetJointCommandWithYawLimit) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, Eigen::Vector3d(0.02, 0.03, 10.0));

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(0.260327, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(-0.0752804, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(-1.8, joint_command(kJointIDSteer), kEpsilon);
}

/// 指令値設定で車輪速度リミットがかかること
TEST_F(OmniBaseJointControllerTest, SetJointCommandWidhWheelLimit) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, Eigen::Vector3d(1.00, 0.03, 0.01));

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(8.5, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(7.90495, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(0.0862005, joint_command(kJointIDSteer), kEpsilon);
}

/// 旋回軸速度リミットのパラメータが反映されること
TEST_F(OmniBaseJointControllerTest, SetYawVelocityLimit) {
  node_->declare_parameter("yaw_velocity_limit", 0.18);

  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, Eigen::Vector3d(0.02, 0.03, 10.0));

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(0.0260327, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(-0.00752804, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(-0.18, joint_command(kJointIDSteer), kEpsilon);

  node_->set_parameter({rclcpp::Parameter("yaw_velocity_limit", -1.0)});

  EXPECT_TRUE(controller_->Init());
  hardware_.steer_handle->set_current_pos(0.0);

  controller_->SetJointCommand(0.1, Eigen::Vector3d(0.02, 0.03, 10.0));

  joint_command = controller_->joint_command();
  EXPECT_NEAR(0.260327, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(-0.0752804, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(-1.8, joint_command(kJointIDSteer), kEpsilon);
}

/// 車輪軸速度リミットのパラメータが反映されること
TEST_F(OmniBaseJointControllerTest, SetWheelVelocityLimit) {
  node_->declare_parameter("wheel_velocity_limit", 0.85);

  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  controller_->SetJointCommand(0.1, Eigen::Vector3d(1.00, 0.03, 0.01));

  auto joint_command = controller_->joint_command();
  EXPECT_NEAR(0.85, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(0.790495, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(0.00862005, joint_command(kJointIDSteer), kEpsilon);

  node_->set_parameter({rclcpp::Parameter("wheel_velocity_limit", -1.0)});

  EXPECT_TRUE(controller_->Init());
  hardware_.steer_handle->set_current_pos(0.0);

  controller_->SetJointCommand(0.1, Eigen::Vector3d(1.00, 0.03, 0.01));

  joint_command = controller_->joint_command();
  EXPECT_NEAR(8.5, joint_command(kJointIDRightWheel), kEpsilon);
  EXPECT_NEAR(7.90495, joint_command(kJointIDLeftWheel), kEpsilon);
  EXPECT_NEAR(0.0862005, joint_command(kJointIDSteer), kEpsilon);
}

/// 軸位置を取得できること
TEST_F(OmniBaseJointControllerTest, GetJointPositions) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  hardware_.r_wheel_handle->set_current_pos(0.1);
  hardware_.l_wheel_handle->set_current_pos(0.2);
  hardware_.steer_handle->set_current_pos(0.3);

  Eigen::Vector3d joint_positions;
  EXPECT_TRUE(controller_->GetJointPositions(joint_positions));
  EXPECT_EQ(joint_positions(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_positions(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_positions(kJointIDSteer), 0.3);
}

/// 軸速度を取得できること
TEST_F(OmniBaseJointControllerTest, GetJointVelocities) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  hardware_.r_wheel_handle->set_current_vel(0.1);
  hardware_.l_wheel_handle->set_current_vel(0.2);
  hardware_.steer_handle->set_current_vel(0.3);

  Eigen::Vector3d joint_velocities;
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));
  EXPECT_EQ(joint_velocities(kJointIDRightWheel), 0.1);
  EXPECT_EQ(joint_velocities(kJointIDLeftWheel), 0.2);
  EXPECT_EQ(joint_velocities(kJointIDSteer), 0.3);
}

/// 大きすぎる車輪軸速度はエラーになる
TEST_F(OmniBaseJointControllerTest, TooBigWheelVelocities) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));

  node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(controller_->Init());

  hardware_.r_wheel_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.r_wheel_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));

  node_->set_parameter({rclcpp::Parameter("wheel_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(controller_->Init());

  hardware_.r_wheel_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.r_wheel_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));
}

/// 大きすぎる旋回軸速度はエラーになる
TEST_F(OmniBaseJointControllerTest, TooBigSteerVelocities) {
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  Eigen::Vector3d joint_velocities;
  hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));

  node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", 1.0)});
  EXPECT_TRUE(controller_->Init());

  hardware_.steer_handle->set_current_vel(1.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.steer_handle->set_current_vel(1.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));

  node_->set_parameter({rclcpp::Parameter("yaw_actual_velocity_threshold", -1.0)});
  EXPECT_TRUE(controller_->Init());

  hardware_.steer_handle->set_current_vel(1000.0 + kEpsilon);
  EXPECT_FALSE(controller_->GetJointVelocities(joint_velocities));

  hardware_.steer_handle->set_current_vel(1000.0 - kEpsilon);
  EXPECT_TRUE(controller_->GetJointVelocities(joint_velocities));
}

/// 旋回軸の目標位置をリセットできること
TEST_F(OmniBaseJointControllerTest, ResetDesiredSteerPosition) {
  hardware_.steer_handle->set_current_pos(1.0);
  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 1.0);

  hardware_.steer_handle->set_current_pos(2.0);
  controller_->ResetDesiredSteerPosition();
  EXPECT_DOUBLE_EQ(controller_->desired_steer_pos(), 2.0);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
