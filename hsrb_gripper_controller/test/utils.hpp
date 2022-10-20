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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/controller_state_names.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hsrb_gripper_controller/hrh_gripper_controller.hpp>

#include "hardware_stub.hpp"

namespace hsrb_gripper_controller {

const char* const kControllerNodeName = "controller_manager";
const char* const kClientNodeName = "test_node";

const char* const kHandJointName = "hand_motor_joint";

constexpr double kEpsilon = 1e-6;


template<typename Action>
bool WaitForStatus(const std::vector<rclcpp::Node::SharedPtr>& node_ptrs,
                   typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
                   int8_t expected) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(1.0);
  while (goal_handle->get_status() != expected) {
    for (auto node_ptr : node_ptrs) { rclcpp::spin_some(node_ptr); }
    if (std::chrono::system_clock::now() > end_time) {
      std::cout << "The last status is " << static_cast<int32_t>(goal_handle->get_status()) << std::endl;
      return false;
    }
  }
  return true;
}


trajectory_msgs::msg::JointTrajectory MakeTrajectory() {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.0};
  point.time_from_start = rclcpp::Duration(0, 500000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {kHandJointName};
  trajectory.points = {point};
  return trajectory;
}


class TestableHrhGripperController : public HrhGripperController {
 public:
  using Ptr = std::shared_ptr<TestableHrhGripperController>;

  TestableHrhGripperController();
  ~TestableHrhGripperController() = default;

  controller_interface::return_type init(const std::string& controller_name) override;

  void SkipConfigure();
};

TestableHrhGripperController::TestableHrhGripperController() {
  EXPECT_EQ(ControllerInterface::init(kControllerNodeName), controller_interface::return_type::OK);
}

controller_interface::return_type TestableHrhGripperController::init(const std::string& controller_name) {
  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

void TestableHrhGripperController::SkipConfigure() {
  lifecycle_state_ = rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                             controller_interface::state_names::INACTIVE);
}

template<typename RosActionType, typename ActionServerType>
class GripperActionTestBase : public ::testing::Test {
 public:
  explicit GripperActionTestBase(const std::string& action_name) : action_name_(action_name) {}
  virtual ~GripperActionTestBase() = default;

  void SetUp() override;

 protected:
  TestableHrhGripperController::Ptr controller_;
  rclcpp::Node::SharedPtr node_;
  HardwareStub::Ptr hardware_;

  IHrhGripperAction::Ptr action_server_;

  using ActionType = RosActionType;
  std::string action_name_;
  typename rclcpp_action::Client<RosActionType>::SharedPtr action_client_;
};

template<typename RosActionType, typename ActionServerType>
void GripperActionTestBase<RosActionType, ActionServerType>::SetUp() {
  controller_ = std::make_shared<TestableHrhGripperController>();
  node_ = controller_->get_node();

  node_->declare_parameter<std::vector<std::string>>("joints", {kHandJointName});
  EXPECT_EQ(controller_->init(kControllerNodeName), controller_interface::return_type::OK);

  hardware_ = std::make_shared<HardwareStub>(kHandJointName);
  controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
  controller_->SkipConfigure();
  EXPECT_EQ(controller_->activate().label(), controller_interface::state_names::ACTIVE);

  action_server_ = std::make_shared<ActionServerType>(controller_.get());
  EXPECT_TRUE(action_server_->Init(node_));

  action_client_ = rclcpp_action::create_client<RosActionType>(
      node_, std::string(kControllerNodeName) + "/" + action_name_);
  EXPECT_TRUE(action_client_->wait_for_action_server());
}

}  // namespace hsrb_gripper_controller
