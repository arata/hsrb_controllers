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
#include <hsrb_base_controllers/omni_base_state.hpp>

#include <string>
#include <vector>

#include <angles/angles.h>

#include "utils.hpp"

namespace {
// 台車状態パブリッシュ周波数[Hz]
const double kDefaultStatePublishRate = 50.0;

void ConvertVector(const Eigen::VectorXd& input_vector,
                   std::vector<double>& dst_vector) {
  dst_vector.resize(input_vector.size());
  Eigen::Map<Eigen::VectorXd> map(&dst_vector[0], dst_vector.size());
  map = input_vector;
}
}  // namespace

namespace hsrb_base_controllers {

void ControllerState::UpdateError() {
  if (actual.positions.size() == desired.positions.size()) {
    error.positions.resize(actual.positions.size());
    for (uint32_t i = 0; i < actual.positions.size(); ++i) {
      error.positions[i] = desired.positions[i] - actual.positions[i];
    }
  } else {
    error.positions.clear();
  }

  if (actual.velocities.size() == desired.velocities.size()) {
    error.velocities.resize(actual.velocities.size());
    for (uint32_t i = 0; i < actual.velocities.size(); ++i) {
      error.velocities[i] = desired.velocities[i] - actual.velocities[i];
    }
  } else {
    error.velocities.clear();
  }

  if (actual.accelerations.size() == desired.accelerations.size()) {
    error.accelerations.resize(actual.accelerations.size());
    for (uint32_t i = 0; i < actual.accelerations.size(); ++i) {
      error.accelerations[i] = desired.accelerations[i] - actual.accelerations[i];
    }
  } else {
    error.accelerations.clear();
  }
}


ControllerBaseState::ControllerBaseState(const Eigen::Vector3d& actual_positions,
                                         const Eigen::Vector3d& actual_velocities,
                                         const std::vector<double>& desired_positions,
                                         const std::vector<double>& desired_velocities,
                                         const std::vector<double>& desired_accelerations) {
  ConvertVector(actual_positions, actual.positions);
  // TODO(Takeshita) ここで変換しているのが微妙だなぁ
  // base_velocity_はbase_footprint基準なので,odom基準に変換
  Eigen::Matrix3d rot_mat;
  rot_mat << cos(actual_positions[kIndexBaseTheta]), -sin(actual_positions[kIndexBaseTheta]), 0.0,
             sin(actual_positions[kIndexBaseTheta]), cos(actual_positions[kIndexBaseTheta]), 0.0,
             0.0, 0.0, 1.0;
  Eigen::Vector3d transformed_velocity(rot_mat * actual_velocities);
  ConvertVector(transformed_velocity, actual.velocities);
  desired.positions = desired_positions;
  desired.velocities = desired_velocities;
  desired.accelerations = desired_accelerations;

  UpdateError();
}

ControllerBaseState::ControllerBaseState(const Eigen::Vector3d& actual_positions,
                                         const Eigen::Vector3d& actual_velocities)
    : ControllerBaseState(actual_positions, actual_velocities, {}, {}, {}) {}

void ControllerBaseState::UpdateError() {
  ControllerState::UpdateError();

  if (error.positions.size() == kNumBaseCoordinateIDs) {
    error.positions[kIndexBaseTheta] = angles::shortest_angular_distance(
        actual.positions[kIndexBaseTheta], desired.positions[kIndexBaseTheta]);
  }
}

ControllerJointState::ControllerJointState(const Eigen::Vector3d& actual_positions,
                                           const Eigen::Vector3d& actual_velocities,
                                           double desired_yaw_position,
                                           const Eigen::Vector3d& desired_velocities) {
  ConvertVector(actual_positions, actual.positions);
  ConvertVector(actual_velocities, actual.velocities);
  ConvertVector(desired_velocities, desired.velocities);
  desired.positions.resize(3, 0.0);
  desired.positions[kJointIDSteer] = desired_yaw_position;

  UpdateError();
}

void ControllerJointState::UpdateError() {
  ControllerState::UpdateError();

  error.positions[kJointIDRightWheel] = 0.0;
  error.positions[kJointIDLeftWheel] = 0.0;
}

void Convert(const ControllerState& in, const rclcpp::Time& stamp, const std::vector<std::string>& joint_names,
             control_msgs::msg::JointTrajectoryControllerState& out) {
  out.header.stamp = stamp;
  out.joint_names = joint_names;
  Convert(in.actual, out.actual);
  Convert(in.desired, out.desired);
  Convert(in.error, out.error);
}

void Convert(const State& in, trajectory_msgs::msg::JointTrajectoryPoint& out) {
  out.positions = in.positions;
  out.velocities = in.velocities;
  out.accelerations = in.accelerations;
}


StatePublisher::StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                               const std::string& topic_name,
                               const std::vector<std::string>& joint_names)
  : node_(node), joint_names_(joint_names), last_state_published_time_(node->now()), state_publish_period_(0,0) {
  const double state_publish_rate = GetPositiveParameter(node, "state_publish_rate", kDefaultStatePublishRate);
  state_publish_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);

  publisher_impl_ = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      topic_name, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);
}

void StatePublisher::Publish(const ControllerState& state, const rclcpp::Time& stamp) {
  if (stamp - last_state_published_time_ >= state_publish_period_) {
    last_state_published_time_ += state_publish_period_;
    if (publisher_ && publisher_->trylock()) {
      Convert(state, stamp, joint_names_, publisher_->msg_);
      publisher_->unlockAndPublish();
    }
  }
}

}  // namespace hsrb_base_controllers
