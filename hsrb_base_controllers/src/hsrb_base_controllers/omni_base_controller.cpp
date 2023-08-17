/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
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
#include <hsrb_base_controllers/omni_base_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "utils.hpp"

namespace hsrb_base_controllers {

controller_interface::return_type OmniBaseController::init(const std::string& controller_name) {
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  if (InitImpl()) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

bool OmniBaseController::InitImpl() {
  // 制御する台車の座標軸名を取得
  const auto base_coordinate_names = GetParameter<std::vector<std::string>>(get_node(), "base_coordinates", {});
  if (base_coordinate_names.size() != kNumBaseCoordinateIDs) {
    RCLCPP_ERROR(get_node()->get_logger(), "The size of joints must be three.");
    return false;
  }
  // default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(base_coordinate_names);

  auto param_listener_ = std::make_shared<joint_trajectory_controller::ParamListener>(get_node());
  auto params_ = param_listener_->get_params();
  default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(params_);

  // 速度のサブスクライバをセット
  velocity_subscriber_ = std::make_shared<CommandVelocitySubscriber>(get_node(), this);
  // 軌道のサブスクライバをセット
  trajectory_subscriber_ = std::make_shared<CommandTrajectorySubscriber>(get_node(), this);
  // アクションサーバ立ちあげ
  trajectory_action_ = std::make_shared<TrajectoryActionServer>(get_node(), base_coordinate_names, this);

  // Jointコントローラクラス
  joint_controller_ = std::make_shared<OmniBaseJointController>(get_node());
  if (!joint_controller_->Init()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Initializing OmniBaseJointController is failed.");
    return false;
  }

  // ホイールオドメトリのパブリッシャをセット
  wheel_odometry_ = std::make_shared<WheelOdometry>(get_node(), joint_controller_->omnibase_size());
  // 台車オドメトリのパブリッシャをセット
  base_odometry_ = std::make_shared<BaseOdometry>(get_node());

  // コントローラを生成
  velocity_control_ = std::make_shared<OmniBaseVelocityControl>(get_node());
  trajectory_control_ = std::make_shared<OmniBaseTrajectoryControl>(get_node(), base_coordinate_names);

  // 内部関節状態のパブリッシャをセット
  base_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/state", base_coordinate_names);
  joint_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/internal_state", joint_controller_->joint_names());

  return true;
}

controller_interface::InterfaceConfiguration OmniBaseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->command_interface_names();
  return conf;
}

controller_interface::InterfaceConfiguration OmniBaseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->state_interface_names();
  return conf;
}

controller_interface::return_type OmniBaseController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // 台車の各軸の位置・速度取得
  Eigen::Vector3d joint_positions;
  Eigen::Vector3d joint_velocities;
  if (!joint_controller_->GetJointPositions(joint_positions) ||
      !joint_controller_->GetJointVelocities(joint_velocities)) {
    return controller_interface::return_type::ERROR;
  }
  const auto current_time = get_node()->get_clock()->now();
  const double period_ = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  // オドメトリの更新
  wheel_odometry_->UpdateOdometry(period_, joint_positions, joint_velocities);
  base_odometry_->UpdateOdometry(period_, wheel_odometry_->odometry(), wheel_odometry_->velocity());

  // 台車の追従状態をアップデート
  ControllerBaseState base_state(base_odometry_->odometry(), base_odometry_->velocity());

  Eigen::Vector3d output_velocity = Eigen::Vector3d::Zero();
  if (trajectory_control_->UpdateActiveTrajectory()) {
    // 軌道追従
    trajectory_msgs::msg::JointTrajectoryPoint desired_state;
    bool before_last_point;
    double time_from_point;
    const auto is_valid = trajectory_control_->SampleDesiredState(
        current_time, base_state.actual.positions, base_state.actual.velocities,
        desired_state, before_last_point, time_from_point);
    if (is_valid) {
      base_state = ControllerBaseState(
          base_odometry_->odometry(), base_odometry_->velocity(),
          desired_state.positions, desired_state.velocities, desired_state.accelerations);
      output_velocity = trajectory_control_->GetOutputVelocity(base_state);

      auto status = CheckTorelances(base_state, before_last_point, time_from_point);
      if (status <= 0) {
        trajectory_action_->UpdateActionResult(status);
        trajectory_control_->ResetCurrentTrajectory();
      } else {
        trajectory_action_->SetFeedback(base_state, current_time);
      }
    }
  } else {
    // 速度追従
    output_velocity = velocity_control_->GetOutputVelocity();
  }
  joint_controller_->SetJointCommand(period_, output_velocity);

  // 台車の現状態(指令値、現在地、差分)をパブリッシュ
  const ControllerJointState joint_state(joint_positions,
                                         joint_velocities,
                                         joint_controller_->desired_steer_pos(),
                                         joint_controller_->joint_command());
  joint_state_publisher_->Publish(joint_state, current_time);
  base_state_publisher_->Publish(base_state, current_time);

  // ホイールオドメトリ、TFをパブリッシュ
  wheel_odometry_->PublishOdometry(current_time);

  trajectory_control_->TerminateControl(current_time, base_state);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OmniBaseController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  // 特になにもしない，initとactivateに全ての処理がある
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OmniBaseController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  if (!joint_controller_->Activate(command_interfaces_, state_interfaces_)) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  // JointTrajectoryControllerに合わせて，ここでActivate
  velocity_control_->Activate();
  trajectory_control_->Activate();

  const auto current_time = get_node()->get_clock()->now();
  wheel_odometry_->set_last_odometry_published_time(current_time);
  wheel_odometry_->set_last_transform_published_time(current_time);
  joint_state_publisher_->set_last_state_published_time(current_time);
  base_state_publisher_->set_last_state_published_time(current_time);
  last_update_time_ = current_time;

  // 立ち上げ時にオドメトリを初期化
  base_odometry_->InitOdometry();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OmniBaseController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  // コントローラ立ち下げ時は現在のアクションゴールをリセット
  trajectory_action_->PreemptActiveGoal();
  // 指令速度をクリア
  const auto zero_velocity = std::make_shared<geometry_msgs::msg::Twist>();
  velocity_control_->UpdateCommandVelocity(zero_velocity);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool OmniBaseController::IsAcceptable() {
  return get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

void OmniBaseController::UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  velocity_control_->UpdateCommandVelocity(msg);
}

bool OmniBaseController::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  return trajectory_control_->ValidateTrajectory(trajectory);
}

void OmniBaseController::UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) {
  active_tolerances_ = default_tolerances_;
  trajectory_control_->AcceptTrajectory(trajectory, base_odometry_->odometry());
}

void OmniBaseController::ResetTrajectory() {
  trajectory_control_->ResetCurrentTrajectory();
}

int32_t OmniBaseController::CheckTorelances(const ControllerBaseState& state,
                                            bool before_last_point,
                                            double time_from_trajectory_end) {
  trajectory_msgs::msg::JointTrajectoryPoint error;
  Convert(state.error, error);

  if (before_last_point) {
    // 軌道追従中なので，経路がズレていないかチェックするだけ
    for (uint32_t i = 0; i < active_tolerances_.state_tolerance.size(); ++i) {
      if (!joint_trajectory_controller::check_state_tolerance_per_joint(error, i,
                                                                        active_tolerances_.state_tolerance[i])) {
        return control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      }
    }
  } else {
    // ゴールしたかのチェック，時間内なら何もせずに待つ
    bool abort = false;
    for (uint32_t i = 0; i < active_tolerances_.goal_state_tolerance.size(); ++i) {
      if (!joint_trajectory_controller::check_state_tolerance_per_joint(
              error, i, active_tolerances_.goal_state_tolerance[i])) {
        abort = true;
        break;
      }
    }
    if (!abort) {
      return control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    } else if (active_tolerances_.goal_time_tolerance != 0.0) {
      // 0.0との!=は危険だが，デフォルト値が0.0なので，これでいく
      if (time_from_trajectory_end > active_tolerances_.goal_time_tolerance) {
        return control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      }
    }
  }
  // 定義されているエラーコードは0以下なので，どれでもないを示すために正の数を返す
  return 1;
}

}  // namespace hsrb_base_controllers

PLUGINLIB_EXPORT_CLASS(hsrb_base_controllers::OmniBaseController,
                       controller_interface::ControllerInterface);
