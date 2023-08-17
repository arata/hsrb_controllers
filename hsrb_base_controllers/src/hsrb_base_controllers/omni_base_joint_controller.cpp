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
#include <hsrb_base_controllers/omni_base_joint_controller.hpp>

#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>

#include "utils.hpp"

namespace {

// 旋回軸指令速度リミット [rad/s]
constexpr double kYawVelocityLimit = 1.8;
// 車輪指令速度リミット [rad/s]
constexpr double kWheelVelocityLimit = 8.5;
/// エンコーダ速度閾値
/// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/sec程度となる)ことが発見されたため、それを弾く為の閾値
/// 異常値は4000程度なので、default値を1000とすることで異常値を弾く
/// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
// 旋回軸エンコーダ速度閾値 [rad/s]
constexpr double kYawActualVelocityThreshold = 1000.0;
// 車輪エンコーダ速度閾値 [rad/s]
constexpr double kWheelActualVelocityThreshold = 1000.0;
// 読み込むurdfロボットモデルのデフォルト名
const char* const kDefaultRobotModelName = "robot_description";
// urdfロボットモデルを読み込むノードのデフォルト名
const char* const kDefaultRobotModelNode = "robot_state_publisher";

// 関節名の取得，取得できない場合はエラー
bool GetJointName(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& parameter_name, std::string& joint_name_out) {
  joint_name_out = hsrb_base_controllers::GetParameter(node, parameter_name, "");
  if (joint_name_out.empty()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not find " << parameter_name);
    return false;
  } else {
    return true;
  }
}

// URDFを読み込む
std::string GetRobotDescription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // まずは自身のノードから読み込めるかを試す，ダメならmodel_node_nameからの読み込みを試す
  // gazeboの場合，controller_managerにrobot_descriptionをおけないので別ノードから読み込むしかない
  const std::string model_name = hsrb_base_controllers::GetParameter(node, "model_name", kDefaultRobotModelName);
  const std::string robot_description_out = hsrb_base_controllers::GetParameter(node, model_name, "");
  if (!robot_description_out.empty()) {
    return robot_description_out;
  }

  const std::string model_node_name = hsrb_base_controllers::GetParameter(node, "model_node_name", kDefaultRobotModelNode);
  const int32_t timeout = hsrb_base_controllers::GetParameter(node, "parameter_connection_timeout", 60);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, model_node_name);
  int32_t wait_for_service_count = 0;
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    ++wait_for_service_count;
    if (!rclcpp::ok()) {
      return "";
    } else if (wait_for_service_count >= timeout) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not connect parameter server of " << model_node_name);
      return "";
    }
  }
  return parameters_client->get_parameter(model_name, std::string());
}

// OmniBaseSizeを初期化する
bool InitializeOmniBaseSize(const rclcpp::Logger& logger,
                            const std::string& robot_description,
                            const std::vector<std::string>& joint_names,
                            hsrb_base_controllers::OmniBaseSize & base_size_out) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(robot_description)) {
    RCLCPP_ERROR_STREAM(logger, "Failed to parse URDF");
    return false;
  }

  const auto steer_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDSteer]);
  const auto l_wheel_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDLeftWheel]);
  const auto r_wheel_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDRightWheel]);
  if (!steer_joint || !l_wheel_joint || !r_wheel_joint) {
    RCLCPP_ERROR(logger, "Could not get joint param from urdf");
    return false;
  }

  base_size_out.tread = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.y -
                              r_wheel_joint->parent_to_joint_origin_transform.position.y);
  base_size_out.caster_offset = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.x);
  base_size_out.wheel_radius = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.z);
  return true;
}

}  // namespace

namespace hsrb_base_controllers {

// コンストラクタ，パラメータの初期化を行う
OmniBaseJointController::OmniBaseJointController(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : node_(node),
      joint_command_(Eigen::Vector3d::Zero()),
      desired_steer_pos_(0.0) {
}

// パラメータ初期化
bool OmniBaseJointController::Init() {
  joint_names_.resize(kNumOmniBaseJointIDs);
  if (!GetJointName(node_, "joints.steer", joint_names_[kJointIDSteer]) ||
      !GetJointName(node_, "joints.l_wheel", joint_names_[kJointIDLeftWheel]) ||
      !GetJointName(node_, "joints.r_wheel", joint_names_[kJointIDRightWheel])) {
    return false;
  }
  if (!InitializeOmniBaseSize(node_->get_logger(), GetRobotDescription(node_), joint_names_, omnibase_size_)) {
    return false;
  }
  twin_drive_ = std::make_shared<TwinCasterDrive>(omnibase_size_);

  velocity_limit_.yaw_limit = GetPositiveParameter(node_, "yaw_velocity_limit", kYawVelocityLimit);
  velocity_limit_.wheel_limit = GetPositiveParameter(node_, "wheel_velocity_limit", kWheelVelocityLimit);

  /// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/sec程度となる)ことが発見されたため、それを弾く為の閾値
  /// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
  actual_velocity_threshold_.yaw_limit = GetPositiveParameter(
      node_, "yaw_actual_velocity_threshold", kYawActualVelocityThreshold);
  actual_velocity_threshold_.wheel_limit = GetPositiveParameter(
      node_, "wheel_actual_velocity_threshold", kWheelActualVelocityThreshold);

  velocity_filters_.resize(kNumOmniBaseJointIDs);
  std::vector<double> coeff_a = GetParameter(node_, "wheel_command_velocity_filter.a", std::vector<double>());
  std::vector<double> coeff_b = GetParameter(node_, "wheel_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    Filter<> filter(coeff_a, coeff_b);
    velocity_filters_[kJointIDRightWheel] = filter;
    velocity_filters_[kJointIDLeftWheel] = filter;
  }
  coeff_a = GetParameter(node_, "steer_command_velocity_filter.a", std::vector<double>());
  coeff_b = GetParameter(node_, "steer_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    velocity_filters_[kJointIDSteer] = Filter<>(coeff_a, coeff_b);
  }

  return true;
}

std::vector<std::string> OmniBaseJointController::command_interface_names() const {
  std::vector<std::string> names;
  names.push_back(joint_names_[kJointIDSteer] + '/' + hardware_interface::HW_IF_POSITION);
  names.push_back(joint_names_[kJointIDRightWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(joint_names_[kJointIDLeftWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  return names;
}

std::vector<std::string> OmniBaseJointController::state_interface_names() const {
  std::vector<std::string> names;
  for (const auto& name : joint_names_) {
    names.push_back(name + '/' + hardware_interface::HW_IF_POSITION);
    names.push_back(name + '/' + hardware_interface::HW_IF_VELOCITY);
  }
  return names;
}

/// インターフェース設定
bool OmniBaseJointController::Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                       std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  for (const auto& name : joint_names_) {
    for (auto& interface : command_interfaces) {
      if (interface.get_name() == name) {
        command_interfaces_.emplace_back(std::ref(interface));
      }
    }
    for (auto& interface : state_interfaces) {
      if (interface.get_name() == name) {
        if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          current_position_interfaces_.emplace_back(std::ref(interface));
        } else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          current_velocity_interfaces_.emplace_back(std::ref(interface));
        }
      }
    }
  }
  if (command_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_position_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_velocity_interfaces_.size() != kNumOmniBaseJointIDs) {
    return false;
  }

  ResetDesiredSteerPosition();
  return true;
}


/// 軸位置を取得する
bool OmniBaseJointController::GetJointPositions(Eigen::Vector3d& positions_out) const {
  positions_out(kJointIDRightWheel) = current_position_interfaces_[kJointIDRightWheel].get().get_value();
  positions_out(kJointIDLeftWheel) = current_position_interfaces_[kJointIDLeftWheel].get().get_value();
  positions_out(kJointIDSteer) = current_position_interfaces_[kJointIDSteer].get().get_value();
  return true;
}

/// 軸速度を取得する
bool OmniBaseJointController::GetJointVelocities(Eigen::Vector3d& velocities_out) const {
  velocities_out(kJointIDRightWheel) = current_velocity_interfaces_[kJointIDRightWheel].get().get_value();
  velocities_out(kJointIDLeftWheel) = current_velocity_interfaces_[kJointIDLeftWheel].get().get_value();
  velocities_out(kJointIDSteer) = current_velocity_interfaces_[kJointIDSteer].get().get_value();
  /// ごくまれに(数時間に一回とか)、エンコーダの値が飛ぶ(4000rad/secくらいになる)ことが発見されたため、それを弾く
  /// joint速度が閾値以上なら、returnする
  /// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
  if ((fabs(velocities_out(kJointIDRightWheel)) > actual_velocity_threshold_.wheel_limit) ||
      (fabs(velocities_out(kJointIDLeftWheel)) > actual_velocity_threshold_.wheel_limit) ||
      (fabs(velocities_out(kJointIDSteer)) > actual_velocity_threshold_.yaw_limit)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Too big joint velocity! [right, left, steer]=[%lf, %lf, %lf]",
        velocities_out(kJointIDRightWheel), velocities_out(kJointIDLeftWheel), velocities_out(kJointIDSteer));
    return false;
  } else {
    return true;
  }
}

/// 指令値を計算する
void OmniBaseJointController::SetJointCommand(double period, const Eigen::Vector3d output_velocity) {
  // ロボット上体座標系の指令速度をジョイント指令速度に変換
  twin_drive_->Update(current_position_interfaces_[kJointIDSteer].get().get_value());
  joint_command_ = twin_drive_->ConvertInverse(output_velocity);

  // 旋回軸速度に対してリミットを掛ける
  if (fabs(joint_command_(kJointIDSteer)) > velocity_limit_.yaw_limit) {
    double ratio = fabs(joint_command_(kJointIDSteer)) / velocity_limit_.yaw_limit;
    joint_command_(kJointIDSteer) /= ratio;
    joint_command_(kJointIDRightWheel) /= ratio;
    joint_command_(kJointIDLeftWheel) /= ratio;
  }

  // 車輪速度に対してリミットを掛ける
  if (fabs(joint_command_(kJointIDRightWheel)) > velocity_limit_.wheel_limit ||
      fabs(joint_command_(kJointIDLeftWheel)) > velocity_limit_.wheel_limit) {
    const double ratio = std::max(fabs(joint_command_(kJointIDRightWheel)),
                                  fabs(joint_command_(kJointIDLeftWheel))) / velocity_limit_.wheel_limit;
    joint_command_(kJointIDSteer) /= ratio;
    joint_command_(kJointIDRightWheel) /= ratio;
    joint_command_(kJointIDLeftWheel) /= ratio;
  }

  // 速度指令値にフィルタをかける
  joint_command_(kJointIDRightWheel) =
      velocity_filters_[kJointIDRightWheel].update(joint_command_(kJointIDRightWheel));
  joint_command_(kJointIDLeftWheel) =
      velocity_filters_[kJointIDLeftWheel].update(joint_command_(kJointIDLeftWheel));
  joint_command_(kJointIDSteer) =
      velocity_filters_[kJointIDSteer].update(joint_command_(kJointIDSteer));

  // 車輪の指令速度をセット
  command_interfaces_[kJointIDRightWheel].get().set_value(joint_command_(kJointIDRightWheel));
  command_interfaces_[kJointIDLeftWheel].get().set_value(joint_command_(kJointIDLeftWheel));

  // 旋回軸の指令位置を更新してセット
  desired_steer_pos_ += joint_command_(kJointIDSteer) * period;
  command_interfaces_[kJointIDSteer].get().set_value(desired_steer_pos_);
}

}  // namespace hsrb_base_controllers
