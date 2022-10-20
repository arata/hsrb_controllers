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
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hsrb_base_controllers/filter.hpp>
#include <hsrb_base_controllers/twin_caster_drive.hpp>

namespace hsrb_base_controllers {

/// 旋回軸速度と車輪速度のリミット
struct VelocityLimit {
  // 旋回軸速度リミット[rad/s]
  double yaw_limit;
  // 車輪速度リミット[rad/s]
  double wheel_limit;
};

/// Jointコントローラクラス
class OmniBaseJointController {
 public:
  using Ptr = std::shared_ptr<OmniBaseJointController>;

  explicit OmniBaseJointController(const rclcpp::Node::SharedPtr& node);
  ~OmniBaseJointController() = default;

  // パラメータ初期化
  bool Init();

  // インターフェース設定
  std::vector<std::string> command_interface_names() const;
  std::vector<std::string> state_interface_names() const;
  bool Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);
  // 指令値を計算する
  void SetJointCommand(double period, const Eigen::Vector3d output_velocity);
  // 軸位置を取得する
  bool GetJointPositions(Eigen::Vector3d& positions_out) const;
  // 軸速度を取得する
  bool GetJointVelocities(Eigen::Vector3d& velocities_out) const;
  // 旋回軸の目標位置をリセットする
  void ResetDesiredSteerPosition() {
    desired_steer_pos_ = current_position_interfaces_[kJointIDSteer].get().get_value();
  }

  // アクセサ
  Eigen::Vector3d joint_command() const { return joint_command_; }
  OmniBaseSize omnibase_size() const { return omnibase_size_; }
  std::string l_wheel_joint_name() const { return joint_names_[kJointIDLeftWheel]; }
  std::string r_wheel_joint_name() const { return joint_names_[kJointIDRightWheel]; }
  std::string steer_joint_name() const { return joint_names_[kJointIDSteer]; }
  std::vector<std::string> joint_names() const { return joint_names_; }
  double desired_steer_pos() const { return desired_steer_pos_; }

 private:
  // コントローラのノードハンドル
  rclcpp::Node::SharedPtr node_;
  // Joint指令値
  Eigen::Vector3d joint_command_;
  // 各軸の名前
  std::vector<std::string> joint_names_;

  // 台車の各軸のハンドラ
  template <typename T>
  using InterfaceReferences = std::vector<std::reference_wrapper<T>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> command_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_position_interfaces_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> current_velocity_interfaces_;

  // 台車の寸法情報
  OmniBaseSize omnibase_size_;
  // 旋回軸の目標位置
  double desired_steer_pos_;
  // 台車の運動学モデル
  TwinCasterDrive::Ptr twin_drive_;
  // 指令速度リミット
  VelocityLimit velocity_limit_;
  // エンコーダ値速度閾値
  VelocityLimit actual_velocity_threshold_;
  // 速度フィルタ
  std::vector<Filter<> > velocity_filters_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_JOINT_CONTROLLER_HPP_*/
