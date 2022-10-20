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
#include <hsrb_base_controllers/omni_base_control_method.hpp>

#include "utils.hpp"

namespace {
// 台車速度指定途絶判定時間のデフォルト値[s]
constexpr double kDefaultCommandTimeout = 0.5;
// 停止と判定する速度の大きさの閾値
constexpr double kStopVelocityThreshold = 0.001;
// 経路追従停止と判定する時間マージン[s]
constexpr double kStopTimeMergin = 0.2;
// 軌道追従のズレに対する制御Pゲイン
constexpr double kDefaultPGain = 1.0;

// 記述された順番が異なる２つの名前配列を並び替えるための配列を作成する
std::vector<uint32_t> MakePermutationVector(
    const std::vector<std::string>& names1,
    const std::vector<std::string>& names2) {
  // 入力配列のサイズが合わなければ終了
  if (names1.size() != names2.size()) {
    return std::vector<uint32_t>();
  }

  // 一致する名前を探し、並び替え用の配列を作成
  std::vector<uint32_t> permutation_vector(names1.size());
  for (std::vector<std::string>::const_iterator it1 = names1.begin(); it1 != names1.end(); ++it1) {
    std::vector<std::string>::const_iterator it2 = std::find(names2.begin(), names2.end(), *it1);
    if (names2.end() == it2) {
      return std::vector<uint32_t>();
    } else {
      const uint32_t t1 = std::distance(names1.begin(), it1);
      const uint32_t t2 = std::distance(names2.begin(), it2);
      permutation_vector[t1] = t2;
    }
  }
  return permutation_vector;
}
}  // namespace

namespace hsrb_base_controllers {

OmniBaseVelocityControl::OmniBaseVelocityControl(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
  // 速度指令値途絶判定時間を取得
  command_timeout_ = GetParameter(node, "command_timeout", kDefaultCommandTimeout);
  if (command_timeout_ <= 0.0) {
    RCLCPP_INFO(
      node->get_logger(),
      "command_timeout must be positive. Use default value [%lf]", kDefaultCommandTimeout);
    command_timeout_ = kDefaultCommandTimeout;
  }
  // コンストラクタでも呼んで，念の為初期化しておく
  Activate();
}

// 初期化する
void OmniBaseVelocityControl::Activate() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_velocity_ = Eigen::Vector3d::Zero();
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}

// 指令速度を取得する
Eigen::Vector3d OmniBaseVelocityControl::GetOutputVelocity() {
  std::lock_guard<std::mutex> lock(command_mutex_);

  Eigen::Vector3d output_velocity = command_velocity_;
  if (node_->get_clock()->now() - last_velocity_subscribed_time_ > rclcpp::Duration::from_seconds(command_timeout_)) {
    output_velocity = Eigen::Vector3d::Zero();
  }
  return output_velocity;
}

// 指令速度を更新する
void OmniBaseVelocityControl::UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  command_velocity_ << msg->linear.x, msg->linear.y, msg->angular.z;
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}


// コンストラクタ，パラメータの初期化を行う
OmniBaseTrajectoryControl::OmniBaseTrajectoryControl(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& cordinates) : node_(node), coordinate_names_(cordinates) {
  stop_velocity_threshold_ = GetPositiveParameter(node, "stop_velocity_threshold", kStopVelocityThreshold);
  feedback_gain_(kIndexBaseX) = GetPositiveParameter(node, "odom_x.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseY) = GetPositiveParameter(node, "odom_y.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseTheta) = GetPositiveParameter(node, "odom_t.p_gain", kDefaultPGain);
}

// 初期化
void OmniBaseTrajectoryControl::Activate() {
  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;
  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
}

// 指令速度を取得する
Eigen::Vector3d OmniBaseTrajectoryControl::GetOutputVelocity(
    const ControllerState& base_state) {
  // 指令速度に位置差分に比例する項を加え目標速度とする
  const Eigen::Vector3d desired(base_state.desired.velocities.data());
  const Eigen::Vector3d error(base_state.error.positions.data());
  Eigen::Vector3d output_velocity = desired + feedback_gain_.cwiseProduct(error);

  // 基準座標系の速度を上体座標系に変換
  const double current_yaw = base_state.actual.positions.at(kIndexBaseTheta);
  Eigen::Matrix3d robot_to_floor;
  robot_to_floor <<
      std::cos(current_yaw), std::sin(current_yaw), 0.0,
     -std::sin(current_yaw), std::cos(current_yaw), 0.0,
      0.0, 0.0, 1.0;
  output_velocity = robot_to_floor * output_velocity;

  return output_velocity;
}

  // 追従中の軌道を更新する，軌道が存在するならtrueを返す
bool OmniBaseTrajectoryControl::UpdateActiveTrajectory() {
  const auto current_msg = trajectory_ptr_->get_trajectory_msg();
  const auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    trajectory_ptr_->update(*new_msg);
  }
  if (trajectory_active_ptr_ && (*trajectory_active_ptr_)->has_trajectory_msg()) {
    if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

// 軌道追従の目標状態を取得
bool OmniBaseTrajectoryControl::SampleDesiredState(
    const rclcpp::Time& time,
    const std::vector<double>& current_positions,
    const std::vector<double>& current_velocities,
    trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
    bool& before_last_point,
    double& time_from_point) {
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    trajectory_msgs::msg::JointTrajectoryPoint current_state;
    current_state.positions = current_positions;
    current_state.velocities = current_velocities;
    (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, current_state);
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  const bool is_ok = (*trajectory_active_ptr_)->sample(time, desired_state, start_segment_it, end_segment_it);
  if (is_ok) {
    before_last_point = end_segment_it != (*trajectory_active_ptr_)->end();
    const rclcpp::Time start_stamp = (*trajectory_active_ptr_)->get_trajectory_start_time();
    const rclcpp::Time end_stamp = start_stamp + start_segment_it->time_from_start;
    time_from_point = time.seconds() - end_stamp.seconds();
  }
  return is_ok;
}

// 入力軌道指令を検証する
bool OmniBaseTrajectoryControl::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const {
  // ジョイント名が合っていなければ無効
  if (trajectory.joint_names.size() != coordinate_names_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_size mismatch.");
    return false;
  }
  for (const auto& joint_name : trajectory.joint_names) {
    if (std::find(coordinate_names_.begin(), coordinate_names_.end(), joint_name) == coordinate_names_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_name mismatch.");
      return false;
    }
  }

  // JointTrajectoryPoint毎に中身が有効であるかチェック
  double last_time = -std::numeric_limits<double>::max();
  for (const auto& point : trajectory.points) {
    // positionの要素数がジョイント数と一致していなければ無効
    if (point.positions.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's position size is wrong.");
      return false;
    }
    // velocityの要素数がジョイント数と一致していなければ無効
    // velocityが空ならば許容
    if (!point.velocities.empty() && point.velocities.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's velocity size is wrong.");
      return false;
    }
    // accelerationの要素数がジョイント数と一致していなければ無効
    // accelerationが空ならば許容
    if (!point.accelerations.empty() && point.accelerations.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's acceleration size is wrong.");
      return false;
    }
    // time_from_startが逆行していれば無効
    double time_from_start = static_cast<rclcpp::Duration>(point.time_from_start).seconds();
    if (time_from_start - last_time <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's time_from_start is going reverse.");
      return false;
    }
    last_time = time_from_start;
  }
  return true;
}

// 追従軌道を更新する
void OmniBaseTrajectoryControl::AcceptTrajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
    const Eigen::Vector3d& base_positions) {
  // 入力軌道の点が無い場合は停止させる
  if (trajectory->points.empty()) {
    ResetCurrentTrajectory();
    return;
  }

  // 入力msgの並べ替え配列の作成を行う
  const std::vector<std::string> trajectory_joint_names = trajectory->joint_names;
  const std::vector<uint32_t> permutation_vector = MakePermutationVector(coordinate_names_, trajectory_joint_names);

  // 軸の名前の記述順を考慮した軌道の作成を行う
  trajectory_msgs::msg::JointTrajectory permutated_trajectory;
  permutated_trajectory.header = trajectory->header;
  permutated_trajectory.joint_names = trajectory->joint_names;
  for (const auto& input_point : trajectory->points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto index : permutation_vector) {
      if (input_point.positions.size() == permutation_vector.size()) {
        point.positions.push_back(input_point.positions.at(index));
      }
      if (input_point.velocities.size() == permutation_vector.size()) {
        point.velocities.push_back(input_point.velocities.at(index));
      }
      if (input_point.accelerations.size() == permutation_vector.size()) {
        point.accelerations.push_back(input_point.accelerations.at(index));
      }
    }
    point.time_from_start = input_point.time_from_start;
    permutated_trajectory.points.push_back(point);
  }
  // 旋回軸の補正
  double prev_position = base_positions[kIndexBaseTheta];
  for (auto& point : permutated_trajectory.points) {
    double diff = angles::shortest_angular_distance(prev_position, point.positions[kIndexBaseTheta]);
    point.positions[kIndexBaseTheta] = prev_position + diff;
    prev_position = point.positions[kIndexBaseTheta];
  }

  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(permutated_trajectory));
}

// 条件を満たしていたら軌道追従を終了させる
void OmniBaseTrajectoryControl::TerminateControl(const rclcpp::Time& time, const ControllerState& base_state) {
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg()) {
    return;
  }
  if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    return;
  }

  const double time_from_start = (time - (*trajectory_active_ptr_)->get_trajectory_start_time()).seconds();
  const Eigen::Vector3d current_velocity(base_state.actual.velocities.data());
  // 軌道追従の追従予定時間を過ぎている，かつ現在静止状態の時，軌道追従を終了させる
  const rclcpp::Duration command_trajectory_period = (--((*trajectory_active_ptr_)->end()))->time_from_start;
  if ((time_from_start > command_trajectory_period.seconds() + kStopTimeMergin) &&
      (current_velocity.norm() < stop_velocity_threshold_)) {
    ResetCurrentTrajectory();
  }
}

// 現在追従中の軌道をリセットする
void OmniBaseTrajectoryControl::ResetCurrentTrajectory() {
  trajectory_msgs::msg::JointTrajectory empty_msg;
  empty_msg.header.stamp = rclcpp::Time(0);
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg));
}

}  // namespace hsrb_base_controllers
