/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#include "hsrb_gripper_controller/hrh_gripper_apply_force_action.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <hsrb_servomotor_protocol/exxx_common.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"


namespace {

// デフォルト力誤差閾値[N]
const double kDefaultForceGoalTolerance = 0.1;
// デフォルトstall判定とする速度閾値[rad/s]
const double kDefaultStallVelocityThreshold = 0.05;
// デフォルトstall判定とする時間[s]
const double kDefaultStallTimeout = 2.0;
// デフォルトローパスフィルタゲイン
const double kDefaultForceLPFCoeff = 0.8;
// デフォルト制御ゲイン
const double kDefaultForceControlPgain = 0.1;
const double kDefaultForceControlIgain = 0.15;
const double kDefaultForceControlDgain = 0.4;
// デフォルト誤差積分蓄積最大値
const double kDefaultForceIerrMax = 0.15;

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperApplyForceCalculator::HrhGripperApplyForceCalculator()
    : hand_spring_coeff_(1.0),
      arm_length_(1.0) {}

HrhGripperApplyForceCalculator::HrhGripperApplyForceCalculator(
    const std::string& calibration_file_path, const rclcpp::Logger& logger)
    : HrhGripperApplyForceCalculator() {
  LoadForceCalibrationData(calibration_file_path, logger);
}

void HrhGripperApplyForceCalculator::LoadForceCalibrationData(const std::string& path, const rclcpp::Logger& logger) {
  try {
    YAML::Node node;
    node = YAML::LoadFile(path);

    hand_left_force_calib_data_ = node["hand_left_force"].as<std::vector<std::vector<double> > >();
    for (size_t i = 0; i < hand_left_force_calib_data_.size(); i ++) {
      if (hand_left_force_calib_data_[i].size() != 2) {
        RCLCPP_WARN(logger, "Lack of calibration data hand_left_force.");
        hand_left_force_calib_data_.clear();
      }
    }
    hand_right_force_calib_data_ = node["hand_right_force"].as<std::vector<std::vector<double> > >();
    for (size_t i = 0; i < hand_right_force_calib_data_.size(); i ++) {
      if (hand_right_force_calib_data_[i].size() != 2) {
        RCLCPP_WARN(logger, "Lack of calibration data hand_right_force.");
        hand_right_force_calib_data_.clear();
      }
    }
    hand_spring_coeff_ = node["hand_spring"].as<double>();
    arm_length_ = node["arm_length"].as<double>();
  } catch (YAML::Exception& e) {
    RCLCPP_WARN(logger, "Failed to load hand force calibration parameter YAML file.");
  }
}

double HrhGripperApplyForceCalculator::GetCurrentForce(double hand_motor_pos,
                                                       double left_spring_proximal_joint_pos,
                                                       double right_spring_proximal_joint_pos) const {
  double hand_left_force = CalculateForce(hand_motor_pos,
                                          left_spring_proximal_joint_pos,
                                          hand_left_force_calib_data_);
  double hand_right_force = CalculateForce(hand_motor_pos,
                                           right_spring_proximal_joint_pos,
                                           hand_right_force_calib_data_);
  return (hand_left_force + hand_right_force) / 2;
}

double HrhGripperApplyForceCalculator::CalculateForce(
    double hand_motor_pos,
    double spring_proximal_joint_pos,
    const std::vector<std::vector<double> >& calib_points) const {
  const double spring_proximal_joint_force = spring_proximal_joint_pos * hand_spring_coeff_ / arm_length_;

  if (!calib_points.size()) {
    return spring_proximal_joint_force;
  }
  if (hand_motor_pos < calib_points[0][0]) {
    return std::max(0.0,
                    spring_proximal_joint_force - calib_points[0][1]);
  }
  for (std::size_t i = 0; i < calib_points.size() - 1; i++) {
    if (hand_motor_pos >= calib_points[i][0] &&
        hand_motor_pos < calib_points[i + 1][0]) {
      return std::max(0.0,
                      spring_proximal_joint_force
                      - CalculateInternalForce(hand_motor_pos,
                                               calib_points[i],
                                               calib_points[i + 1]));
    }
  }
  return 0.0;
}

double HrhGripperApplyForceCalculator::CalculateInternalForce(
    double hand_motor_pos,
    const std::vector<double>& calib_p0,
    const std::vector<double>& calib_p1) const {
  // サイズチェックは読み込み時に行う
  return fabs(calib_p1[0] - calib_p0[0]) > std::numeric_limits<double>::epsilon() * fmax(1, fmax(calib_p1[0], calib_p0[0]))  // NOLINT
      ? (calib_p1[1] - calib_p0[1]) / (calib_p1[0] - calib_p0[0]) * (hand_motor_pos - calib_p0[0]) + calib_p0[1]
      : (calib_p1[1] + calib_p0[1]) / 2;
}


HrhGripperApplyForceAction::HrhGripperApplyForceAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "~/apply_force", hsrb_servomotor_protocol::kDriveModeHandPosition),
      goal_tolerance_(kDefaultForceGoalTolerance),
      stall_velocity_threshold_(kDefaultStallVelocityThreshold),
      stall_timeout_(kDefaultStallTimeout),
      force_control_pgain_(kDefaultForceControlPgain),
      force_control_igain_(kDefaultForceControlIgain),
      force_control_dgain_(kDefaultForceControlDgain),
      force_ierr_max_(kDefaultForceIerrMax),
      force_ierr_buff_(0.0),
      force_lpf_coeff_(kDefaultForceLPFCoeff),
      force_lpf_buff_(0.0) {}

/// 周期更新処理
void HrhGripperApplyForceAction::Update(const rclcpp::Time& time) {
  if (!IsActive() && *(stop_flag_buffer_.readFromRT())) {
    return;
  }
  controller_->SetComandPosition(GetCommandPos());
  CheckForSuccess(time);
}

void HrhGripperApplyForceAction::PreemptActiveGoal() {
  HrhGripperAction::PreemptActiveGoal();
  stop_flag_buffer_.writeFromNonRT(true);
}

/// アクションの初期化の実装
  bool HrhGripperApplyForceAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  command_buffer_.initRT(0.0);

  goal_tolerance_ = GetPositiveParameter(node, "force_goal_tolerance", kDefaultForceGoalTolerance);
  stall_velocity_threshold_ = GetPositiveParameter(node, "stall_velocity_threshold", kDefaultStallVelocityThreshold);
  stall_timeout_ = GetPositiveParameter(node, "stall_timeout", kDefaultStallTimeout);

  // TODO(OTA): dynamic_reconfigureで変更できるようにする
  force_control_pgain_ = GetNonNegativeParameter(node, "force_control_pgain", kDefaultForceControlPgain);
  force_control_igain_ = GetNonNegativeParameter(node, "force_control_igain", kDefaultForceControlIgain);
  force_control_dgain_ = GetNonNegativeParameter(node, "force_control_dgain", kDefaultForceControlDgain);
  force_ierr_max_ = GetNonNegativeParameter(node, "force_ierr_max", kDefaultForceIerrMax);

  force_lpf_coeff_ = GetParameter(node, "force_lpf_coeff", kDefaultForceLPFCoeff);
  if (force_lpf_coeff_ < 0.0 || force_lpf_coeff_ >= 1.0) {
    RCLCPP_WARN(node->get_logger(), "force_lpf_coeff must be above 0.0 and less than 1.0. Using default...");
    force_lpf_coeff_ = kDefaultForceLPFCoeff;
  }

  std::string path = GetParameter(node, "force_calib_data_path", std::string());
  if (path.empty()) {
    RCLCPP_WARN(node->get_logger(), "Failed to load force_calib_data_path.");
    force_calculator_ = std::make_shared<HrhGripperApplyForceCalculator>();
  } else {
    force_calculator_ = std::make_shared<HrhGripperApplyForceCalculator>(path, node->get_logger());
  }

  return true;
}

/// アクションの目標を更新する
void HrhGripperApplyForceAction::UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) {
  command_buffer_.writeFromNonRT(goal.effort);
  stop_flag_buffer_.writeFromNonRT(goal.do_control_stop);
  last_movement_time_ = node_->now();
}

double HrhGripperApplyForceAction::GetCommandPos() {
  const double ref_force = *(command_buffer_.readFromRT());
  const double current_force = force_calculator_->GetCurrentForce(controller_->GetCurrentPosition(),
                                                                  controller_->GetLeftSpringPosition(),
                                                                  controller_->GetRightSpringPosition());
  current_force_lpf_ = (1 - force_lpf_coeff_) * current_force + force_lpf_coeff_ * force_lpf_buff_;

  // フィードバックの送信
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal) {
    const auto feedback = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Feedback>();
    feedback->effort = current_force_lpf_;
    active_goal->setFeedback(feedback);
  }

  // バッファの更新
  force_ierr_buff_ += current_force_lpf_ - ref_force;
  force_ierr_buff_ = std::max(std::min(force_ierr_buff_, force_ierr_max_), -force_ierr_max_);
  double current_position = controller_->GetCurrentPosition();
  current_position += force_control_pgain_ * (current_force_lpf_ - ref_force) +
                      force_control_igain_ * force_ierr_buff_ +
                      force_control_dgain_ * (current_force_lpf_ - force_lpf_buff_);
  // バッファの更新
  force_lpf_buff_ = current_force_lpf_;

  return current_position;
}

void HrhGripperApplyForceAction::CheckForSuccess(const rclcpp::Time& time) {
  double current_velocity = controller_->GetCurrentVelocity();
  if (fabs(current_velocity) > stall_velocity_threshold_) {
    // 動いていると判定して、最後に動いた時刻を更新
    last_movement_time_ = time;
  } else if ((time - last_movement_time_).seconds() > stall_timeout_) {
    // stall状態と判定
    auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
    result->stalled = true;
    result->effort = current_force_lpf_;

    // stallして平衡状態になった時に指令値と現在値を比較
    double command = *(command_buffer_.readFromRT());
    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (!active_goal) {
      return;
    }
    if (fabs(command - current_force_lpf_) < goal_tolerance_) {
      active_goal->gh_->succeed(result);
    } else {
      active_goal->gh_->abort(result);
    }
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

}  // namespace hsrb_gripper_controller
