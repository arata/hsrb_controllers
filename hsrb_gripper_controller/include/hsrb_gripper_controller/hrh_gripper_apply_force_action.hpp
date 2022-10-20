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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperApplyForceCalculator
/// @brief Hrhグリッパ指先力計算クラス
class HrhGripperApplyForceCalculator {
 public:
  using Ptr = std::shared_ptr<HrhGripperApplyForceCalculator>;
  /// コンストラクタ
  HrhGripperApplyForceCalculator();
  /// コンストラクタ
  /// @param [in] path キャリブレーションファイルパス
  HrhGripperApplyForceCalculator(const std::string& calibration_file_path, const rclcpp::Logger& logger);

  virtual ~HrhGripperApplyForceCalculator() = default;

  /// 左右の指先力を統合して算出
  // @return 現在指先力[N]
  double GetCurrentForce(double hand_motor_pos,
                         double left_spring_proximal_joint_pos,
                         double right_spring_proximal_joint_pos) const;

 private:
  /// 力制御用キャリブレーションデータの読み込み
  void LoadForceCalibrationData(const std::string& path, const rclcpp::Logger& logger);

  /// 指先力の現在値を内力と照らしあわせて算出
  /// @return 現在指先力[N]
  double CalculateForce(double hand_motor_pos,
                        double spring_proximal_joint_pos,
                        const std::vector<std::vector<double>>& calib_data) const;

  /// グリッパの内力をキャリブデータから算出
  /// @return 内力[N]
  double CalculateInternalForce(double hand_motor_pos,
                                const std::vector<double>& calib_p0,
                                const std::vector<double>& calib_p1) const;

  /// 指先力のキャリブレーションデータ[N]
  std::vector<std::vector<double>> hand_left_force_calib_data_;
  std::vector<std::vector<double>> hand_right_force_calib_data_;

  /// 指付け根関節のバネ定数[Nm/rad]
  double hand_spring_coeff_;

  /// 指の長さ[m]
  double arm_length_;
};


/// @class HrhGripperApplyForceAction
/// @brief Hrhグリッパ力制御アクションクラス
class HrhGripperApplyForceAction : public HrhGripperAction<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  /// コンストラクタ
  /// @param [in] controller 親コントローラ
  explicit HrhGripperApplyForceAction(HrhGripperController* controller);
  virtual ~HrhGripperApplyForceAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

 private:
  /// アクションの初期化の実装
  bool InitImpl(const rclcpp::Node::SharedPtr& node) override;
  /// アクションの目標を更新する
  void UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) override;

  /// ゴール力の許容誤差[N]
  double goal_tolerance_;
  /// stall判定する速度閾値[rad/s]
  double stall_velocity_threshold_;
  /// stall判定する時間[s]
  double stall_timeout_;

  /// 力制御用PIDゲイン
  double force_control_pgain_;
  double force_control_igain_;
  double force_control_dgain_;

  /// I制御の誤差積分蓄積制限値
  double force_ierr_max_;
  /// I制御の誤差積分蓄積値バッファ
  double force_ierr_buff_;

  /// 指先力のローパスフィルタ係数
  double force_lpf_coeff_;
  /// 指先力のローパスフィルタバッファ[N]
  double force_lpf_buff_;

  /// 指先力計算機
  HrhGripperApplyForceCalculator::Ptr force_calculator_;

  /// 指令値バッファ
  realtime_tools::RealtimeBuffer<double> command_buffer_;
  /// アクション継続フラグバッファ
  realtime_tools::RealtimeBuffer<bool> stop_flag_buffer_;

  /// 指先力の指令値と現在値との誤差から目標位置を算出
  /// @return 目標位置
  double GetCommandPos();
  /// ローパスフィルタを通した現在の指先力[N]
  double current_force_lpf_;

  /// アクション成否判定
  /// @param [in] time 現在時刻
  void CheckForSuccess(const rclcpp::Time& time);
  /// 最後に動作した時刻
  rclcpp::Time last_movement_time_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
