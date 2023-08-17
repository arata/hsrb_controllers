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
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>

#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <hsrb_base_controllers/omni_base_input_odometry.hpp>
#include <hsrb_base_controllers/twin_caster_drive.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace hsrb_base_controllers {

/// オドメトリ計算クラス
class Odometry {
 public:
  using Ptr = std::shared_ptr<Odometry>;

  explicit Odometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~Odometry() {}

  // オドメトリを更新する
  virtual void UpdateOdometry(double period,
                              const Eigen::Vector3d& positions,
                              const Eigen::Vector3d& velocities) = 0;

  // アクセサ
  Eigen::Vector3d odometry() const { return odometry_; }
  Eigen::Vector3d velocity() const { return velocity_; }

 protected:
  // 台車オドメトリ
  Eigen::Vector3d odometry_;
  // 台車速度
  Eigen::Vector3d velocity_;
};

/// 全方位台車オドメトリ計算クラス
class BaseOdometry : public Odometry {
 public:
  using Ptr = std::shared_ptr<BaseOdometry>;

  explicit BaseOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~BaseOdometry() {}

  // オドメトリを更新する
  virtual void UpdateOdometry(double period,
                              const Eigen::Vector3d& positions,
                              const Eigen::Vector3d& velocities);
  // オドメトリデータを初期化する
  void InitOdometry();

 private:
  // オドメトリ
  InputOdometry::Ptr input_odom_;
};

/// 全方位台車ホイールオドメトリ計算クラス
class WheelOdometry : public Odometry {
 public:
  using Ptr = std::shared_ptr<WheelOdometry>;

  WheelOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const OmniBaseSize& omnibase_size);
  virtual ~WheelOdometry() {}
  // オドメトリを更新する
  virtual void UpdateOdometry(double period, const Eigen::Vector3d& positions, const Eigen::Vector3d& velocities);
  // オドメトリを発行する
  virtual void PublishOdometry(const rclcpp::Time& time);

  void set_last_odometry_published_time(const rclcpp::Time& time) {
    last_odometry_published_time_ = time;
  }
  void set_last_transform_published_time(const rclcpp::Time& time) {
    last_transform_published_time_ = time;
  }

 private:
  // オドメトリに関するフレーム名
  std::string tf_prefix_;
  std::string wheel_base_frame_;
  std::string wheel_odom_frame_;
  // 全方位台車モデル
  TwinCasterDrive::Ptr twin_drive_;

  // 最後にオドメトリを発行した時間
  rclcpp::Time last_odometry_published_time_;
  // 最後にオドメトリtfを発行した時間
  rclcpp::Time last_transform_published_time_;

  // オドメトリパブリッシャ
  using OdometryPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using OdometryPublisherPtr = std::unique_ptr<OdometryPublisher>;
  OdometryPublisherPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_impl_;

  // オドメトリ発行の周期
  rclcpp::Duration odometry_publish_period_;

  // オドメトリtfパブリッシャ
  using TFPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
  using TFPublisherPtr = std::unique_ptr<TFPublisher>;
  TFPublisherPtr transform_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transform_publisher_impl_;

  // オドメトリtf発行の周期
  rclcpp::Duration transform_publish_period_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_ODOMETRY_HPP_*/
