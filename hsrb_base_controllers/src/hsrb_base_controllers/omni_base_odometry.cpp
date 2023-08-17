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
#include <string>

#include <hsrb_base_controllers/omni_base_odometry.hpp>
#include "utils.hpp"

namespace {
// 台車オドメトリパブリッシュ周波数デフォルト値[Hz]
constexpr double kDefaultOdometryPublishRate = 30.0;
// 台車オドメトリのTFパブリッシュ周波数デフォルト値[Hz]
constexpr double kDefaultTransformPublishRate = 30.0;
}

namespace hsrb_base_controllers {

/// オドメトリ計算クラスの初期化
Odometry::Odometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : odometry_(Eigen::Vector3d::Zero()),
      velocity_(Eigen::Vector3d::Zero()) {}


/// 全方位台車ホイールオドメトリ計算クラスの初期化
BaseOdometry::BaseOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) : Odometry(node) {
  input_odom_ = std::make_shared<InputOdometry>(node);
}

/// オドメトリを更新する
void BaseOdometry::UpdateOdometry(double period,
                                  const Eigen::Vector3d& positions,
                                  const Eigen::Vector3d& velocities) {
  auto current_odom = input_odom_->GetOdometry();
  odometry_<< current_odom.pose.pose.position.x,
              current_odom.pose.pose.position.y,
              2.0 * std::atan2(current_odom.pose.pose.orientation.z,
                               current_odom.pose.pose.orientation.w);
  // レーザオドメトリ等、速度が計算できないオドメトリが入る可能性があるため
  // controller内で計算している速度を入力
  velocity_ = velocities;
}

/// オドメトリデータを初期化する
void BaseOdometry::InitOdometry() {
  input_odom_->InitOdometry();
}

/// 全方位台車ホイールオドメトリ計算クラスの初期化
WheelOdometry::WheelOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const OmniBaseSize& omnibase_size)
    : Odometry(node),
      last_odometry_published_time_(node->now()),
      last_transform_published_time_(node->now()),
      odometry_publish_period_(0, 0),
      transform_publish_period_(0, 0) {
  // 台車オドメトリに関するフレーム名を取得
  wheel_odom_frame_ = GetParameter(node, "wheel_odom_map_frame", "odom");
  wheel_base_frame_ = GetParameter(node, "wheel_odom_base_frame", "base_footprint_wheel");
  tf_prefix_ = GetParameter(node, "tf_prefix", "");
  // 全方位台車モデルのパラメータセット
  twin_drive_ = std::make_shared<TwinCasterDrive>(omnibase_size);

  // オドメトリのパブリッシャをセット
  odometry_publisher_impl_ = node->create_publisher<nav_msgs::msg::Odometry>(
      "~/wheel_odom", rclcpp::SystemDefaultsQoS());
  odometry_publisher_ = std::make_unique<OdometryPublisher>(odometry_publisher_impl_);

  transform_publisher_impl_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
  transform_publisher_ = std::make_unique<TFPublisher>(transform_publisher_impl_);
  transform_publisher_->msg_.transforms.resize(1);

  // 台車オドメトリのパブリッシュ間隔を取得
  const double odometry_publish_rate = GetPositiveParameter(node, "odometry_publish_rate", kDefaultOdometryPublishRate);
  odometry_publish_period_ = rclcpp::Duration::from_seconds(1.0 / odometry_publish_rate);
  // 台車オドメトリTFのパブリッシュ間隔を取得
  const double transform_publish_rate = GetPositiveParameter(node, "transform_publish_rate",
                                                             kDefaultTransformPublishRate);
  transform_publish_period_ = rclcpp::Duration::from_seconds(1.0 / transform_publish_rate);
}

/// ホイールオドメトリの更新
void WheelOdometry::UpdateOdometry(double period,
                                   const Eigen::Vector3d& positions,
                                   const Eigen::Vector3d& velocities) {
  // 車輪の位置、速度からオドメトリと台車速度を更新
  odometry_ = twin_drive_->UpdateOdometry(period, positions, velocities);
  twin_drive_->Update(positions[kJointIDSteer]);
  velocity_ = twin_drive_->ConvertForward(velocities);
}


/// オドメトリを発行する
void WheelOdometry::PublishOdometry(const rclcpp::Time& time) {
  const Eigen::Vector3d wheel_odometry = odometry_;
  const Eigen::Vector3d wheel_odom_velocity = velocity_;
  const Eigen::Quaterniond wheel_quat_trans(
      Eigen::AngleAxisd(wheel_odometry(kIndexBaseTheta), Eigen::Vector3d::UnitZ()));

  // オドメトリトピックを発行
  if (time - last_odometry_published_time_ >= odometry_publish_period_) {
    last_odometry_published_time_ += odometry_publish_period_;
    // ホイールオドメトリトピックを発行
    if (odometry_publisher_ && odometry_publisher_->trylock()) {
      auto& msg = odometry_publisher_->msg_;
      msg.header.stamp = time;
      // TODO(Takeshita) 厳密にはtf::resolveに相当する関数が必要
      msg.header.frame_id = tf_prefix_ + wheel_odom_frame_;
      msg.child_frame_id = tf_prefix_ + wheel_base_frame_;
      msg.pose.pose.position.x = wheel_odometry(kIndexBaseX);
      msg.pose.pose.position.y = wheel_odometry(kIndexBaseY);
      msg.pose.pose.position.z = 0.0;
      msg.pose.pose.orientation.x = wheel_quat_trans.x();
      msg.pose.pose.orientation.y = wheel_quat_trans.y();
      msg.pose.pose.orientation.z = wheel_quat_trans.z();
      msg.pose.pose.orientation.w = wheel_quat_trans.w();
      msg.twist.twist.linear.x = wheel_odom_velocity(kIndexBaseX);
      msg.twist.twist.linear.y = wheel_odom_velocity(kIndexBaseY);
      msg.twist.twist.angular.z = wheel_odom_velocity(kIndexBaseTheta);
      odometry_publisher_->unlockAndPublish();
    }
  }

  // オドメトリのtfを発行
  if (time - last_transform_published_time_ >= transform_publish_period_) {
    last_transform_published_time_ += transform_publish_period_;
    geometry_msgs::msg::Transform transform;
    transform.translation.x = wheel_odometry(kIndexBaseX);
    transform.translation.y = wheel_odometry(kIndexBaseY);
    transform.translation.z = 0.0;
    transform.rotation.x = wheel_quat_trans.x();
    transform.rotation.y = wheel_quat_trans.y();
    transform.rotation.z = wheel_quat_trans.z();
    transform.rotation.w = wheel_quat_trans.w();

    // ホイールオドメトリのtfを発行
    if (transform_publisher_ && transform_publisher_->trylock()) {
      auto& msg = transform_publisher_->msg_.transforms.front();
      msg.header.stamp = time;
      msg.header.frame_id = tf_prefix_ + wheel_odom_frame_;
      msg.child_frame_id = tf_prefix_ + wheel_base_frame_;
      msg.transform = transform;
      transform_publisher_->unlockAndPublish();
    }
  }
}

}  // namespace hsrb_base_controllers
