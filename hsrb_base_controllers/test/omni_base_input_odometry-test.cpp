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
#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_input_odometry.hpp>

#include "utils.hpp"

namespace hsrb_base_controllers {

/// オドメトリを初期化すること
TEST(OmniBaseInputOdometryTest, InitOdometry) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto odom = InputOdometry(node);

  odom.InitOdometry();

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.header.stamp, rclcpp::Time(0));
  EXPECT_EQ(output.pose.pose.position.x, 0.0);
  EXPECT_EQ(output.pose.pose.position.y, 0.0);
  EXPECT_EQ(output.pose.pose.position.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 1.0);
}

/// 現在のオドメトリを取得すること
TEST(OmniBaseInputOdometryTest, GetOdometry) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto odom = InputOdometry(node);
  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node->now();
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.position.z = 3.0;
  msg.pose.pose.orientation.x = 4.0;
  msg.pose.pose.orientation.y = 5.0;
  msg.pose.pose.orientation.z = 6.0;
  msg.pose.pose.orientation.w = 7.0;

  publisher->publish(msg);
  auto timeout = TimeoutDetection(node);
  while (odom.GetOdometry().header.stamp == rclcpp::Time(0)) {
    timeout.Run();
    rclcpp::spin_some(node);
  }

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.pose.pose.position.x, 1.0);
  EXPECT_EQ(output.pose.pose.position.y, 2.0);
  EXPECT_EQ(output.pose.pose.position.z, 3.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 4.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 5.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 6.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 7.0);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
