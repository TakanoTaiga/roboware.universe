// Copyright 2023 Hakoroboken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef P_SIM_NODE_HPP_
#define P_SIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <actuator_control_msgs/msg/mecanum_wheel.hpp>

namespace p_sim
{

class PSIMNode: public rclcpp::Node
{
public:
    explicit PSIMNode(const rclcpp::NodeOptions & node_options);

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<actuator_control_msgs::msg::MecanumWheel>::SharedPtr sub_;

    void sub_callback(const std::shared_ptr<actuator_control_msgs::msg::MecanumWheel> msg);
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    actuator_control_msgs::msg::MecanumWheel data_chache;
    actuator_control_msgs::msg::MecanumWheel odom_chache;
};
}

#endif