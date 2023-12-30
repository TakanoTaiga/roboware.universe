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

#include <memory>
#include <chrono>
#include <vector>
#include <cstdlib>
#include <random>

#include "p_sim/node.hpp"

namespace p_sim
{
    PSIMNode::PSIMNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("p_sim", node_option)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        sub_ = this->create_subscription<actuator_control_msgs::msg::MecanumWheel>(
            "/manual_mode/mecanum_wheel", 0,
            std::bind(&PSIMNode::sub_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this](){
            timer_callback();
        });
    }

    void PSIMNode::timer_callback()
    {
        std::uniform_real_distribution<> dist(0.0, 0.002);
        std::random_device rd;
        std::mt19937 gen(rd());

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = data_chache.vec_x;
        t.transform.translation.y = data_chache.vec_y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, data_chache.rotation_power);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        odom_chache.vec_x += dist(gen) - 0.001;
        odom_chache.vec_y += dist(gen) - 0.001;
        odom_chache.rotation_power += dist(gen) - 0.001;

        geometry_msgs::msg::TransformStamped t_o;
        t_o.header.stamp = this->get_clock()->now();
        t_o.header.frame_id = "map";
        t_o.child_frame_id = "odom";

        t_o.transform.translation.x = odom_chache.vec_x;
        t_o.transform.translation.y = odom_chache.vec_y;
        t_o.transform.translation.z = 0.0;

        tf2::Quaternion q_o;
        q_o.setRPY(0.0, 0.0, odom_chache.rotation_power);
        t_o.transform.rotation.x = q_o.x();
        t_o.transform.rotation.y = q_o.y();
        t_o.transform.rotation.z = q_o.z();
        t_o.transform.rotation.w = q_o.w();

        tf_broadcaster_->sendTransform(t_o);
    }

    void PSIMNode::sub_callback(const std::shared_ptr<actuator_control_msgs::msg::MecanumWheel> msg)
    {
        data_chache.vec_x += msg->vec_x * 0.1;
        data_chache.vec_y += msg->vec_y * 0.1;
        data_chache.rotation_power += msg->rotation_power * 0.1;
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p_sim::PSIMNode)