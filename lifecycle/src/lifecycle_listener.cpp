// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

/// LifecycleListener 类作为一个简单的监听节点
/**
 * @brief 我们订阅了两个话题：
 *        - lifecycle_chatter：来自 talker 的数据话题
 *        - lc_talker__transition_event：发布有关节点 lc_talker 状态更改的通知的话题
 */
class LifecycleListener : public rclcpp::Node {
public:
  /**
   * @brief LifecycleListener 类的构造函数。
   * @param node_name 节点名称。
   */
  explicit LifecycleListener(const std::string& node_name) : Node(node_name) {
    // 来自 lc_talker 节点的数据话题
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
        "lifecycle_chatter", 10,
        std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));

    // 通知事件话题。所有状态更改都会以 TransitionEvent
    // 的形式发布在此处，其中包括起始状态和目标状态。
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/lc_talker/transition_event", 10,
        std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1));
  }

  /**
   * @brief 数据话题的回调函数。
   * @param msg 收到的消息的共享指针。
   */
  void data_callback(std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
  }

  /**
   * @brief 通知事件话题的回调函数。
   * @param msg 收到的消息的共享指针。
   */
  void notification_callback(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg) {
    RCLCPP_INFO(
        get_logger(), "notify callback: Transition from state %s to %s",
        msg->start_state.label.c_str(), msg->goal_state.label.c_str());
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;  ///< 数据话题的订阅。
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>>
      sub_notification_;  ///< 通知事件话题的订阅。
};

/**
 * @brief 节点的主函数。
 * @param argc 命令行参数的数量。
 * @param argv 命令行参数。
 * @return 成功执行返回 0。
 */
int main(int argc, char** argv) {
  // 强制刷新 stdout 缓冲区。
  // 即使在启动文件中同时执行时，这也可以确保所有打印正确同步。
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
  rclcpp::spin(lc_listener);

  rclcpp::shutdown();

  return 0;
}
