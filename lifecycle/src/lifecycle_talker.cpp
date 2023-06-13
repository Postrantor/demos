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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// LifecycleTalker 继承自 rclcpp_lifecycle::LifecycleNode
/**
 * LifecycleTalker 不像常规的 "talker" 节点继承自 node，而是继承自 lifecyclenode。
 * 这带来了一组回调函数，这些回调函数根据节点的当前状态被调用。
 * (每个生命周期节点默认有 5 种不同的通信接口)
 * 每个生命周期节点都有一组附加到它上面的服务，这使得它可以从外部进行控制并调用状态更改。
 * Beta1 中可用的服务：
 *   - <node_name>__get_state
 *   - <node_name>__change_state
 *   - <node_name>__get_available_states
 *   - <node_name>__get_available_transitions
 * 另外，还创建了一个用于状态更改通知的发布者：
 *   - <node_name>__transition_event
 */
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
public:
  /// LifecycleTalker 构造函数
  /**
   * lifecycletalker/lifecyclenode 构造函数与常规节点具有相同的参数。
   */
  explicit LifecycleTalker(
      const std::string &node_name,  //
      bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(
            node_name,  //
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

  /// walltimer 的回调函数，用于发布消息。
  /**
   * walltimer 的回调函数。此函数由计时器调用并执行发布操作。
   * 对于此演示，要求节点提供其当前状态。
   * 如果生命周期发布者未激活，仍然调用 publish，但通信被阻塞，因此实际上不传输任何消息。
   */
  void publish() {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

    // 为演示目的打印当前状态
    if (!pub_->is_activated()) {
      RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
      RCLCPP_INFO(
          get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    }

    // 独立于当前状态调用生命周期发布者的 publish 函数。
    // 只有当发布者处于活动状态时，消息传输才可用，并且消息实际上会被发布。
    pub_->publish(std::move(msg));
  }

  /// 状态配置(configure)的转换回调函数
  /**
   * 当生命周期节点进入“配置”状态时，将调用 on_configure 回调函数。
   * 根据此函数的返回值，状态机将调用到“inactive”状态或保持在“unconfigured”状态。
   * TRANSITION_CALLBACK_SUCCESS 转换到“inactive”
   * TRANSITION_CALLBACK_FAILURE 转换到“unconfigured”
   * TRANSITION_CALLBACK_ERROR 或任何未捕获的异常转换到“errorprocessing”
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  //
  on_configure(const rclcpp_lifecycle::State &) {
    // 此回调函数应用于初始化和配置目的。
    // 因此，初始化和配置的发布者和计时器。
    // 生命周期节点 API
    // 返回生命周期组件，例如生命周期发布者。这些实体遵守生命周期并可以符合节点的当前状态。 在 beta
    // 版本中，只有一个生命周期发布者可用。
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&LifecycleTalker::publish, this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    // 返回成功，因此调用转换到下一个步骤：“inactive”。
    // 如果返回TRANSITION_CALLBACK_FAILURE，则状态机将保持在“未配置”状态。
    // 在此回调中出现TRANSITION_CALLBACK_ERROR或任何抛出的异常的情况下，状态机会转换到状态“errorprocessing”。
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// 状态激活(active)的转换回调
  /**
   * 当生命周期节点进入“activating”状态时，将调用on_activate回调。
   * 根据此函数的返回值，状态机将调用到“active”状态或保持在“inactive”状态。
   * TRANSITION_CALLBACK_SUCCESS 转换到 "active"
   * TRANSITION_CALLBACK_FAILURE 转换到 "inactive"
   * TRANSITION_CALLBACK_ERROR 或任何未捕获的异常转换到 "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  //
  on_activate(const rclcpp_lifecycle::State &state) {
    // 父类方法会自动管理实体（当前为LifecyclePublisher）上的转换。
    // pub_->on_activate()也可以在此手动调用。
    // 覆盖此方法是可选的，很多时候默认就足够了。
    LifecycleNode::on_activate(state);
    // impl_->on_activate(); // 默认提供的
    // pub_->on_activate(); // 也可以在此手动调用

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    // 让睡眠2秒钟。
    // 模拟在激活阶段执行重要工作。
    std::this_thread::sleep_for(2s);

    // 返回成功，因此调用转换到下一个步骤：“active”。
    // 如果返回TRANSITION_CALLBACK_FAILURE，则状态机将保持在“inactive”状态。
    // 在此回调中出现TRANSITION_CALLBACK_ERROR或任何抛出的异常的情况下，状态机会转换到状态“errorprocessing”。
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// 状态去激活(deactivate)的转换回调
  /**
   * 当生命周期节点进入“deactivating”状态时，将调用on_deactivate回调。
   * 根据此函数的返回值，状态机将调用到“inactive”状态或保持在“active”状态。
   * TRANSITION_CALLBACK_SUCCESS 转换到 "inactive"
   * TRANSITION_CALLBACK_FAILURE 转换到 "active"
   * TRANSITION_CALLBACK_ERROR 或任何未捕获的异常转换到 "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  //
  on_deactivate(const rclcpp_lifecycle::State &state) {
    // 父类方法会自动管理实体（当前为LifecyclePublisher）上的转换。
    // pub_->on_deactivate()也可以在此手动调用。
    // 覆盖此方法是可选的，很多时候默认就足够了。
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    // 返回成功，因此调用转换到下一个步骤：“inactive”。
    // 如果返回TRANSITION_CALLBACK_FAILURE，则状态机将保持在“active”状态。
    // 在此回调中出现TRANSITION_CALLBACK_ERROR或任何抛出的异常的情况下，状态机会转换到状态“errorprocessing”。
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// 状态清理(cleanup)的转换回调
  /**
   * 当生命周期节点进入“cleaningup”状态时，将调用on_cleanup回调。
   * 根据此函数的返回值，状态机将调用到“unconfigured”状态或保持在“inactive”状态。
   * TRANSITION_CALLBACK_SUCCESS 转换到 "unconfigured"
   * TRANSITION_CALLBACK_FAILURE 转换到 "inactive"
   * TRANSITION_CALLBACK_ERROR 或任何未捕获的异常转换到 "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  //
  on_cleanup(const rclcpp_lifecycle::State &) {
    // 在的清理阶段，释放计时器和发布者的共享指针。这些实体不再可用，的节点是“干净”的。
    timer_.reset();
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    // 返回成功，因此调用转换到下一个步骤：“unconfigured”。
    // 如果返回TRANSITION_CALLBACK_FAILURE，则状态机将保持在“inactive”状态。
    // 在此回调中出现TRANSITION_CALLBACK_ERROR或任何抛出的异常的情况下，状态机会转换到状态“errorprocessing”。
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /// 状态转换回调函数，用于关闭状态(shutdown)
  /**
   * 当生命周期节点进入“正在关闭”状态时，将调用on_shutdown回调函数。
   * 根据此函数的返回值，状态机将执行从当前状态到“已完成”状态的转换或保持在当前状态。
   * TRANSITION_CALLBACK_SUCCESS 转换到“已完成”状态
   * TRANSITION_CALLBACK_FAILURE 保持当前状态
   * TRANSITION_CALLBACK_ERROR 或任何未捕获的异常转换到“错误处理”状态
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn  //
  on_shutdown(const rclcpp_lifecycle::State &state) {
    // 在关闭阶段，释放计时器和发布者的共享指针。这些实体不再可用，的节点是“干净的”。
    timer_.reset();
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(
        get_name(), "on shutdown is called from state %s.", state.label().c_str());

    // 返回成功并因此调用转换到下一步：“已完成”。
    // 如果返回TRANSITION_CALLBACK_FAILURE，则状态机将保持在当前状态。
    // 如果在此回调中返回TRANSITION_CALLBACK_ERROR或引发任何异常，则状态机将转换到状态“错误处理”。
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  // 拥有一个生命周期发布者的实例。这个生命周期发布者可以根据生命周期节点所处的状态被激活或停用。
  // 默认情况下，生命周期发布者在创建时处于非活动状态，并且必须被激活才能将消息发布到ROS世界中。
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

  // 拥有一个计时器的实例，它会定期触发发布函数。
  // 在beta版本中，这是一个常规计时器。在将来的版本中，将创建一个生命周期计时器，该计时器遵循与生命周期发布者相同的生命周期管理。
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

/**
 * 生命周期节点与普通节点具有相同的节点 API。
 * 这意味着可以生成一个节点，为其命名并将其添加到执行器中。
 */
int main(int argc, char *argv[]) {
  // 强制刷新 stdout 缓冲区。
  // 这确保了所有打印的正确同步，即使在启动文件中同时执行也是如此。
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<LifecycleTalker> lc_node = std::make_shared<LifecycleTalker>("lc_talker");
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
