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
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

/*
  [](D:\Document\Hirain\Project\rolling\ros2\demos\lifecycle\src\lifecycle_service_client.md)
  [](D:\Document\Hirain\cyberdog_ros2\cyberdog_common\cyberdog_utils\include\cyberdog_utils\simple_action_server.hpp)
  [](D:\Document\Hirain\Project\rolling\ros2\demos\lifecycle\src\lifecycle_service_client.cpp)
  感觉cyber_dog 中的这个action_server 是对manager的封装和抽象
  [](D:\Document\Hirain\Project\rolling\ros-planning\navigation2\nav2_util\src\lifecycle_service_client.cpp)
  这里还有一个一样的角色
*/

/**
 * @brief 声明一个常量字符串，表示需要处理的节点名称。
 * Declare a constant string to represent the name of the node to be handled.
 */
static constexpr char const* lifecycle_node = "lc_talker";

/**
 * @brief 每个生命周期节点都有各种服务与其关联。按照惯例，我们使用以下格式：
 * <node name>/<service name>。在此演示中，我们使用 get_state 和 change_state，
 * 因此两个服务主题分别为：lc_talker/get_state 和 lc_talker/change_state。
 *
 * Every lifecycle node has various services attached to it. By convention, we use the format of
 * <node name>/<service name>. In this demo, we use get_state and change_state
 * and thus the two service topics are: lc_talker/get_state and lc_talker/change_state.
 */
static constexpr char const* node_get_state_topic = "lc_talker/get_state";
static constexpr char const* node_change_state_topic = "lc_talker/change_state";

/**
 * @brief 等待未来结果的函数模板。
 * A function template that waits for future results.
 *
 * @tparam FutureT 未来对象类型。
 * @tparam WaitTimeT 等待时间类型。
 * @param future 未来对象。
 * @param time_to_wait 等待时间。
 * @return std::future_status 状态。
 */
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;  // 获取等待结束时间点。
  std::chrono::milliseconds wait_period(100);               // 定义等待周期为 100 毫秒。
  std::future_status status = std::future_status::timeout;  // 初始化状态为超时。
  do {
    auto now = std::chrono::steady_clock::now();            // 获取当前时间点。
    auto time_left = end - now;                             // 计算剩余等待时间。
    if (time_left <= std::chrono::seconds(0)) {  // 如果剩余等待时间小于等于 0 秒，则跳出循环。
      break;
    }
    status = future.wait_for(
        (time_left < wait_period) ? time_left : wait_period);  // 等待未来对象的结果。
  } while (rclcpp::ok() && status != std::future_status::ready);
  // 只要 ROS 2 节点处于运行状态且未来对象的状态不是 ready，就继续等待。
  return status;  // 返回状态。
}

class LifecycleServiceClient : public rclcpp::Node {
public:
  explicit LifecycleServiceClient(const std::string& node_name) : Node(node_name) {}

  void init() {
    // 每个生命周期节点自动生成一对服务，允许外部与这些节点进行交互。
    // 最重要的两个是 GetState 和 ChangeState。
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ =
        this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
  }

  /// 请求节点的当前状态
  /**
   * 在此函数中，我们发送一个服务请求，请求节点 lc_talker 的当前状态。
   * 如果在给定的 time_out 内没有返回，
   * 我们将返回节点的当前状态，如果没有，则返回未知状态。
   * \param time_out 持续时间（秒），指定在返回未知状态之前等待响应的时间
   */
  unsigned int get_state(std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
          get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // 发送请求，询问 lc_talker 节点的当前状态。
    auto future_result = client_get_state_->async_send_request(request).future.share();

    // 让我们等到从节点获得答案。
    // 如果请求超时，我们将返回一个未知状态。
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
          get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // 我们有了一个成功的答案。所以让我们打印当前状态。
    if (future_result.get()) {
      RCLCPP_INFO(
          get_logger(), "Node %s has current state %s.", lifecycle_node,
          future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  /// 调用转换
  /**
   * 我们发送一个服务请求，并指示要调用 ID 为“transition”的转换。
   * 默认情况下，这些转换是
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition 指定要调用的转换的 ID
   * \param time_out 持续时间（秒），指定在返回未知状态之前等待响应的时间
   */
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
          get_logger(), "Service %s is not available.", client_change_state_->get_service_name());
      return false;
    }

    // 我们发送请求，其中包含我们要调用的转换。
    auto future_result = client_change_state_->async_send_request(request).future.share();

    // 让我们等到从节点获得答案。
    // 如果请求超时，我们将返回一个未知状态。
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
          get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // 我们有了一个答案，打印成功。
    if (future_result.get()->success) {
      RCLCPP_INFO(
          get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
          get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

/** ========= ========= ========= //
// ========= ========= ========= **/

/**
 * @brief This is a little independent script which triggers the default lifecycle of a node.
 *       It starts with configure --> activate --> deactivate --> activate --> deactivate -->
 * cleanup and finally shutdown
 * @param lc_client A shared pointer to a LifecycleServiceClient object
 */
void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client) {
  rclcpp::WallRate time_between_state_changes(0.1);  // 10s

  // configure
  {
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }
  // activate
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 调用LifecycleServiceClient的change_state函数，将状态转换为激活状态
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
  // deactivate
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 调用LifecycleServiceClient的change_state函数，将状态转换为非激活状态
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
  // activate it again
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 再次调用LifecycleServiceClient的change_state函数，将状态转换为激活状态
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
  // and deactivate it again
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 再次调用LifecycleServiceClient的change_state函数，将状态转换为非激活状态
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
  // we cleanup
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 调用LifecycleServiceClient的change_state函数，将状态转换为清理状态
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
  // and finally shutdown
  // Note: We have to be precise here on which shutdown transition id to call
  // We are currently in the unconfigured state and thus have to call
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  // 注意：我们在这里必须准确地说明要调用的关闭转换id。我们当前处于未配置状态，因此必须调用transition_unconfigured_shutdown
  {
    time_between_state_changes.sleep();  // 等待一段时间
    if (!rclcpp::ok()) {                 // 如果ROS2节点已经关闭，返回
      return;
    }
    // 调用LifecycleServiceClient的change_state函数，将状态转换为未配置关机状态
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
      return;
    }
    if (!lc_client->get_state()) {  // 获取当前状态
      return;
    }
  }
}

/**
 * @brief 唤醒执行器
 * @param future 一个 std::shared_future<void> 类型的对象，表示异步操作的结果
 * @param exec 一个 rclcpp::executors::SingleThreadedExecutor 类型的对象，表示单线程执行器
 */
void wake_executor(
    std::shared_future<void> future,  //
    rclcpp::executors::SingleThreadedExecutor& exec) {
  future.wait();                      // 等待异步操作完成
  // 当脚本完成时唤醒执行器
  // https://github.com/ros2/rclcpp/issues/1916
  exec.cancel();
}

int main(int argc, char** argv) {
  // 强制刷新 stdout 缓冲区。
  // 这确保了所有打印都正确同步，即使在启动文件中同时执行。
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // 创建生命周期服务客户端
  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  // 初始化生命周期服务客户端
  lc_client->init();

  // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor exe;
  // 将生命周期服务客户端添加到执行器中
  exe.add_node(lc_client);

  // 异步调用 callee_script 函数，并返回一个 std::shared_future<void> 对象
  std::shared_future<void> script =
      std::async(std::launch::async, std::bind(callee_script, lc_client));
  // 异步调用 wake_executor 函数，并返回一个 std::future<void> 对象
  auto wake_exec = std::async(std::launch::async, std::bind(wake_executor, script, std::ref(exe)));

  // 执行器开始执行，直到异步操作完成
  exe.spin_until_future_complete(script);
  rclcpp::shutdown();

  return 0;
}
