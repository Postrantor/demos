// Copyright 2023 Sony Group Corporation.
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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// This demo program shows matched event work. Matched event occurs when publisher and subscription
// establishes the connection. Class MatchedEventDetectNode output connection information of
// publisher and subscription. Class MultiSubNode is used to create/destroy subscription to
// connect/disconnect the publisher of MatchedEventDetectNode. Class MultiPubNode is used to
// created/destroy publisher to connect/disconnect the subscription of MatchedEventDetectNode.

// 此演示程序显示匹配的事件工作。当发布者和订阅建立连接时，就会发生匹配的事件。
// 匹配发布者和订阅的类匹配EventDetectNode输出连接信息。
// MultiSubNode用于创建/破坏订阅以连接/断开MatchedEventDetectNode的发布者。
// 类MultipubNode用于创建/破坏发布者，以连接/断开MatchedeventDetectNode的订阅。

/*
  这段代码是一个 MatchedEventDetectNode 类，继承自 rclcpp::Node
  类。该类用于检测发布者和订阅者之间的匹配事件，并在事件发生时输出相应的日志信息。

  在构造函数中，首先创建了发布者选项 pub_options 和订阅者选项
  sub_options，并分别设置了它们的事件回调函数。然后通过 create_publisher() 和 create_subscription()
  函数创建了发布者和订阅者。

  在事件回调函数中，根据当前连接的发布者或订阅者数量来判断是否有新的发布者或订阅者连接或断开，并输出相应的日志信息。同时，将异步操作的状态设置为完成。

  最后，通过 get_future() 函数获取一个 future 对象，用于等待异步操作的完成。
*/
class MatchedEventDetectNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，创建一个 MatchedEventDetectNode 对象。
   *
   * @param pub_topic_name 发布者话题名称。
   * @param sub_topic_name 订阅者话题名称。
   */
  explicit MatchedEventDetectNode(
      const std::string& pub_topic_name, const std::string& sub_topic_name)
      : Node("matched_event_detect_node") {
    // 创建发布者选项
    rclcpp::PublisherOptions pub_options;

    // 设置发布者事件回调函数
    pub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo& s) {
      if (any_subscription_connected_) {
        if (s.current_count == 0) {
          RCLCPP_INFO(this->get_logger(), "Last subscription is disconnected.");
          any_subscription_connected_ = false;
        } else {
          RCLCPP_INFO(
              this->get_logger(),
              "The changed number of connected subscription is %d and current number of connected"
              " subscription is %lu.",
              s.current_count_change, s.current_count);
        }
      } else {
        if (s.current_count != 0) {
          RCLCPP_INFO(this->get_logger(), "First subscription is connected.");
          any_subscription_connected_ = true;
        }
      }
      promise_->set_value(true);
    };

    // 创建发布者
    pub_ = create_publisher<std_msgs::msg::String>(pub_topic_name, 10, pub_options);

    // 创建订阅者选项
    rclcpp::SubscriptionOptions sub_options;

    // 设置订阅者事件回调函数
    sub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo& s) {
      if (any_publisher_connected_) {
        if (s.current_count == 0) {
          RCLCPP_INFO(this->get_logger(), "Last publisher is disconnected.");
          any_publisher_connected_ = false;
        } else {
          RCLCPP_INFO(
              this->get_logger(),
              "The changed number of connected publisher is %d and current number of connected"
              " publisher is %lu.",
              s.current_count_change, s.current_count);
        }
      } else {
        if (s.current_count != 0) {
          RCLCPP_INFO(this->get_logger(), "First publisher is connected.");
          any_publisher_connected_ = true;
        }
      }
      promise_->set_value(true);
    };

    // 创建订阅者
    sub_ = create_subscription<std_msgs::msg::String>(
        sub_topic_name, 10, [](std_msgs::msg::String::ConstSharedPtr) {}, sub_options);
  }

  /**
   * @brief 获取一个 future 对象，用于等待异步操作的完成。
   *
   * @return std::future<bool> 对象。
   */
  std::future<bool> get_future() {
    promise_.reset(new std::promise<bool>());
    return promise_->get_future();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;     // 发布者指针
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;  // 订阅者指针
  bool any_subscription_connected_{false};                      // 是否有订阅者连接
  bool any_publisher_connected_{false};                         // 是否有发布者连接
  std::shared_ptr<std::promise<bool>> promise_;                 // 异步操作的 promise 对象
};

/*
上述代码定义了两个类 MultiSubNode 和
MultiPubNode，分别用于创建多个订阅节点和发布节点。这两个类都继承自 rclcpp::Node 类。

在 MultiSubNode 中，构造函数 MultiSubNode(const std::string& topic_name) 接收一个字符串参数
topic_name，用于初始化订阅的 topic 名称。create_one_sub()
函数用于创建一个新的订阅节点，并返回一个指向 Subscription 的弱指针。destroy_one_sub()
函数用于销毁一个订阅节点，接收一个指向 Subscription 的弱指针作为参数。

在 MultiPubNode 中，构造函数 MultiPubNode(const std::string& topic_name) 接收一个字符串参数
topic_name，用于初始化发布的 topic 名称。create_one_pub()
函数用于创建一个新的发布节点，并返回一个指向 Publisher 的弱指针。destroy_one_pub()
函数用于销毁一个发布节点，接收一个指向 Publisher 的弱指针作为参数。
*/

/**
 * @brief MultiSubNode 类，继承自 rclcpp::Node 类，用于创建多个订阅节点
 */
class MultiSubNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，初始化 Node 名称和 topic 名称
   * @param topic_name 订阅的 topic 名称
   */
  explicit MultiSubNode(const std::string& topic_name)
      : Node("multi_sub_node"), topic_name_(topic_name) {}

  /**
   * @brief 创建一个新的订阅节点
   * @return 返回一个指向 Subscription 的弱指针
   */
  rclcpp::Subscription<std_msgs::msg::String>::WeakPtr create_one_sub(void) {
    RCLCPP_INFO(this->get_logger(), "Create a new subscription.");
    auto sub = create_subscription<std_msgs::msg::String>(
        topic_name_, 10, [](std_msgs::msg::String::ConstSharedPtr) {});

    subs_.emplace_back(sub);
    return sub;
  }

  /**
   * @brief 销毁一个订阅节点
   * @param sub 指向 Subscription 的弱指针
   */
  void destroy_one_sub(rclcpp::Subscription<std_msgs::msg::String>::WeakPtr sub) {
    auto sub_shared_ptr = sub.lock();
    if (sub_shared_ptr == nullptr) {
      return;
    }

    for (auto s = subs_.begin(); s != subs_.end(); s++) {
      if (*s == sub_shared_ptr) {
        RCLCPP_INFO(this->get_logger(), "Destroy a subscription.");
        subs_.erase(s);
        break;
      }
    }
  }

private:
  std::string topic_name_;  // 订阅的 topic 名称
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;  // 订阅节点的指针数组
};

/**
 * @brief MultiPubNode 类，继承自 rclcpp::Node 类，用于创建多个发布节点
 */
class MultiPubNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，初始化 Node 名称和 topic 名称
   * @param topic_name 发布的 topic 名称
   */
  explicit MultiPubNode(const std::string& topic_name)
      : Node("multi_pub_node"), topic_name_(topic_name) {}

  /**
   * @brief 创建一个新的发布节点
   * @return 返回一个指向 Publisher 的弱指针
   */
  rclcpp::Publisher<std_msgs::msg::String>::WeakPtr create_one_pub(void) {
    RCLCPP_INFO(this->get_logger(), "Create a new publisher.");
    auto pub = create_publisher<std_msgs::msg::String>(topic_name_, 10);
    pubs_.emplace_back(pub);

    return pub;
  }

  /**
   * @brief 销毁一个发布节点
   * @param pub 指向 Publisher 的弱指针
   */
  void destroy_one_pub(rclcpp::Publisher<std_msgs::msg::String>::WeakPtr pub) {
    auto pub_shared_ptr = pub.lock();
    if (pub_shared_ptr == nullptr) {
      return;
    }

    for (auto s = pubs_.begin(); s != pubs_.end(); s++) {
      if (*s == pub_shared_ptr) {
        RCLCPP_INFO(this->get_logger(), "Destroy a publisher.");
        pubs_.erase(s);
        break;
      }
    }
  }

private:
  std::string topic_name_;                                                 // 发布的 topic 名称
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;  // 发布节点的指针数组
};

/**
 * @brief 主函数
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 返回值为0表示程序正常结束
 */
int main(int argc, char** argv) {
  // 设置标准输出的缓冲区，使得输出立即刷新
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // 初始化 ROS2 节点
  rclcpp::init(argc, argv);

  // 定义发布和订阅的话题名称
  std::string topic_name_for_detect_pub_matched_event = "pub_topic_matched_event_detect";
  std::string topic_name_for_detect_sub_matched_event = "sub_topic_matched_event_detect";

  // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;

  // 创建 MatchedEventDetectNode 节点
  auto matched_event_detect_node = std::make_shared<MatchedEventDetectNode>(
      topic_name_for_detect_pub_matched_event, topic_name_for_detect_sub_matched_event);

  // 创建 MultiSubNode 节点
  auto multi_sub_node = std::make_shared<MultiSubNode>(topic_name_for_detect_pub_matched_event);

  // 创建 MultiPubNode 节点
  auto multi_pub_node = std::make_shared<MultiPubNode>(topic_name_for_detect_sub_matched_event);

  // 定义最大等待时间
  auto maximum_wait_time = 10s;

  // 将节点添加到执行器中
  executor.add_node(matched_event_detect_node);
  executor.add_node(multi_sub_node);
  executor.add_node(multi_pub_node);

  // 创建第一个订阅者
  auto sub1 = multi_sub_node->create_one_sub();
  // 等待 MatchedEventDetectNode 输出 "First subscription is connected."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 创建第二个订阅者
  auto sub2 = multi_sub_node->create_one_sub();
  // 等待 MatchedEventDetectNode 输出 "The changed number of connected subscription is 1 and current
  // number of connected subscription is 2."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 销毁第一个订阅者
  multi_sub_node->destroy_one_sub(sub1);
  // 等待 MatchedEventDetectNode 输出 "The changed number of connected subscription is -1 and
  // current number of connected subscription is 1."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 销毁第二个订阅者
  multi_sub_node->destroy_one_sub(sub2);
  // 等待 MatchedEventDetectNode 输出 "Last subscription is disconnected."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 创建第一个发布者
  auto pub1 = multi_pub_node->create_one_pub();
  // 等待 MatchedEventDetectNode 输出 "First publisher is connected."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 创建第二个发布者
  auto pub2 = multi_pub_node->create_one_pub();
  // 等待 MatchedEventDetectNode 输出 "The changed number of connected publisher is 1 and current
  // number of connected publisher is 2."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 销毁第一个发布者
  multi_pub_node->destroy_one_pub(pub1);
  // 等待 MatchedEventDetectNode 输出 "The changed number of connected publisher is -1 and current
  // number of connected publisher is 1."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 销毁第二个发布者
  multi_pub_node->destroy_one_pub(pub2);
  // 等待 MatchedEventDetectNode 输出 "Last publisher is disconnected."
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // 关闭 ROS2 节点
  rclcpp::shutdown();
  return 0;
}
