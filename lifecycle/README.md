---
tip: translate by openai@2023-05-30 08:05:58
...

# Introduction

ROS 2 introduces the concept of managed nodes, also called `LifecycleNode`s. In the following tutorial, we explain the purpose of these nodes, what makes them different from regular nodes and how they comply to a lifecycle management. Managed nodes contain a state machine with a set of predefined states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state. The state machine is implemented as described at the [ROS 2 design page](http://design.ros2.org/articles/node_lifecycle.html).

> ROS 2 引入了托管节点(也称为`LifecycleNode`)的概念。在下面的教程中，我们将解释这些节点的目的，它们与常规节点的不同之处以及它们如何符合生命周期管理。管理节点包含一个具有预定义状态集的状态机。可以通过调用过渡 ID 来更改这些状态，该 ID 指示后续连续状态。状态机的实现方式如[ROS 2 Design]中所述。

Our implementation differentiates between `Primary States` and `Transition States`. Primary States are supposed to be steady states in which any node can do the respected task. On the other hand, Transition States are meant as temporary intermediate states attached to a transition. The result of these intermediate states are used to indicate whether a transition between two primary states is considered successful or not. Thus, any managed node can be in one of the following states:

> 实施区分了“主状态”和“过渡状态”。
>
> - **主状态应该是任何节点都可以执行相应任务的稳定状态**。另一方面，
> - **过渡状态被认为是与转换相关联的临时中间状态**。这些
> - **中间状态的结果用于表明两个主状态之间的转换是否被认为是成功的**。因此，任何受管理的节点可以处于以下状态中的一种：

Primary States (steady states):

> 主要状态(稳定状态)：

- unconfigured
- inactive
- active
- shutdown

Transition States (intermediate states):

> 过渡态(中间态)：

- configuring
- activating
- deactivating
- cleaningup
- shuttingdown

The possible transitions to invoke are:

> 可以调用的可能转换是：

- configure
- activate
- deactivate
- cleanup
- shutdown

For a more verbose explanation on the applied state machine, we refer to the design page which provides an in-detail explanation about each state and transition.

> 对于应用的状态机的更详细的解释，我们参考设计页面，该页面提供了关于每个状态和转换的详细解释。

# The demo

## What\'s happening

The demo is split into 3 separate applications:

> 这个演示分为 3 个独立的应用程序：

- lifecycle_talker
- lifecycle_listener
- lifecycle_service_client

The `lifecycle_talker` represents a managed node and publishes according to which state the node is in. We split the tasks of the talker node into separate pieces and execute them as follows:

> `lifecycle_talker`代表着一个**受管理的节点**，**并根据节点所处的状态发布信息**。我们将 talker 节点的任务分成几个部分，并按照以下方式执行：

1. configuring: We initialize our publisher and timer
2. activate: We activate the publisher and timer in order to enable a publishing
3. deactivate: We stop the publisher and timer
4. cleanup: We destroy the publisher and timer

> 1. 配置：初始化发布者和计时器
> 2. 激活：激活发布者和计时器以启用发布功能
> 3. 停用：停止发布者和定时器
> 4. 清理：销毁发布者和定时器

This demo shows a typical talker/listener pair of nodes. However, imagine a real scenario with attached hardware which may have a rather long booting phase, i.e. a laser or camera. One could imagine bringing up the device driver in the configuring state, start and stop only the publishing of the device\'s data in active/deactive state, and only in the cleanup/shutdown state actually shutdown the device.

> 这个演示展示了一对典型的 talker/listener 节点。但是，想象一个实际的场景，其中**有一些附加的硬件，可能有一个相当长的启动阶段，例如激光或相机**。可以想象在配置状态下启动设备驱动程序，仅在活动/非活动状态下启动设备数据的发布，仅在清理/关闭状态下关闭设备。

The `lifecycle_listener` is a simple listener which shows the characteristics of the lifecycle talker. The talker enables message publishing only in the active state and thus the listener only receives messages when the talker is in an active state.

> `lifecycle_listener`是一个简单的 listener，它显示了生命周期 talker 的特征。talker 只在活动状态下发布消息，因此，只有当 talker 处于活动状态时，listener 才能接收消息。

The `lifecycle_service_client` is a script calling different transitions on the `lifecycle_talker`. This is meant as the external user controlling the lifecycle of nodes.

> `lifecycle_service_client` 是一个脚本，用于在`lifecycle_talker`上调用不同的转换。这**是外部用户控制节点生命周期的意图**。

# Run the demo

In order to run this demo, we open three terminals and source our ROS 2 environment variables either from the binary distributions or the workspace we compiled from source.

> 为了运行这个演示，我们打开三个终端，并从二进制分发或从源编译的工作区源自 ROS 2 环境变量。

lifecycle_talker lifecycle_listener lifecycle_service_client

```bash
$ ros2 run lifecycle lifecycle_talker
$ ros2 run lifecycle lifecycle_listener
$ ros2 run lifecycle lifecycle_service_client
```

[![asciicast](https://asciinema.org/a/249049.png)](https://asciinema.org/a/249049) [![asciicast](https://asciinema.org/a/249050.png)](https://asciinema.org/a/249050) [![asciicast](https://asciinema.org/a/249051.png)](https://asciinema.org/a/249051)

点击图片查看相关的 ASCII 动画。

Alternatively, these three programs can be run together in the same terminal using the launch file:

> 另外，这三个程序可以使用启动文件在同一个终端中一起运行：

```bash
ros2 launch lifecycle lifecycle_demo_launch.py
```

If we look at the output of the `lifecycle_talker`, we notice that nothing seems to happen. This makes sense, since every node starts as `unconfigured`. The lifecycle_talker is not configured yet and in our example, no publishers and timers are created yet. The same behavior can be seen for the `lifecycle_listener`, which is less surprising given that no publishers are available at this moment. The interesting part starts with the third terminal. In there we launch our `lifecycle_service_client` which is responsible for changing the states of the `lifecycle_talker`.

> 如果我们看一下`lifecycle_talker`的输出，我们会发现似乎没有什么发生。这是有道理的，因为每个节点都以`unconfigured`状态开始。lifecycle_talker **还没有配置**，在示例中，**还没有创建发布者和计时器**。同样的行为也可以在`lifecycle_listener`中看到，这一点并不奇怪，因为此时还没有可用的发布者。有趣的部分是第三个终端。我们在那里**启动`lifecycle_service_client`，它负责更改`lifecycle_talker`的状态**。

## Triggering transition 1 (configure)

```bash
[lc_client] Transition 1 successfully triggered.
[lc_client] Node lc_talker has current state inactive.
```

Makes the lifecycle talker change its state to inactive. Inactive means that all publishers and timers are created and configured. However, the node is still not active. Therefore no messages are getting published.

> 使生命周期 talker 将其状态更改为非活动状态。非活动意味着所有发布者和定时器都已创建和配置。但是，节点仍未激活。因此，没有消息被发布。

```bash
[lc_talker] on_configure() is called.
Lifecycle publisher is currently inactive. Messages are not published.
...
```

At the same time the lifecycle listener receives a notification as it listens to every state change notification of the lifecycle talker. In fact, the listener receives two consecutive notifications. One for changing from the primary state \"unconfigured\" to \"configuring\", and a second notification changing the state from \"configuring\" to \"inactive\" (since the configuring step was successful in the talker).

> 同时，**生命周期 listener 收到通知，因为它侦听生命周期 talker 的每个状态更改通知**。事实上，listener 收到两个连续的通知。**一个是从主状态“未配置”更改为“配置”，第二个通知是从“配置”更改为“不活动”**(因为 talker 的配置步骤成功)。

> [!NOTE]
> 也就是说，listener 是一直跑着的！

```bash
[lc_listener] notify callback: Transition from state unconfigured to configuring
[lc_listener] notify callback: Transition from state configuring to inactive
```

## Triggering transition 2 (activate)

```bash
[lc_client] Transition 2 successfully triggered.
[lc_client] Node lc_talker has current state active.
```

Makes the lifecycle talker change its state to active. That means all publishers and timers are now activated and therefore the messages are now getting published.

> 使生命周期 talker 改变其状态为活动状态。这意味着所有的发布者和定时器现在都被激活，因此**消息现在正在发布**。

```bash
[lc_talker] on_activate() is called.
[lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #11]
[lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #12]
...
```

The lifecycle listener receives the same set of notifications as before. Lifecycle talker changed its state from inactive to active.

> listener 收到和以前一样的通知集合。生命周期 talker 将其状态从不活动更改为活动。

```bash
[lc_listener]: notify callback: Transition from state inactive to activating
[lc_listener]: notify callback: Transition from state activating to active
```

The difference from the earlier transition event is that our listener now also receives the actual published data.

> 这次过渡事件与之前的不同之处在于 listener 现在也接收实际发布的数据。

```bash
[lc_listener] data_callback: Lifecycle HelloWorld #11
[lc_listener] data_callback: Lifecycle HelloWorld #12
...
```

Please note that the index of the published message is already at 11. The purpose of this demo is to show that even though we call `publish` at every state of the lifecycle talker, the messages are only actually published when the state in active.

> 请注意，发布消息的索引已经达到 11。此演示的目的是显示，即使我们在**生命周期 talker 的每个状态都调用`publish`，但只有在状态处于活动状态时，消息才会实际发布**。

> [!NOTE]
> 内部在跑，但是对外的行为是受控制的

For the rest of the demo, you will see similar output as we deactivate and activate the lifecycle talker and finally shut it down.

> 在接下来的演示中，您将看到类似的输出，因为我们会关闭和激活生命周期 talker，最后关闭它。

# The demo code

## lifecycle_talker, lifecycle_listener and lifecycle_service_client

If we have a look at the code, there is one significant change for the lifecycle talker compared to a regular talker. Our node does not inherit from the regular `rclcpp::node::Node` but from `rclcpp_lifecycle::LifecycleNode`.

> 如果我们看一下代码，与常规 talker 相比，生命周期 talker 有一个重要的变化。节点不再继承常规的`rclcpp::node::Node`，而是继承`rclcpp_lifecycle::LifecycleNode`。

```bash
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
```

Every child of LifecycleNodes have a set of callbacks provided. These callbacks go along with the applied state machine attached to it. These callbacks are:

> 每个 LifecycleNodes 的子类都有一套回调函数。这些**回调函数与它所附带的状态机相一致**。这些回调函数是：

```c
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_configure(const rclcpp_lifecycle::State & previous_state)

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_activate(const rclcpp_lifecycle::State & previous_state)

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_deactivate(const rclcpp_lifecycle::State & previous_state)

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_cleanup(const rclcpp_lifecycle::State & previous_state)

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_shutdown(const rclcpp_lifecycle::State & previous_state)
```

In the following we assume that we are inside the namespace `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface` to shorten the name of the return type. All these callbacks have a positive default return value (`return CallbackReturn::SUCCESS`). This allows a lifecycle node to change its state even though no explicit callback function was overridden. There is one other callback function for error handling. Whenever a state transition throws an uncaught exception, we call `on_error`:

> 在接下来的内容中，我们假设我们在 `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface` 命名空间中，以缩短返回类型的名称。所有这些**回调函数都有一个正默认返回值(`return CallbackReturn::SUCCESS`)。这允许生命周期节点即使没有明确覆盖回调函数，也可以改变其状态**。**还有一个回调函数用于错误处理。每当状态转换抛出未捕获的异常时，我们调用`on_error`**：

```cpp
CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)
```

This gives room for executing custom error handling. Only (!) in the case that this function returns `CallbackReturn::SUCCESS`, the state machine transitions to the state `unconfigured`. By default, the `on_error` returns `CallbackReturn::FAILURE` and the state machine transitions into `finalized`.

> 这为**执行自定义错误处理**提供了空间。仅当此函数返回`CallbackReturn::SUCCESS`时，状态机才会转换到`unconfigured`状态。默认情况下，`on_error`返回`CallbackReturn::FAILURE`，状态机转换为`finalized`。

At the same time, every lifecycle node has by default 5 different communication interfaces.

> 同时，**每个生命周期节点默认有 5 种不同的通信接口**。

- Publisher `<node_name>__transition_event`: publishes in case a transition is happening.
  This allows users to get notified of transition events within the network.

  > 这允许用户在网络中获得过渡事件的通知。

- Service `<node_name>__get_state`: query about the current state of the node.
  Return either a primary or transition state.

  > 返回主状态或过渡状态。

- Service `<node_name>__change_state`: triggers a transition for the current node.
  This service call takes a transition id. The transition is fulfilled only in the case that this transition ID is a valid transition from the current state. All other cases are ignored.

  > 这个服务调用需要一个转换 ID。只有当这个转换 ID 是从当前状态有效的转换时，转换才能完成。所有其他情况都会被忽略。

- Service `<node_name>__get_available_states`: This is meant to be an introspection tool.
  It returns a list of all possible states this node can be.

  > 它返回一个包含此节点所有可能状态的列表。

- Service `<node_name>__get_available_transitions`: Same as above, meant to an introspection tool.
  It returns a list of all possible transitions this node can execute.

  > 它返回一个所有可能的此节点可以执行的转换列表。

## ros2 lifecycle command line interface

The `lifecycle_service_client` application is a fixed order script for demo purposes only. It explains the use and the API calls made for this lifecycle implementation, but may be inconvenient to use otherwise. For this reason we implemented a command line tool which lets you dynamically change states or various nodes.

> `lifecycle_service_client`应用程序只是一个固定顺序的演示脚本，用于解释此生命周期实现的**使用和 API 调用**，但在其他方面可能不太方便使用。因此，我们实现了一个命令行工具，可以**动态更改各个节点的状态**。

In the case you want to get the current state of the `lc_talker` node, you would call:

> 如果您想**获取 lc_talker 节点的当前状态**，您可以调用：

```bash
$ ros2 lifecycle get /lc_talker
unconfigured [1]
```

The next step would be to execute a state change:

> 下一步就是**执行状态变更**：

```bash
$ ros2 lifecycle set /lc_talker configure
Transitioning successful
```

In order to see what states are currently available:

> 要**查看当前可用的状态**，请：

```bash
$ ros2 lifecycle list lc_talker
- configure [1]
  Start: unconfigured
  Goal: configuring
- shutdown [5]
  Start: unconfigured
  Goal: shuttingdown
```

In this case we see that currently, the available transitions are `configure` and `shutdown`. The complete state machine can be viewed with the following command, which can be helpful for debugging or visualization purposes:

> 在这种情况下，我们看到当前可用的转换是“配置”和“关闭”。 可以使用以下命令**查看完整的状态机**，这可能有助于调试或可视化目的：

```bash
$ ros2 lifecycle list lc_talker -a
- configure [1]
  Start: unconfigured
  Goal: configuring
- transition_success [10]
  Start: configuring
  Goal: inactive
- transition_failure [11]
  Start: configuring
  Goal: unconfigured
- transition_error [12]
  Start: configuring
  Goal: errorprocessing

[...]

- transition_error [62]
  Start: errorprocessing
  Goal: finalized
```

All of the above commands are nothing more than calling the lifecycle node\'s services. With that being said, we can also call these services directly with the ros2 command line interface:

> **以上所有命令不过是调用生命周期节点的服务**。话虽如此，我们**也可以使用 ros2 命令行界面直接调用这些服务**：

```bash
$ ros2 service call /lc_talker/get_state lifecycle_msgs/GetState
requester: making request: lifecycle_msgs.srv.GetState_Request()

response:
lifecycle_msgs.srv.GetState_Response(current_state=lifecycle_msgs.msg.State(id=1, label='unconfigured'))
```

In order to trigger a transition, we call the `change_state` service

> 为了触发转换，我们调用`change_state`服务。

```bash
$ ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"
requester: making request: lifecycle_msgs.srv.ChangeState_Request(transition=lifecycle_msgs.msg.Transition(id=2, label=''))

response:
lifecycle_msgs.srv.ChangeState_Response(success=True)
```

It is slightly less convenient, because you have to know the IDs which correspond to each transition. You can find them though in the lifecycle_msgs package.

> 它稍微不太方便，因为你必须知道哪些 ID 对应每个转换。不过你可以在 lifecycle_msgs 包中找到它们。

```bash
$ ros2 interface show lifecycle_msgs/msg/Transition
```
