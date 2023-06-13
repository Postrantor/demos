## 这是 ros2 中使用 lifecycle 的示例程序的一部分，主要是 main() 函数中的逻辑

请从语法的角度上详细解释一下，这里使用到的 `std::shared_future` 以及 `std::async` `std::launch:;async`
我对上面这些 stl 库不太清楚是什么作用

```cpp
int main(int argc, char** argv) {
  // 强制刷新 stdout 缓冲区。
  // 这确保了所有打印都正确同步，即使在启动文件中同时执行。
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // 初始化 ROS
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

  // 关闭 ROS
  rclcpp::shutdown();

  return 0;
}
```

在这个代码片段中，`main()` 函数主要实现了以下功能：

1. 初始化 ROS。
2. 创建一个生命周期服务客户端，并初始化。
3. 创建一个单线程执行器，并将生命周期服务客户端添加到执行器中。
4. 异步调用两个函数：`callee_script()` 和 `wake_executor()`。
5. 执行器开始执行，直到异步操作完成。
6. 关闭 ROS。

这里涉及到的 `std::shared_future`, `std::async` 和 `std::launch::async` 主要是用于异步操作。下面分别解释它们的作用：

- `std::shared_future`: `std::shared_future` 是一个可共享的、非唯一拥有的 future，可以被多个对象共享。`std::shared_future` 提供了一种机制，用于在多个线程之间同步对某个值的访问。当与 `std::async` 一起使用时，它可以让您在多个线程中共享对异步操作的结果的访问。

- `std::async`: `std::async` 是一个模板函数，用于实现异步操作。它接受一个函数（可以是普通函数、lambda 表达式或成员函数）和该函数的参数，**然后在一个单独的线程中执行该函数**。`std::async` 返回一个 `std::future` 对象，表示异步操作的结果。通过调用 `std::future::get()` 或 `std::future::wait()`，可以等待异步操作完成并获取结果。

- `std::launch::async`: `std::launch::async` 是一个枚举值，用于指定 `std::async` 的启动策略。当将 `std::launch::async` 作为 `std::async` 的第一个参数时，它**表示在一个新的线程中立即启动异步操作**。

在这个示例中，代码使用 `std::async` 异步调用了两个函数：`callee_script(lc_client)` 和 `wake_executor(script, exe)`。这两个异步操作将在不同的线程中并发执行。同时，`std::shared_future` 被用来在这两个异步操作之间共享 `callee_script` 函数的结果。

最后，`exe.spin_until_future_complete(script)` 使执行器开始执行，并等待 `callee_script` 函数的异步操作完成。

## 可以对这几个使用到的 stl 库函数分别给出合理的示例程序，详细解释其中的用法，尽可能专业的示例程序。

当然可以。下面是针对`std::shared_future`, `std::async` 和 `std::launch::async`的示例程序，以及对它们用法的详细解释。

1. `std::shared_future`

```cpp
#include <iostream>
#include <future>
#include <thread>

int calculate_sum(int a, int b) {
  std::this_thread::sleep_for(std::chrono::seconds(2)); // 模拟耗时任务
  return a + b;
}

int main() {
  std::packaged_task<int(int, int)> task(calculate_sum);
  std::shared_future<int> shared_result = task.get_future().share();

  std::thread t1([&task] { task(5, 7); });

  std::future<int> fut1 = std::async(std::launch::async, [&shared_result] {
    return shared_result.get() * 2;
  });

  std::future<int> fut2 = std::async(std::launch::async, [&shared_result] {
    return shared_result.get() * 3;
  });

  std::cout << "fut1: " << fut1.get() << std::endl;
  std::cout << "fut2: " << fut2.get() << std::endl;

  t1.join();
  return 0;
}
```

在这个示例中，我们创建了一个`std::packaged_task`对象，它包含了一个计算两个整数之和的函数`calculate_sum`。我们从`packaged_task`对象中获取一个`std::shared_future`对象，用于在多个异步操作中共享对计算结果的访问。

我们创建了两个异步操作，分别在不同线程中执行。它们都等待`shared_result`的结果，然后分别将结果乘以 2 和 3。最后，我们打印这两个异步操作的结果。

2. `std::async`

```cpp
#include <iostream>
#include <future>
#include <string>

std::string reverse_string(const std::string& input) {
  std::this_thread::sleep_for(std::chrono::seconds(2)); // 模拟耗时任务
  return std::string(input.rbegin(), input.rend());
}

int main() {
  std::future<std::string> result = std::async(reverse_string, "Hello, World!");

  // 在此期间，可以执行其他任务

  std::string reversed = result.get();
  std::cout << "Reversed string: " << reversed << std::endl;

  return 0;
}
```

在这个示例中，我们使用`std::async`异步执行一个字符串反转函数`reverse_string`。`std::async`会立即返回一个`std::future`对象，表示异步操作的结果。我们可以在异步操作进行期间执行其他任务。通过调用`result.get()`，我们等待异步操作完成并获取结果。最后，我们打印反转后的字符串。

3. `std::launch::async`

```cpp
#include <iostream>
#include <future>
#include <chrono>

void print_hello() {
  std::this_thread::sleep_for(std::chrono::seconds(2)); // 模拟耗时任务
  std::cout << "Hello from async thread!" << std::endl;
}

int main() {
  std::future<void> fut = std::async(std::launch::async, print_hello);

  // 在此期间，可以执行其他任务

  fut.get(); // 等待异步操作完成

  std::cout << "Hello from main thread!" << std::endl;

  return 0;
}
```

在这个示例中，我们使用`std::async`和`std::launch::async`策略异步执行一个打印函数`print_hello`。这将在一个新线程中立即启动异步操作。我们可以在异步操作进行期间执行其他任务。通过调用`fut.get()`，我们等待异步操作完成。最后，我们在主线程中打印一条消息。

## 你能解释一下 std::async 和 std::launch::async 是如何协同工作的吗？

当然可以。`std::async`是一个模板函数，用于执行异步操作。它接收一个函数和这个函数的参数，然后在一个单独的线程或者在调用线程中执行这个函数。`std::async`返回一个`std::future`对象，表示异步操作的结果。可以通过调用`std::future::get()`或`std::future::wait()`来等待异步操作完成并获取结果。

`std::launch`是一个枚举类型，用于指定`std::async`的启动策略。它有两个枚举值：`std::launch::async`和`std::launch::deferred`。

1. `std::launch::async`：当将`std::launch::async`作为`std::async`的第一个参数时，它表示异步操作应该在一个新的线程中立即启动。这样，异步操作将在调用线程之外并发执行。这就是`std::async`和`std::launch::async`如何协同工作的。

例如：

```cpp
std::future<void> fut = std::async(std::launch::async, some_function, arg1, arg2);
```

2. `std::launch::deferred`：当将`std::launch::deferred`作为`std::async`的第一个参数时，它表示异步操作应该在`std::future::get()`或`std::future::wait()`被调用时才在调用线程中执行。这意味着异步操作将在调用线程中延迟执行。

例如：

```cpp
std::future<void> fut = std::async(std::launch::deferred, some_function, arg1, arg2);
```

如果不提供`std::launch`策略，`std::async`的默认行为是根据实现选择一个策略。这可能是`std::launch::async`，也可能是`std::launch::deferred`或者它们的组合。这意味着异步操作可能在调用线程中执行，也可能在单独的线程中执行。为了确保异步操作在新线程中执行，可以明确地提供`std::launch::async`策略。

## 这里为什么使用 `std::ref(exe)`

在这段提供的代码中，`std::ref`用于传递`rclcpp::executors::SingleThreadedExecutor`类型的对象`exe`的引用。`std::ref`主要用于在需要引用的情况下传递参数，特别是在和`std::thread`、`std::async`等多线程相关的场景中。

`std::ref`的主要原因是，当将参数传递给`std::thread`或`std::async`时，默认情况下，参数会被拷贝。然而，有时我们需要在多个线程或异步操作之间共享对象的状态。在这种情况下，我们需要传递对象的引用，而不是副本。`std::ref`允许我们实现这一目标。

在您提到的代码中，`std::ref`用于将执行器对象`exe`的引用传递给`wake_executor()`函数。这样做的目的是确保`wake_executor()`函数与调用`wake_executor()`的线程共享相同的执行器对象。这使得`wake_executor()`函数能够调用执行器对象的`spin_some()`方法，以在执行器上运行一些可用的回调。

例如：

```cpp
auto wake_executor = [&exe](std::shared_future<void> exit_future) {
  while (exit_future.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
    exe.spin_some(std::chrono::milliseconds(500));
  }
};

std::async(std::launch::async, wake_executor, std::ref(future));
```

在这个示例中，`std::ref`允许我们将`exe`的引用传递给`wake_executor()`函数，而不是一个副本。这样，**`wake_executor()`函数可以在与调用线程共享的执行器对象上执行操作**。
