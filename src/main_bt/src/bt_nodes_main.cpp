#include "main_bt/bt_nodes.hpp"

// 说明：
// 该文件实现 main_bt 主流程相关节点（主树用到的通用动作节点）。
// 主要包含：
// - InitAllConfigs：初始化占位（记录起始时间）
// - Start*Vision / Stop*Vision：以外部进程方式启动/关闭视觉程序
// - Go* / WaitArrive* / ScanQRCode / DisplayQRCode：话题发布/订阅 + 文件读取相关
//
// 注意：进程相关逻辑完全依赖 POSIX 系统调用（fork/exec/kill/waitpid），
// 由本进程派生出子进程执行外部命令。

#include <cerrno>
#include <cctype>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <robot_interfaces/srv/get_circles.hpp>
#include <robot_interfaces/srv/send_command.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace main_bt
{
namespace
{
// 进程表：按 key 保存启动的外部进程 PID。
// key 由节点注册时传入（例如 "qrcode"/"turntable"）。
std::unordered_map<std::string, pid_t> g_processes;
std::mutex g_processes_mutex;
constexpr int kKillWaitMs = 10000;

// 判断 PID 是否仍在运行：
// waitpid(..., WNOHANG) == 0 表示子进程仍在运行；
// 返回 PID 表示已退出；-1 且 ECHILD 代表没有这个子进程。
bool processAlive(pid_t pid)
{
  int status = 0;
  const pid_t res = ::waitpid(pid, &status, WNOHANG);
  if (res == 0)
  {
    return true;
  }
  if (res == pid)
  {
    return false;
  }
  return !(res == -1 && errno == ECHILD);
}

// 等待子进程退出（带超时），用于 Stop 时优雅等待。
bool waitForExit(pid_t pid, int timeout_ms)
{
  if (timeout_ms < 0)
  {
    int status = 0;
    const pid_t res = ::waitpid(pid, &status, 0);
    if (res == pid)
    {
      return true;
    }
    return (res == -1 && errno == ECHILD);
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    int status = 0;
    const pid_t res = ::waitpid(pid, &status, WNOHANG);
    if (res == pid)
    {
      return true;
    }
    if (res == -1 && errno == ECHILD)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

// 从进程表移除 key（避免悬挂 PID）。
void removeProcess(const std::string& key)
{
  std::lock_guard<std::mutex> lock(g_processes_mutex);
  g_processes.erase(key);
}
}  // namespace

InitAllConfigs::InitAllConfigs(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus InitAllConfigs::tick()
{
  // 记录“初始化开始时间”（占位功能，方便后续统计或扩展）。
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = ctx_->node->now();
  RCLCPP_INFO(ctx_->node->get_logger(), "初始化配置：起始时间 %.3f",
              start_time_.seconds());

  bool ok = true;

  auto send_client =
    ctx_->node->create_client<robot_interfaces::srv::SendCommand>("send_command");
  if (!send_client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "初始化配置：发送命令服务不可用");
    ok = false;
  }

  auto circles_client =
    ctx_->node->create_client<robot_interfaces::srv::GetCircles>("Circles");
  if (!circles_client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "初始化配置：圆检测服务不可用");
    ok = false;
  }

  if (!ok)
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "初始化配置：自检失败");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "初始化配置：自检通过");
  return BT::NodeStatus::SUCCESS;
}

StartProcessAction::StartProcessAction(const std::string& name, const BT::NodeConfig& config,
                                       std::shared_ptr<BtContext> ctx,
                                       const std::string& process_key,
                                       const std::string& command)
  : BT::SyncActionNode(name, config),
    ctx_(std::move(ctx)),
    process_key_(process_key),
    command_(command)
{}

BT::NodeStatus StartProcessAction::tick()
{
  // 说明：
  // 1) 如果该 key 对应进程仍在运行，直接返回 SUCCESS
  // 2) 否则 fork 出子进程，并在子进程里 exec 外部命令
  // 3) 将 pid 记录到进程表，供 Stop 节点关闭
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (command_.empty())
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "启动进程动作：%s 的命令为空",
                process_key_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(g_processes_mutex);
    auto it = g_processes.find(process_key_);
    if (it != g_processes.end())
    {
      if (processAlive(it->second))
      {
        return BT::NodeStatus::SUCCESS;
      }
      g_processes.erase(it);
    }
  }

  // fork 创建子进程；子进程用 /bin/sh -c 执行命令字符串
  const pid_t pid = ::fork();
  if (pid < 0)
  {
    RCLCPP_ERROR(ctx_->node->get_logger(), "启动进程动作：创建子进程失败：%s",
                process_key_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (pid == 0)
  {
    // 子进程：独立进程组，便于后续按组终止（涵盖其子进程）。
    ::setpgid(0, 0);
    // 子进程：执行命令。若失败，直接退出 127。
    ::execl("/bin/sh", "sh", "-c", command_.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  // 父进程：记录 PID
  {
    std::lock_guard<std::mutex> lock(g_processes_mutex);
    g_processes[process_key_] = pid;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "启动进程动作：已启动 %s，进程号=%d",
              process_key_.c_str(), static_cast<int>(pid));
  return BT::NodeStatus::SUCCESS;
}

StopProcessAction::StopProcessAction(const std::string& name, const BT::NodeConfig& config,
                                     std::shared_ptr<BtContext> ctx,
                                     const std::string& process_key)
  : BT::SyncActionNode(name, config),
    ctx_(std::move(ctx)),
    process_key_(process_key)
{}

BT::NodeStatus StopProcessAction::tick()
{
  // 说明：
  // 1) 找到该 key 的 pid
  // 2) 先 SIGTERM 请求优雅退出
  // 3) 超时则 SIGKILL 强杀
  // 4) waitpid 回收子进程，避免僵尸进程
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::FAILURE;
  }

  pid_t pid = -1;
  {
    std::lock_guard<std::mutex> lock(g_processes_mutex);
    auto it = g_processes.find(process_key_);
    if (it != g_processes.end())
    {
      pid = it->second;
    }
  }

  if (pid <= 0)
  {
    RCLCPP_WARN(ctx_->node->get_logger(), "停止进程动作：%s 没有进程",
                process_key_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (!processAlive(pid))
  {
    // 子进程已退出，直接清理记录
    removeProcess(process_key_);
    return BT::NodeStatus::SUCCESS;
  }

  auto kill_group = [&](int sig, const char* sig_name)
  {
    if (::kill(-pid, sig) == -1 && errno != ESRCH)
    {
      RCLCPP_WARN(ctx_->node->get_logger(),
                  "停止进程动作：发送信号 %s 失败：%s", sig_name, process_key_.c_str());
    }
  };

  // 直接 SIGKILL（按进程组发送）
  kill_group(SIGKILL, "SIGKILL");
  if (!waitForExit(pid, kKillWaitMs))
  {
    RCLCPP_ERROR(ctx_->node->get_logger(),
                 "停止进程动作：停止 %s 超时", process_key_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  removeProcess(process_key_);
  RCLCPP_INFO(ctx_->node->get_logger(), "停止进程动作：已停止 %s", process_key_.c_str());
  return BT::NodeStatus::SUCCESS;
}

KillAllProcessesAction::KillAllProcessesAction(const std::string& name,
                                               const BT::NodeConfig& config,
                                               std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus KillAllProcessesAction::tick()
{
  if (!ctx_ || !ctx_->node)
  {
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<std::pair<std::string, pid_t>> processes;
  {
    std::lock_guard<std::mutex> lock(g_processes_mutex);
    processes.reserve(g_processes.size());
    for (const auto& it : g_processes)
    {
      processes.emplace_back(it.first, it.second);
    }
  }

  for (const auto& item : processes)
  {
    const auto& key = item.first;
    const pid_t pid = item.second;
    if (pid <= 0)
    {
      removeProcess(key);
      continue;
    }

    if (::kill(-pid, SIGKILL) == -1 && errno != ESRCH)
    {
      RCLCPP_WARN(ctx_->node->get_logger(),
                  "结束全部进程：发送 SIGKILL 失败：%s", key.c_str());
    }

    if (waitForExit(pid, kKillWaitMs))
    {
      removeProcess(key);
      RCLCPP_INFO(ctx_->node->get_logger(), "结束全部进程：已停止 %s", key.c_str());
    }
    else
    {
      RCLCPP_ERROR(ctx_->node->get_logger(),
                   "结束全部进程：停止 %s 超时", key.c_str());
    }
  }

  return BT::NodeStatus::SUCCESS;
}

PublishConstStringAction::PublishConstStringAction(const std::string& name,
                                                   const BT::NodeConfig& config,
                                                   std::shared_ptr<BtContext> ctx,
                                                   const std::string& topic,
                                                   const std::string& message)
  : BT::SyncActionNode(name, config),
    ctx_(std::move(ctx)),
    topic_(topic),
    message_(message)
{
  if (ctx_ && ctx_->node)
  {
    pub_ = ctx_->node->create_publisher<std_msgs::msg::String>(topic_, 10);
  }
}

BT::NodeStatus PublishConstStringAction::tick()
{
  if (!ctx_ || !ctx_->node || !pub_)
  {
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = message_;
  pub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

PublishFromPortStringAction::PublishFromPortStringAction(const std::string& name,
                                                         const BT::NodeConfig& config,
                                                         std::shared_ptr<BtContext> ctx,
                                                         const std::string& topic)
  : BT::SyncActionNode(name, config),
    ctx_(std::move(ctx)),
    topic_(topic)
{
  if (ctx_ && ctx_->node)
  {
    pub_ = ctx_->node->create_publisher<std_msgs::msg::String>(topic_, 10);
  }
}

BT::NodeStatus PublishFromPortStringAction::tick()
{
  if (!ctx_ || !ctx_->node || !pub_)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto input = getInput<std::string>("task_order_in");
  if (!input)
  {
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = input.value();
  pub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

WaitForBoolTopicAction::WaitForBoolTopicAction(const std::string& name,
                                               const BT::NodeConfig& config,
                                               std::shared_ptr<BtContext> ctx,
                                               const std::string& topic)
  : BT::StatefulActionNode(name, config),
    ctx_(std::move(ctx)),
    topic_(topic),
    received_(false)
{
  if (ctx_ && ctx_->node)
  {
    sub_ = ctx_->node->create_subscription<std_msgs::msg::Bool>(
      topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          received_.store(true);
        }
      });
  }
}

BT::NodeStatus WaitForBoolTopicAction::onStart()
{
  if (!ctx_ || !ctx_->node || !sub_)
  {
    return BT::NodeStatus::FAILURE;
  }

  received_.store(false);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBoolTopicAction::onRunning()
{
  if (received_.load())
  {
    received_.store(false);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForBoolTopicAction::onHalted()
{
  received_.store(false);
}

ReadQRCodeFileAction::ReadQRCodeFileAction(const std::string& name,
                                           const BT::NodeConfig& config,
                                           std::shared_ptr<BtContext> ctx)
  : BT::StatefulActionNode(name, config),
    ctx_(std::move(ctx))
{}

BT::NodeStatus ReadQRCodeFileAction::onStart()
{
  if (!ctx_)
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReadQRCodeFileAction::onRunning()
{
  const std::filesystem::path path("/home/fins/robot/src/Config/Qr.yaml");
  if (!std::filesystem::exists(path))
  {
    return BT::NodeStatus::RUNNING;
  }

  std::ifstream in(path);
  if (!in.is_open())
  {
    return BT::NodeStatus::RUNNING;
  }

  std::string raw;
  std::getline(in, raw);
  if (raw.empty())
  {
    return BT::NodeStatus::RUNNING;
  }

  std::string digits;
  digits.reserve(raw.size());
  for (const char ch : raw)
  {
    if (std::isdigit(static_cast<unsigned char>(ch)))
    {
      digits.push_back(ch);
    }
  }

  if (digits.empty())
  {
    return BT::NodeStatus::RUNNING;
  }

  std::string task_order;
  task_order.reserve(digits.size() * 2);
  for (size_t i = 0; i < digits.size(); ++i)
  {
    if (i > 0)
    {
      task_order.push_back(',');
    }
    task_order.push_back(digits[i]);
  }

  setOutput("task_order_out", task_order);
  return BT::NodeStatus::SUCCESS;
}

void ReadQRCodeFileAction::onHalted() {}

}  // namespace main_bt
