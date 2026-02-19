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
  // 参数：
  // - pid: 需要检测的子进程 PID。
  // 返回：
  // - true: 进程仍在运行，或无法确认其已退出。
  // - false: 进程已退出（被 waitpid 回收到了退出状态）。
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
  // 参数：
  // - pid: 需要等待退出的子进程 PID。
  // - timeout_ms: 超时毫秒。<0 表示无限等待。
  // 返回：
  // - true: 进程已退出或已被回收。
  // - false: 在 timeout_ms 时间内未退出。
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
  // 参数：
  // - key: 进程键名（如 "qrcode"/"turntable"）。
  // 行为：
  // - 从全局进程表中移除该键，避免后续引用失效 PID。
  std::lock_guard<std::mutex> lock(g_processes_mutex);
  g_processes.erase(key);
}
}  // namespace

// 构造函数参数说明：
// - name: BT 节点实例名（来自 XML Action ID）。
// - config: BT 端口配置对象（框架传入）。
// - ctx: 共享上下文（ROS 节点、MoveIt、通信句柄等）。
InitAllConfigs::InitAllConfigs(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<BtContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
{}

BT::NodeStatus InitAllConfigs::tick()
{
  // 功能：
  // - 做系统启动前自检：检查 send_command 服务与 Circles 服务是否可达。
  // 返回：
  // - SUCCESS: 两个服务都可用。
  // - FAILURE: 任意服务不可用或上下文无效。
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
  // 构造参数含义（供维护者查阅）：
  // - process_key: 外部进程的逻辑键，作为进程表索引。
  // - command: 通过 "/bin/sh -c" 执行的完整命令字符串。
  //
  // tick 行为：
  // 1) 若该 key 已有存活进程，直接 SUCCESS（避免重复启动）。
  // 2) 否则 fork 子进程，并 execl 执行 command。
  // 3) 父进程记录 PID，供 Stop/Kill 节点管理。
  // 返回：
  // - SUCCESS: 已存在可用进程，或新进程启动成功。
  // - FAILURE: ctx 无效、命令为空、fork 失败。
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
  // 构造参数含义：
  // - process_key: 需要停止的外部进程键。
  //
  // tick 行为：
  // 1) 在进程表查找 process_key 对应 PID。
  // 2) 若进程不存在/已退出，清理表项后 SUCCESS。
  // 3) 对进程组发送 SIGKILL，并等待退出回收。
  // 返回：
  // - SUCCESS: 停止成功，或本就没有需要停止的进程。
  // - FAILURE: 仅在上下文无效时返回。
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

  // 当前策略：直接 SIGKILL（按进程组发送），确保视觉进程及其子进程被终止。
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
  // 功能：
  // - 遍历进程表，逐个按进程组发送 SIGKILL 并等待退出。
  // 返回：
  // - 始终 SUCCESS（用于错误兜底清理，不阻断 BT 结束流程）。
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
  // 构造参数含义：
  // - topic: 目标话题名。
  // - message: 每次 tick 固定发送的字符串内容。
  if (ctx_ && ctx_->node)
  {
    pub_ = ctx_->node->create_publisher<std_msgs::msg::String>(topic_, 10);
  }
}

BT::NodeStatus PublishConstStringAction::tick()
{
  // 功能：
  // - 向 topic_ 发布固定 message_。
  // 返回：
  // - SUCCESS: 发布成功。
  // - FAILURE: 上下文或发布器未初始化。
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
  // 构造参数含义：
  // - topic: 目标话题名，消息内容来自输入端口 task_order_in。
  if (ctx_ && ctx_->node)
  {
    pub_ = ctx_->node->create_publisher<std_msgs::msg::String>(topic_, 10);
  }
}

BT::NodeStatus PublishFromPortStringAction::tick()
{
  // 功能：
  // - 从输入端口 task_order_in 读取字符串并发布到 topic_。
  // 返回：
  // - SUCCESS: 读取端口并发布成功。
  // - FAILURE: 发布器无效或端口缺失。
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
  // 构造参数含义：
  // - topic: 需要等待的 Bool 话题名。
  // 行为：
  // - 订阅 topic，收到 data=true 时将 received_ 置位。
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
  // 功能：
  // - 启动等待状态，清空上一轮标志位。
  // 返回：
  // - RUNNING: 开始等待。
  // - FAILURE: 上下文/订阅器无效。
  if (!ctx_ || !ctx_->node || !sub_)
  {
    return BT::NodeStatus::FAILURE;
  }

  received_.store(false);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBoolTopicAction::onRunning()
{
  // 功能：
  // - 轮询 received_。
  // 返回：
  // - SUCCESS: 已收到 true。
  // - RUNNING: 继续等待。
  if (received_.load())
  {
    received_.store(false);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForBoolTopicAction::onHalted()
{
  // 功能：
  // - 节点被中断时复位状态，避免下一轮误触发。
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
  // 功能：
  // - 进入 RUNNING 状态，等待二维码文件可读取。
  // 返回：
  // - RUNNING: 正常。
  // - FAILURE: 上下文无效。
  if (!ctx_)
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReadQRCodeFileAction::onRunning()
{
  // 功能：
  // - 读取二维码任务文件第一行，解析两批任务顺序。
  // 输入格式（推荐）：
  // - "123+231"
  // 输出端口：
  // - task_order_out: "1,2,3,2,3,1"（用于显示）
  // - batch1_order_out: "1,2,3"
  // - batch2_order_out: "2,3,1"
  // 返回：
  // - SUCCESS: 解析完成且两批都非空。
  // - RUNNING: 文件不存在/不可读/格式不完整，继续等待下一 tick。
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

  auto toCommaOrder = [](const std::string& digits_only) -> std::string
  {
    // 参数：
    // - digits_only: 仅包含数字字符的序列，例如 "123"。
    // 返回：
    // - 逗号分隔形式，例如 "1,2,3"。
    std::string out;
    out.reserve(digits_only.size() * 2);
    for (size_t i = 0; i < digits_only.size(); ++i)
    {
      if (i > 0)
      {
        out.push_back(',');
      }
      out.push_back(digits_only[i]);
    }
    return out;
  };

  std::string left_digits;
  std::string right_digits;
  bool seen_plus = false;

  for (const char ch : raw)
  {
    if (ch == '+')
    {
      seen_plus = true;
      continue;
    }

    if (!std::isdigit(static_cast<unsigned char>(ch)))
    {
      continue;
    }

    if (!seen_plus)
    {
      left_digits.push_back(ch);
    }
    else
    {
      right_digits.push_back(ch);
    }
  }

  // 兼容旧格式：
  // - 若没有 '+' 且总长度 >= 6，则按前3位/后3位切分两批。
  // - 仅3位任务码视为非法输入（不兼容），保持 RUNNING 等待正确格式。
  if (!seen_plus)
  {
    if (left_digits.size() >= 6)
    {
      right_digits = left_digits.substr(3, 3);
      left_digits = left_digits.substr(0, 3);
    }
  }

  if (left_digits.empty() || right_digits.empty())
  {
    return BT::NodeStatus::RUNNING;
  }

  const std::string batch1_order = toCommaOrder(left_digits);
  const std::string batch2_order = toCommaOrder(right_digits);
  const std::string task_order = batch1_order + "," + batch2_order;

  setOutput("task_order_out", task_order);
  setOutput("batch1_order_out", batch1_order);
  setOutput("batch2_order_out", batch2_order);
  return BT::NodeStatus::SUCCESS;
}

void ReadQRCodeFileAction::onHalted() {}
// onHalted:
// - 当前节点无内部资源需要释放，保持空实现即可。

}  // namespace main_bt
