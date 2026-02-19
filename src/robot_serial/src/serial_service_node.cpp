#include "rclcpp/rclcpp.hpp"
#include "robot_serial/fin_serial.hpp"
#include <robot_interfaces/srv/send_command.hpp>
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

constexpr size_t JOINT_NUM = 6;

class SerialServiceNode : public rclcpp::Node
{
public:
    SerialServiceNode()
        : Node("serial_service_node")
    {
        // 读取串口参数：serial_port_1 必填，serial_port_2 可选
        this->declare_parameter<std::string>("serial_port_1", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("serial_port_2", ""); // 留空则只启用单口

        const std::string port1 = this->get_parameter("serial_port_1").as_string();
        const std::string port2 = this->get_parameter("serial_port_2").as_string();
        try {
            // 至少打开第一个串口
            senders_.emplace_back(std::make_unique<FineSerialSender>(this, port1));
            RCLCPP_INFO(this->get_logger(), "已打开串口1: %s", port1.c_str());

            // 若提供了第二个串口，则再打开一个
            if (!port2.empty()) {
                senders_.emplace_back(std::make_unique<FineSerialSender>(this, port2));
                RCLCPP_INFO(this->get_logger(), "已打开串口2: %s", port2.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "串口初始化失败: %s", e.what());
            throw;
        }

        start_pub_ = this->create_publisher<std_msgs::msg::Bool>("start", 10);
        success_pub_ = this->create_publisher<std_msgs::msg::Bool>("success", 10);

        // 创建服务
        send_command_service_ = this->create_service<robot_interfaces::srv::SendCommand>(
            "send_command",
            std::bind(&SerialServiceNode::handleSendCommand, this,
                      std::placeholders::_1, std::placeholders::_2));
        // 订阅轨迹
        sub_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_dense_path", rclcpp::SystemDefaultsQoS(),
            std::bind(&SerialServiceNode::Traj_callback, this, std::placeholders::_1));

        // 定时发送轨迹点
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialServiceNode::sendNext, this));

        // 启动串口接收线程
        rx_running_.store(true);
        for (size_t i = 0; i < senders_.size(); ++i) {
            rx_threads_.emplace_back(&SerialServiceNode::rxLoop, this, i);
        }

        RCLCPP_INFO(this->get_logger(), "串口服务节点已启动");
    }

    ~SerialServiceNode() override
    {
        rx_running_.store(false);
        for (auto &t : rx_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

private:
    // 将命令广播到所有已打开的串口；全部成功才算成功
    template <typename Fn>
    bool broadcastToAll(const char* what, Fn&& sender_fn) {
        bool all_ok = true;
        for (size_t i = 0; i < senders_.size(); ++i) {
            bool ok = false;
            try {
                ok = sender_fn(*senders_[i]);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s 发送到串口%zu异常: %s", what, i + 1, e.what());
                ok = false;
            }
            if (!ok) {
                all_ok = false;
                RCLCPP_WARN(this->get_logger(), "%s 发送到串口%zu失败", what, i + 1);
            }
        }
        return all_ok;
    }

    void handleSendCommand(
        const std::shared_ptr<robot_interfaces::srv::SendCommand::Request> request,
        std::shared_ptr<robot_interfaces::srv::SendCommand::Response> response)
    {
        bool success = false;
        std::string message = "未知命令类型";

        switch (request->command_type) {
            case 1: { // MOVE_TARGET
                if (request->data.empty()) {
                    success = false;
                    message = "MOVE_TARGET 需要 data[0]";
                    break;
                }
                const auto target = static_cast<uint8_t>(request->data[0]);
                success = broadcastToAll("MOVE_TARGET", [&](FineSerialSender& s){
                    return s.sendFrame(FineSerialSender::CMD_MOVE_TARGET, &target, 1);
                });
                message = success ? "MOVE_TARGET 发送成功" : "MOVE_TARGET 发送失败";
                break;
            }
            case 2: { // DISPLAY_TASK_ORDER
                if (request->data.size() > 255) {
                    success = false;
                    message = "DISPLAY_TASK_ORDER 数据过长";
                    break;
                }
                std::vector<uint8_t> payload;
                payload.reserve(request->data.size());
                for (const auto v : request->data) {
                    const int iv = static_cast<int>(v);
                    payload.push_back(static_cast<uint8_t>(iv));
                }
                success = broadcastToAll("DISPLAY_TASK_ORDER", [&](FineSerialSender& s){
                    return s.sendFrame(FineSerialSender::CMD_DISPLAY_TASK_ORDER,
                                       payload.data(),
                                       static_cast<uint8_t>(payload.size()));
                });
                message = success ? "DISPLAY_TASK_ORDER 发送成功" : "DISPLAY_TASK_ORDER 发送失败";
                break;
            }
            case 3: { // TURNTABLE_ROTATE
                success = broadcastToAll("TURNTABLE_ROTATE", [&](FineSerialSender& s){
                    return s.sendFrame(FineSerialSender::CMD_TURNTABLE_ROTATE, nullptr, 0);
                });
                message = success ? "TURNTABLE_ROTATE 发送成功" : "TURNTABLE_ROTATE 发送失败";
                break;
            }
            case 4: { // GRIPPER_CLOSE
                success = broadcastToAll("GRIPPER_CLOSE", [&](FineSerialSender& s){
                    return s.sendFrame(FineSerialSender::CMD_GRIPPER_CLOSE, nullptr, 0);
                });
                message = success ? "GRIPPER_CLOSE 发送成功" : "GRIPPER_CLOSE 发送失败";
                break;
            }
            case 5: { // GRIPPER_OPEN
                success = broadcastToAll("GRIPPER_OPEN", [&](FineSerialSender& s){
                    return s.sendFrame(FineSerialSender::CMD_GRIPPER_OPEN, nullptr, 0);
                });
                message = success ? "GRIPPER_OPEN 发送成功" : "GRIPPER_OPEN 发送失败";
                break;
            }
            
            default: {
                message = "不支持的命令类型: " + std::to_string(request->command_type);
                success = false;
                break;
            }
        }

        response->success = success;
        response->message = message;
        RCLCPP_INFO(this->get_logger(), "处理命令 %d: %s", request->command_type, message.c_str());
    }

    void Traj_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        // 清空队列
        std::queue<std::array<float, 6>> empty;
        std::swap(queue_, empty);

        // 解析轨迹
        for (const auto &rt : msg->trajectory) {
            if (rt.joint_trajectory.joint_names.size() != JOINT_NUM) continue;
            for (const auto &pt : rt.joint_trajectory.points) {
                std::array<float, 6> j{};
                std::transform(pt.positions.begin(), pt.positions.end(),
                               j.begin(),
                               [](double d) { return static_cast<float>(d); });
                queue_.push(j);
            }
        }
        RCLCPP_INFO(get_logger(), "缓存 %zu 个轨迹点", queue_.size());
    }

    void sendNext()
    {
        if (queue_.empty()) return;
        const auto point = queue_.front();

        // 广播到所有串口：全部成功再弹出队列，确保两个设备同步
        bool all_ok = broadcastToAll("MOVE_MANIPULATOR", [&](FineSerialSender& s){
            return s.sendMoveManipulator(point);
        });

        if (all_ok) {
            queue_.pop();
        }
    }

    void rxLoop(size_t sender_index)
    {
        if (sender_index >= senders_.size()) {
            return;
        }

        enum class RxState { WAIT_HEADER, READ_CMD, READ_LEN, READ_PAYLOAD, READ_CRC, READ_TAIL };
        RxState state = RxState::WAIT_HEADER;
        uint8_t cmd = 0;
        uint8_t len = 0;
        uint8_t crc = 0;
        std::vector<uint8_t> payload;
        payload.reserve(256);

        while (rclcpp::ok() && rx_running_.load()) {
            std::vector<uint8_t> data;
            if (senders_[sender_index]->readAvailable(data) == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            for (const auto b : data) {
                switch (state) {
                    case RxState::WAIT_HEADER:
                        if (b == FineSerialSender::FRAME_HEADER) {
                            state = RxState::READ_CMD;
                        }
                        break;
                    case RxState::READ_CMD:
                        cmd = b;
                        state = RxState::READ_LEN;
                        break;
                    case RxState::READ_LEN:
                        len = b;
                        payload.clear();
                        if (len == 0) {
                            state = RxState::READ_CRC;
                        } else {
                            state = RxState::READ_PAYLOAD;
                        }
                        break;
                    case RxState::READ_PAYLOAD:
                        payload.push_back(b);
                        if (payload.size() >= len) {
                            state = RxState::READ_CRC;
                        }
                        break;
                    case RxState::READ_CRC:
                        crc = b;
                        state = RxState::READ_TAIL;
                        break;
                    case RxState::READ_TAIL:
                        if (b == FineSerialSender::FRAME_TRAILER) {
                            const uint8_t calc =
                                FineSerialSender::crc8_dvb_s2(payload.data(),
                                                             static_cast<uint16_t>(payload.size()));
                            if (calc == crc) {
                                handleFrame(cmd, payload);
                            } else {
                                RCLCPP_WARN(this->get_logger(),
                                            "串口%zu CRC错误 cmd=0x%02X len=%u",
                                            sender_index + 1, cmd, len);
                            }
                        } else if (b == FineSerialSender::FRAME_HEADER) {
                            state = RxState::READ_CMD;
                            cmd = 0;
                            len = 0;
                            payload.clear();
                            continue;
                        }
                        state = RxState::WAIT_HEADER;
                        break;
                }
            }
        }
    }

    void handleFrame(uint8_t cmd, const std::vector<uint8_t> &payload)
    {
        if (cmd == kRxStartCmd) {
            const bool value = payload.empty() ? true : (payload[0] != 0);
            publishBool(start_pub_, value);
            return;
        }

        if (cmd == kRxSuccessCmd) {
            const bool value = payload.empty() ? true : (payload[0] != 0);
            publishBool(success_pub_, value);
            return;
        }
    }

    void publishBool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub, bool value)
    {
        if (!pub) {
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = value;
        pub->publish(msg);
    }

    // 多串口发送器
    std::vector<std::unique_ptr<FineSerialSender>> senders_;

    rclcpp::Service<robot_interfaces::srv::SendCommand>::SharedPtr send_command_service_;
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::queue<std::array<float, 6>> queue_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub_;

    std::atomic<bool> rx_running_{false};
    std::vector<std::thread> rx_threads_;
    static constexpr uint8_t kRxStartCmd = 0x20;
    static constexpr uint8_t kRxSuccessCmd = 0x21;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SerialServiceNode>();
        RCLCPP_INFO(node->get_logger(), "串口服务节点开始运行...");
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("serial_service_node"), "节点运行异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
