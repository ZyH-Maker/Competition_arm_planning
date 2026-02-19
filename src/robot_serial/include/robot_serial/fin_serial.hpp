#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <serial/serial.h>
#include <array>
#include <vector>
#include <cstdint>
#include <mutex>

class FineSerialSender
{
public:
    // 协议常量
    static constexpr uint8_t FRAME_HEADER = 0xAA;
    static constexpr uint8_t FRAME_TRAILER = 0xBB;
    
    // 命令定义（重新规划）
    static constexpr uint8_t CMD_MOVE_TARGET = 0x10;
    static constexpr uint8_t CMD_DISPLAY_TASK_ORDER = 0x11;
    static constexpr uint8_t CMD_TURNTABLE_ROTATE = 0x12;
    static constexpr uint8_t CMD_GRIPPER_CLOSE = 0x13;
    static constexpr uint8_t CMD_GRIPPER_OPEN = 0x14;
    static constexpr uint8_t CMD_MOVE_MANIPULATOR = 0x0B;


    FineSerialSender(rclcpp::Node *node,const std::string& port_name);
    
    ~FineSerialSender();

    // CRC8校验
    static uint8_t crc8_dvb_s2(const uint8_t *data, uint16_t len);
    
    // 通用发送框架
    bool sendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);

    // 读取当前可用的字节（非阻塞），返回读取到的字节数
    size_t readAvailable(std::vector<uint8_t> &out);
    
    // 连续指令
    bool sendMoveManipulator(const std::array<float,6> &joint);

private:
    serial::Serial serial_;
    rclcpp::Logger logger_;
    std::mutex io_mutex_;
};
