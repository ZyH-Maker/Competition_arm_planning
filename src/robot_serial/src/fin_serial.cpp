#include "rclcpp/rclcpp.hpp"
#include "robot_serial/fin_serial.hpp"
#include <cstring>
#include <iomanip>
#include <string>  

/**************** CRC8 实现 *******************************/
uint8_t FineSerialSender::crc8_dvb_s2(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

/**************** 构造函数 *******************************/
FineSerialSender::FineSerialSender(rclcpp::Node *node,const std::string& port_name)
  : logger_(node->get_logger())
{
    std::string port = port_name;
    uint32_t baud_rate = 921600;
    serial_.setPort(port);
    serial_.setBaudrate(baud_rate);
    serial::Timeout time = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(time);
    try {
        serial_.open();
        RCLCPP_INFO(logger_, "串口 %s @%u 已打开", port.c_str(), baud_rate);
    } catch (serial::IOException &e) {
        RCLCPP_FATAL(logger_, "串口打开失败: %s", e.what());
        throw;
    }
}

FineSerialSender::~FineSerialSender()
{
    if (serial_.isOpen()) {
        serial_.close();
        RCLCPP_INFO(logger_, "串口已关闭");
    }
}

/**************** 工具：统一封包 ***************************/
bool FineSerialSender::sendFrame(uint8_t cmd,
                                 const uint8_t *payload,
                                 uint8_t len)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    std::vector<uint8_t> pkt(1 + 1 + 1 + len + 1 + 1);
    size_t i = 0;
    pkt[i++] = FRAME_HEADER;
    pkt[i++] = cmd;
    pkt[i++] = len;
    if (len && payload)
    {
        std::memcpy(&pkt[i], payload, len);
    }
    i += len;
    pkt[i++] = crc8_dvb_s2(&pkt[3], len);
    pkt[i++] = FRAME_TRAILER;
    
    size_t n = serial_.write(pkt);
    if (n != pkt.size())
    {
        RCLCPP_ERROR(logger_, "串口发送不完整 %zu/%zu", n, pkt.size());
        return false;
    }
    return true;
}

size_t FineSerialSender::readAvailable(std::vector<uint8_t> &out)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!serial_.isOpen())
    {
        return 0;
    }

    const size_t avail = serial_.available();
    if (avail == 0)
    {
        return 0;
    }

    const std::string data = serial_.read(avail);
    out.assign(data.begin(), data.end());
    return out.size();
}

bool FineSerialSender::sendMoveManipulator(const std::array<float,6> &joint)
{ 
    return sendFrame(CMD_MOVE_MANIPULATOR,
                   reinterpret_cast<const uint8_t*>(joint.data()), 24); 
}
