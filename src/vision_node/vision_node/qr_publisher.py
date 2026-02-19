#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import struct

class QrPublisher(Node):
    def __init__(self):
        super().__init__('qr_publisher')
        
        # 创建发布者
        self.publisher = self.create_publisher(String, '/qr_code', 10)
        
        # Socket配置
        self.host = '0.0.0.0'  # 监听所有接口
        self.port = 9090
        
        # 启动Socket服务器线程
        self.running = True
        self.server_thread = threading.Thread(target=self.start_socket_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f'QR发布节点启动，监听端口: {self.port}')
    
    def start_socket_server(self):
        """启动Socket服务器"""
        try:
            # 创建Socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(2.0)  # 设置超时，以便检查rclpy状态
            
            self.get_logger().info('Socket服务器准备就绪，等待连接...')
            
            while self.running and rclpy.ok():
                try:
                    # 接受连接
                    client_socket, client_address = self.server_socket.accept()
                    self.get_logger().info(f'视觉库已连接: {client_address}')
                    
                    # 处理客户端
                    self.handle_client(client_socket)
                    
                except socket.timeout:
                    continue  # 正常超时，继续循环
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f'接受连接错误: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'服务器启动失败: {e}')
    
    def handle_client(self, client_socket):
        """处理单个客户端连接"""
        try:
            client_socket.settimeout(0.1)  # 设置短超时，避免阻塞
            
            while self.running and rclpy.ok():
                try:
                    # 读取消息头 (9字节: 4字节magic + 1字节type + 4字节size)
                    header = client_socket.recv(9)
                    
                    if not header:
                        break  # 连接断开
                    
                    if len(header) < 9:
                        continue  # 数据不完整，继续等待
                    
                    # 解析消息头
                    magic, msg_type, data_size = struct.unpack('<IBI', header)
                    
                    # 验证magic number (0x56425331 = 'VBS1')
                    if magic != 0x56425331:
                        self.get_logger().warn(f'无效的消息头: {hex(magic)}')
                        continue
                    
                    # 只处理字符串消息 (type = 1)
                    if msg_type == 1 and data_size > 0:
                        # 读取字符串数据
                        data = client_socket.recv(data_size)
                        
                        if data:
                            # 转换为字符串（去掉结束符\0）
                            qr_text = data.decode('utf-8').rstrip('\x00')
                            
                            # 发布到ROS 2
                            self.publish_qr_code(qr_text)
                            
                    else:
                        self.get_logger().debug(f'跳过非字符串消息: type={msg_type}, size={data_size}')
                        
                except socket.timeout:
                    continue  # 非阻塞读取，正常超时
                except Exception as e:
                    self.get_logger().error(f'处理消息错误: {e}')
                    break
                    
        except Exception as e:
            self.get_logger().error(f'客户端处理错误: {e}')
        finally:
            client_socket.close()
            self.get_logger().info('视觉库连接断开')
    
    def publish_qr_code(self, qr_text):
        """发布二维码信息到ROS 2"""
        try:
            # 创建ROS 2消息
            msg = String()
            msg.data = qr_text
            
            # 发布
            self.publisher.publish(msg)
            
            # 日志输出
            self.get_logger().info(f'发布: {qr_text}')
            
        except Exception as e:
            self.get_logger().error(f'发布消息失败: {e}')
    
    def destroy_node(self):
        """清理资源"""
        self.running = False
        if hasattr(self, 'server_socket'):
            self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QrPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()