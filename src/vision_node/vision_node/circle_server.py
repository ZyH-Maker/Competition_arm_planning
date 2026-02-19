#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import struct

from robot_interfaces.srv import GetCircles

# 圆服务节点,监听端口:9091
class CircleServer(Node):
    def __init__(self):
        super().__init__('cirlce_server')

        self.srv = self.create_service(
            GetCircles,
            'Circles',
            self.circle_callback
        )

        # 数据存储和锁
        self.current_circles_data = None
        self.circles_string = None
        self.circles_data_lock = threading.Lock()
        
        # Socket配置
        self.host = '0.0.0.0'
        self.port = 9091
        
        # 启动Socket服务器线程
        self.running = True
        self.server_thread = threading.Thread(target=self.start_socket_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f'圆服务节点启动，监听端口: {self.port}')



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
        """接收并解析三圆数据"""
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
                    
                    # 只处理圆数据 (type = 2)
                    if msg_type == 2 and data_size > 0:
                        # 读取圆数据
                        data = client_socket.recv(data_size)
                        
                        if data:
                            # 解析三圆数据
                            self.parse_circles_data(data)
                    
                    else:
                        self.get_logger().debug(f'跳过未知消息: type={msg_type}, size={data_size}')
                        
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

    def parse_circles_data(self, data):
        """

        解析固定三个圆的数据

        拼接成字符串发送 字符串格式: 颜色,flag,x,y,z

        circles_data = "1:red,true,0.0f,0.0f,0.0f;2:green,true,0.0f,0.0f,0.0f;3:blue,true,0.0f,0.0f,0.0f"

        颜色顺序固定为red,green,blue

        """
        try:
            offset = 0
            circles_data = []
            circles_string = []
            
            # 读取三个圆的数据
            for i in range(3):
                if offset + 4 > len(data):
                    break
                
                # 1. 颜色字符串长度 (4字节)
                color_len = struct.unpack('<I', data[offset:offset+4])[0]
                offset += 4
                
                # 2. 颜色字符串 (变长，包含结束符)
                if offset + color_len > len(data):
                    break
                color = data[offset:offset+color_len-1].decode('utf-8')
                offset += color_len
                
                # 3. flag (1字节)
                if offset + 1 > len(data):
                    break
                flag = bool(data[offset])
                offset += 1
                
                # 4. 坐标 (12字节，3个float)
                if offset + 12 > len(data):
                    break
                x, y, z = struct.unpack('<fff', data[offset:offset+12])
                offset += 12
                
                circles_data.append({
                    'color': color,
                    'flag': flag,
                    'x': float(x),
                    'y': float(y),
                    'z': float(z)
                })

                #字符串拼接
                circle_str = f"{i+1}:{color},{str(flag)},{x:.4f},{y:.4f},{z:.4f}"
                circles_string.append(circle_str)
                 
            self.circles_string = ";".join(circles_string)
            # 存储解析结果
            with self.circles_data_lock:
                self.current_circles_data = circles_data
            
            self.get_logger().info(f'拼接结果:{self.circles_string}',throttle_duration_sec=1.0)
            


            
        except Exception as e:
            self.get_logger().error(f'解析圆数据错误: {e}')
    

    def circle_callback(self,request,response):
        """处理服务请求"""
        response.circles_string = ""
        try:
            if request.trigger:
                with self.circles_data_lock:

                    circles_string = self.circles_string
                    self.circles_string = None

                if circles_string:

                    response.circles_string = circles_string
                    self.get_logger().info(f'服务返回: {circles_string}')
                else:

                    response.circles_string = ""
                    self.get_logger().warn('无圆数据可用')

            else:
                self.get_logger().info("等待请求中")
        except Exception as e:
            self.get_logger().error(f'服务处理错误: {e}')
            response.circles_string = ""

        return response
    
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'server_socket'):
            self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CircleServer()
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
    
