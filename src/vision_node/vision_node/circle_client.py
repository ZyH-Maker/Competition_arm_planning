#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_interfaces.srv import Circles

def main(args=None):
    """测试圆服务的主函数"""
    rclpy.init(args=args)
    
    # 创建临时节点
    node = Node('test_circle_client')
    
    try:
        # 创建客户端
        client = node.create_client(Circles, 'Circles')
        
        # 等待服务可用
        print("等待服务连接...")
        while not client.wait_for_service(timeout_sec=1.0):
            print("服务未就绪，继续等待...")
        
        print("服务连接成功!")
        
        # 创建请求
        request = Circles.Request()
        request.trigger = True
        
        # 发送请求
        print("发送请求...")
        future = client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(node, future)
        
        if future.done():
            try:
                response = future.result()
                print(f"✓ 成功收到响应:")
                print(f"  圆数据: {response.circles_string}")
            except Exception as e:
                print(f"✗ 服务调用失败: {e}")
        else:
            print("✗ 请求未完成")
    
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()