#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class MockFTSensor(Node):
    def __init__(self):
        super().__init__('mock_fts_broadcaster')
        
        # 目标话题需与 force_admittance_servo 节点订阅的一致
        self.publisher_ = self.create_publisher(WrenchStamped, '/tcp_fts_sensor/wrench', 10)
        
        # 控制频率 125Hz (0.008s)
        self.timer_period = 0.008
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 记录启动时间
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # 设定 -10N 维持的时间（秒）
        self.force_duration = 5.0 
        
        self.get_logger().info('伪造传感器节点已启动，Z轴将输出 -10N 持续 5 秒...')

    def timer_callback(self):
        msg = WrenchStamped()
        
        # 填充 Header
        msg.header.stamp = self.get_clock().now().to_msg()
        # 坐标系需与你的控制坐标系或传感器坐标系对齐
        msg.header.frame_id = 'tool0' 
        
        # 计算已运行时间
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # 时间逻辑控制
        if elapsed < self.force_duration:
            msg.wrench.force.z = -10.0
        else:
            msg.wrench.force.z = 0.0
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockFTSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
