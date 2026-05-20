# collect_imu.py —— 直接订阅，存到本地
# 把它放在PC上运行，和 ros2 topic echo 一样，但格式更干净

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuCollector(Node):
    def __init__(self):
        super().__init__('imu_collector')
        self.gyro = []
        self.accel = []
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.cb, 10)
        
    def cb(self, msg):
        self.gyro.append([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        self.accel.append([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])
        if len(self.gyro) % 500 == 0:
            self.get_logger().info(f'已收集 {len(self.gyro)} 条')

def main():
    rclpy.init()
    node = ImuCollector()
    try:
        print("正在收集... IMU请保持静置！Ctrl+C 停止并保存\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        g = np.array(node.gyro)
        a = np.array(node.accel)
        np.savez('imu_static.npz', gyro=g, accel=a)
        print(f"\n已保存 {len(g)} 条数据到 imu_static.npz")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()