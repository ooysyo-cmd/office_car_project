#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32   # 新增 Float32 用于电压/温度
import serial
import struct
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # ========== 串口配置 ==========
        self.port = '/dev/ttyUSB0'     # 改成你的串口号
        self.baudrate = 115200
        
        # ========== 发送命令用的变量 ==========
        self.steer_tx = 0    # 转向命令 (int16)
        self.speed_tx = 0    # 速度命令 (int16)
        
        # ========== 接收反馈的变量 ==========
        self.start      = 0
        self.cmd1_rx    = 0   # 对应 steer 或 brake
        self.cmd2_rx    = 0   # 对应 speed 或 throttle
        self.speedR_rx  = 0   # 右轮转速
        self.speedL_rx  = 0   # 左轮转速
        self.batVoltage = 0   # 电池电压 (mV 或 0.1V?)
        self.boardTemp  = 0   # 温度 (摄氏度)
        self.cmdLed_rx  = 0   # LED 状态
        self.checksum_rx= 0
        
        # ========== 打开串口 ==========
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.get_logger().info(f'✅ 串口已打开: {self.port}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口打开失败: {e}')
            self.ser = None
        
        # ========== 接收缓冲区 ==========
        self.buffer = bytearray()
        
        # ========== 订阅话题（用于发送命令） ==========
        self.speed_sub = self.create_subscription(
            Int16, '/cmd_speed', self.speed_callback, 10
        )
        # 如果你还需要单独控制转向，可以再加一个订阅
        self.steer_sub = self.create_subscription(
            Int16, '/cmd_steer', self.steer_callback, 10
        )
        
        # ========== 发布话题（用于反馈数据） ==========
        self.pub_cmd1   = self.create_publisher(Int16,   '/feedback/cmd1', 10)       # 刹车/转向命令
        self.pub_cmd2   = self.create_publisher(Int16,   '/feedback/cmd2', 10)       # 油门/速度命令
        self.pub_speedL = self.create_publisher(Int16,   '/feedback/speed_left', 10) # 左轮转速
        self.pub_speedR = self.create_publisher(Int16,   '/feedback/speed_right', 10)# 右轮转速
        self.pub_battery= self.create_publisher(Float32, '/feedback/battery_voltage', 10)  # 电压 (浮点数)
        self.pub_temp   = self.create_publisher(Float32, '/feedback/board_temp', 10)       # 温度 (浮点数)
        self.pub_led    = self.create_publisher(Int16,   '/feedback/led', 10)         # LED 状态
        
        # ========== 启动接收线程 ==========
        self.running = True
        self.recv_thread = threading.Thread(target=self.receive_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()
        
        # ========== 定时发送命令（20Hz） ==========
        self.timer = self.create_timer(0.05, self.send_command)
        
        self.get_logger().info('🚀 节点已启动（接收18字节反馈）')
    
    # ==================== 发送命令部分（8字节，保持原有格式） ====================
    def speed_callback(self, msg):
        """收到速度指令"""
        speed = msg.data
        self.speed_tx = max(-300, min(300, speed))
        self.get_logger().info(f'📨 发送速度指令: {self.speed_tx}')
    
    def steer_callback(self, msg):
        """收到转向指令"""
        steer = msg.data
        self.steer_tx = max(-300, min(300, steer))
        self.get_logger().info(f'📨 发送转向指令: {self.steer_tx}')
    
    def calculate_checksum(self, start, steer, speed):
        """计算8字节命令的XOR校验（和原来一样）"""
        data = struct.pack('<HHH', start, steer & 0xFFFF, speed & 0xFFFF)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFFFF
    
    def send_command(self):
        """发送8字节命令（格式：start + steer + speed + checksum）"""
        if self.ser is None:
            return
        
        start = 0xABCD
        checksum = self.calculate_checksum(start, self.steer_tx, self.speed_tx)
        cmd = struct.pack('<HHHH', start, self.steer_tx, self.speed_tx, checksum)
        
        try:
            self.ser.write(cmd)
            # 调试：self.get_logger().debug(f'TX: {cmd.hex()}')
        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')
    
    # ==================== 接收反馈部分（18字节） ====================
    def receive_loop(self):
        """串口接收线程"""
        while self.running and rclpy.ok():
            if self.ser is None:
                break
            
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.get_logger().info(f'RAW: {data.hex()}')
                    self.buffer.extend(data)
                    self.parse_buffer()
                    
            except Exception as e:
                self.get_logger().error(f'接收错误: {e}')
    
    def parse_buffer(self):
        """解析缓冲区中的18字节帧"""
        FRAME_LEN = 18   # 关键改动：帧长度改为18字节
        
        while len(self.buffer) >= FRAME_LEN:
            frame = self.buffer[:FRAME_LEN]
            
            if self.parse_frame(frame):
                self.buffer = self.buffer[FRAME_LEN:]   # 解析成功，移除
            else:
                # 帧同步错误，丢弃第一个字节
                self.get_logger().warn(f'帧解析失败，跳过字节: 0x{self.buffer[0]:02X}')
                self.buffer = self.buffer[1:]
    
    def parse_frame(self, frame):
        """
        解析18字节帧，结构体为 SerialFeedback（小端序）：
        - start       : uint16   (应为 0xABCD)
        - cmd1        : int16
        - cmd2        : int16
        - speedR_meas : int16
        - speedL_meas : int16
        - batVoltage  : int16   (单位 0.01V，例如 4073 → 40.73V)
        - boardTemp   : int16   (单位 0.1°C，例如 459 → 45.9°C)
        - cmdLed      : uint16
        - checksum    : uint16   (前8个 uint16_t 的异或)
        """
        try:
            # 全部按无符号16位解包（9个字段）
            unpacked = struct.unpack('<9H', frame)
            start, cmd1_u, cmd2_u, speedR_u, speedL_u, bat_u, temp_u, led, checksum = unpacked

            # 将有符号的字段转换为正确的有符号整数
            def to_signed(x):
                return x if x < 32768 else x - 65536

            cmd1   = to_signed(cmd1_u)
            cmd2   = to_signed(cmd2_u)
            speedR = to_signed(speedR_u)
            speedL = to_signed(speedL_u)
            bat    = to_signed(bat_u)    # 单位 0.01V
            temp   = to_signed(temp_u)   # 单位 0.1°C

            # 帧头校验（0xABCD）
            if start != 0xABCD:
                self.get_logger().debug(f'帧头错误: 0x{start:04X}')
                return False

            # 正确的校验和算法：前8个 uint16_t 异或
            checksum_calc = start ^ cmd1_u ^ cmd2_u ^ speedR_u ^ speedL_u ^ bat_u ^ temp_u ^ led
            checksum_calc &= 0xFFFF

            if checksum != checksum_calc:
                self.get_logger().warn(f'校验和错误: 计算 0x{checksum_calc:04X}, 收到 0x{checksum:04X}')
                return False

            # 保存数值
            self.start      = start
            self.cmd1_rx    = cmd1
            self.cmd2_rx    = cmd2
            self.speedR_rx  = speedR
            self.speedL_rx  = speedL
            self.batVoltage = bat
            self.boardTemp  = temp
            self.cmdLed_rx  = led
            self.checksum_rx= checksum

            # 发布到 ROS2 话题
            self.pub_cmd1.publish(Int16(data=cmd1))
            self.pub_cmd2.publish(Int16(data=cmd2))
            self.pub_speedL.publish(Int16(data=speedL))
            self.pub_speedR.publish(Int16(data=speedR))
            # 电池电压：0.01V → V
            self.pub_battery.publish(Float32(data=bat / 100.0))
            # 温度：0.1°C → °C
            self.pub_temp.publish(Float32(data=temp / 10.0))
            self.pub_led.publish(Int16(data=led))

            self.get_logger().info(
                f'📥 反馈 | Cmd1:{cmd1} Cmd2:{cmd2} '
                f'SpdL:{speedL} SpdR:{speedR} '
                f'Bat:{bat/100.0:.2f}V Temp:{temp/10.0:.1f}°C LED:{led}'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'解包错误: {e}')
            return False
    
    def __del__(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()