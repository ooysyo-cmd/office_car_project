# analyze_imu.py
import numpy as np

data = np.load('imu_static.npz')
gyro  = data['gyro']   # shape: (N, 3)
accel = data['accel']  # shape: (N, 3)

# 计算协方差矩阵（3x3）
gyro_cov  = np.cov(gyro.T)   # .T 让每行是一个变量
accel_cov = np.cov(accel.T)

print("=== 陀螺仪协方差矩阵 ===")
print(gyro_cov)
print("\n=== 加速度计协方差矩阵 ===")
print(accel_cov)

# ROS 需要的是 row-major 展开的 9 个值
print("\n=== 直接复制到 YAML 的格式 ===")
print("angular_velocity_covariance:")
print(" ", list(gyro_cov.flatten()))
print("linear_acceleration_covariance:")
print(" ", list(accel_cov.flatten()))

# 同时打印各轴标准差，帮助判断数据质量
print("\n=== 各轴噪声标准差（σ）===")
print(f"Gyro  σ: x={gyro.std(0)[0]:.6f}, y={gyro.std(0)[1]:.6f}, z={gyro.std(0)[2]:.6f} rad/s")
print(f"Accel σ: x={accel.std(0)[0]:.6f}, y={accel.std(0)[1]:.6f}, z={accel.std(0)[2]:.6f} m/s²")