# R550PLUS全向轮机器人配置
# 根据实际硬件配置修改

# ===== 串口配置 =====
# 底盘控制器统一使用udev别名，和 base_serial.launch 保持一致
WHEELTEC_CONTROLLER_PORT = '/dev/wheeltec_controller'
WHEELTEC_CONTROLLER_BAUD = 115200

# 电流采集串口（4CH-Current传感器 - CH343芯片）
CURRENT_SENSOR_PORT = '/dev/ttyUSB1'
CURRENT_SENSOR_BAUD = 115200

# 高精度IMU串口（维特IMU - CP2102芯片)
IMU_SENSOR_PORT = '/dev/ttyUSB1'
IMU_SENSOR_BAUD = 115200

# ===== 数据帧配置 =====
# STM32控制器数据帧
MAIN_FRAME_HEADER = 0x7B  # 帧头
MAIN_FRAME_TAIL = 0x7D    # 帧尾

# 电流数据帧
CURRENT_FRAME_HEADER = 0xAA
CURRENT_FRAME_TAIL = 0x55

# ===== 机器人参数 =====
NUM_MOTORS = 4
SPEED_RATIO = 1000.0

# ===== 数据采集参数 =====
SAMPLE_INTERVAL = 0.01
LOG_DIRECTORY = '/home/wheeltec/R550PLUS_data_collect/log'
PRINT_LOG = 1

# ===== IMU参数 =====
ACCEL_RATIO = 1671.84
GYROSCOPE_RATIO = 0.00026644

# ===== 电流采集参数 =====
ADC_BASELINE = 1990
ADC_TO_CURRENT = 0.00805664
