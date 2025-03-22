import serial
import time
import keyboard
import threading
import struct

# 串口配置
PORT = 'COM6'  
BAUDRATE = 115200
PARITY = serial.PARITY_ODD
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# 控制变量
speed = 0       # 12位 (最大值4095)
dirs = 0        # 4位 (0-15，表示方向组合)
speed5 = 0      # 4位 (最大值15)
speed_snail = 0 # 4位 (最大值15)
angle_servo = 128 # 8位 (0-255)

# 速度增量
SPEED_INCREMENT = 50
ANGLE_INCREMENT = 5
MOTOR5_INCREMENT = 1
SNAIL_INCREMENT = 1

# 方向映射
DIR_FORWARD = 0x01   # 前
DIR_BACKWARD = 0x02  # 后
DIR_LEFT = 0x04      # 左
DIR_RIGHT = 0x08     # 右

# 串口初始化
ser = None
is_running = True

def connect_serial():
    global ser
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            parity=PARITY,
            stopbits=STOPBITS,
            bytesize=BYTESIZE,
            timeout=1
        )
        print(f"成功连接到 {PORT}")
        return True
    except Exception as e:
        print(f"连接错误: {e}")
        return False

def send_command():
    global speed, dirs, speed5, speed_snail, angle_servo
    
    # 构建32位控制值
    control_value = (speed & 0xFFF) << 20
    control_value |= (dirs & 0x0F) << 16
    control_value |= (speed5 & 0x0F) << 12
    control_value |= (speed_snail & 0x0F) << 8
    control_value |= (angle_servo & 0xFF)
    
    # 将控制值转换为小端序字节
    data_bytes = struct.pack("<BI", 0xAB, control_value)
    
    # 打印调试信息
    print(f"发送: 帧头=AB, 数据={control_value:08X}, 方向={dirs:X}, 速度={speed}, 舵机={angle_servo}")
    print(f"原始字节: {' '.join([f'{b:02X}' for b in data_bytes])}")
    
    if ser and ser.is_open:
        ser.write(data_bytes)

def keyboard_listener():
    global speed, dirs, speed5, speed_snail, angle_servo, is_running
    
    print("键盘控制说明:")
    print("W/A/S/D - 控制方向 (前/左/后/右)")
    print("↑/↓ - 增加/减少速度")
    print("←/→ - 调整舵机角度")
    print("Q/E - 增加/减少电机5速度")
    print("Z/X - 增加/减少蜗轮速度")
    print("空格 - 紧急停止 (所有电机速度归零)")
    print("ESC - 退出程序")
    
    while is_running:
        # 方向控制 (WASD)
        new_dirs = 0
        if keyboard.is_pressed('w'):
            new_dirs |= DIR_FORWARD
        if keyboard.is_pressed('s'):
            new_dirs |= DIR_BACKWARD
        if keyboard.is_pressed('a'):
            new_dirs |= DIR_LEFT
        if keyboard.is_pressed('d'):
            new_dirs |= DIR_RIGHT
        
        # 只在方向改变时更新
        if new_dirs != dirs:
            dirs = new_dirs
            send_command()
        
        # 速度控制
        if keyboard.is_pressed('up'):
            speed = min(speed + SPEED_INCREMENT, 4095)
            send_command()
            time.sleep(0.1)  # 防抖
        elif keyboard.is_pressed('down'):
            speed = max(speed - SPEED_INCREMENT, 0)
            send_command()
            time.sleep(0.1)  # 防抖
            
        # 舵机角度控制
        if keyboard.is_pressed('right'):
            angle_servo = min(angle_servo + ANGLE_INCREMENT, 255)
            send_command()
            time.sleep(0.1)  # 防抖
        elif keyboard.is_pressed('left'):
            angle_servo = max(angle_servo - ANGLE_INCREMENT, 0)
            send_command()
            time.sleep(0.1)  # 防抖
            
        # 电机5控制
        if keyboard.is_pressed('q'):
            speed5 = min(speed5 + MOTOR5_INCREMENT, 15)
            send_command()
            time.sleep(0.1)  # 防抖
        elif keyboard.is_pressed('e'):
            speed5 = max(speed5 - MOTOR5_INCREMENT, 0)
            send_command()
            time.sleep(0.1)  # 防抖
            
        # # 蜗轮控制
        # if keyboard.is_pressed('z'):
        #     speed_snail = min(speed_snail + SNAIL_INCREMENT, 15)
        #     send_command()
        #     time.sleep(0.1)  # 防抖
        # elif keyboard.is_pressed('x'):
        #     speed_snail = max(speed_snail - SNAIL_INCREMENT, 0)
        #     send_command()
        #     time.sleep(0.1)  # 防抖
            
        # 紧急停止
        if keyboard.is_pressed('space'):
            speed = 0
            speed5 = 0
            speed_snail = 0
            dirs = 0
            send_command()
            print("紧急停止!")
            time.sleep(0.3)  # 防抖
            
        # 退出程序
        if keyboard.is_pressed('esc'):
            is_running = False
            print("程序退出")

        # 蜗轮控制 - 只用于控制是否发弹
        if keyboard.is_pressed('z'):
            speed_snail = 1  # 启动发弹
            send_command()
            print("开始发弹")
            time.sleep(0.1)  # 防抖
        elif keyboard.is_pressed('x'):
            speed_snail = 0  # 停止发弹
            send_command()
            print("停止发弹")
            time.sleep(0.1)  # 防抖
            
        time.sleep(0.01)  # 减少CPU使用率

def main():
    if connect_serial():
        try:
            # 启动键盘监听线程
            kb_thread = threading.Thread(target=keyboard_listener)
            kb_thread.daemon = True
            kb_thread.start()
            
            # 初始化发送一次命令
            send_command()
            
            # 主线程保持活动直到键盘监听线程退出
            while is_running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("程序被用户中断")
        finally:
            if ser and ser.is_open:
                ser.close()
                print("串口已关闭")

if __name__ == "__main__":
    main()