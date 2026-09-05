#!/usr/bin/env python3
import math
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class RobotGUIController(Node):
    def __init__(self):
        super().__init__('robot_gui_controller')
        
        # 1. 创建 6 个机械臂关节 Publisher
        self.arm_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/arm/joint{i}/cmd_pos', 10)
            self.arm_pubs.append(pub)

        # 2. 创建底盘 cmd_vel Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 速度控制变量
        self.linear_speed = 0.5   # 默认线速度 (m/s)
        self.angular_speed = 0.8  # 默认角速度 (rad/s)
        self.pressed_keys = set() # 记录当前按下的键盘按键

        # 初始化 Tkinter 界面
        self.root = tk.Tk()
        self.root.title("Scout 小车底盘 + 机械臂一体化图形控制器")
        self.root.geometry("520x720")

        # 创建选项卡/分隔区域
        self.setup_chassis_ui()
        
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)
        
        self.setup_arm_ui()

        # 绑定键盘事件（用于 WASD 遥控）
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    # ==================== 底盘控制 UI 构建 ====================
    def setup_chassis_ui(self):
        chassis_frame = ttk.LabelFrame(self.root, text=" 🚗 底盘控制 (支持键盘 WASD 遥控) ")
        chassis_frame.pack(fill='x', padx=15, pady=10)

        # 速度调节滑块
        speed_frame = ttk.Frame(chassis_frame)
        speed_frame.pack(fill='x', padx=10, pady=5)

        # 线速度滑块
        ttk.Label(speed_frame, text="线速度 (m/s):", width=12).grid(row=0, column=0)
        self.lin_scale = ttk.Scale(speed_frame, from_=0.1, to=2.0, value=self.linear_speed,
                                   command=self.update_speeds)
        self.lin_scale.grid(row=0, column=1, sticky='ew', padx=5)
        self.lin_label = ttk.Label(speed_frame, text=f"{self.linear_speed:.1f}", width=5)
        self.lin_label.grid(row=0, column=2)

        # 角速度滑块
        ttk.Label(speed_frame, text="角速度 (rad/s):", width=12).grid(row=1, column=0)
        self.ang_scale = ttk.Scale(speed_frame, from_=0.1, to=2.5, value=self.angular_speed,
                                   command=self.update_speeds)
        self.ang_scale.grid(row=1, column=1, sticky='ew', padx=5)
        self.ang_label = ttk.Label(speed_frame, text=f"{self.angular_speed:.1f}", width=5)
        self.ang_label.grid(row=1, column=2)

        speed_frame.columnconfigure(1, weight=1)

        # 方向控制按钮面板 (九宫格布局)
        btn_frame = ttk.Frame(chassis_frame)
        btn_frame.pack(pady=10)

        btn_w = ttk.Button(btn_frame, text="前 (W)", width=8, 
                           command=lambda: self.publish_twist(self.linear_speed, 0.0))
        btn_w.grid(row=0, column=1, pady=2)

        btn_a = ttk.Button(btn_frame, text="左转 (A)", width=8, 
                           command=lambda: self.publish_twist(0.0, self.angular_speed))
        btn_a.grid(row=1, column=0, padx=2)

        btn_stop = ttk.Button(btn_frame, text="🛑 急停 (Space)", width=10, command=self.stop_chassis)
        btn_stop.grid(row=1, column=1, padx=2)

        btn_d = ttk.Button(btn_frame, text="右转 (D)", width=8, 
                           command=lambda: self.publish_twist(0.0, -self.angular_speed))
        btn_d.grid(row=1, column=2, padx=2)

        btn_s = ttk.Button(btn_frame, text="后 (S)", width=8, 
                           command=lambda: self.publish_twist(-self.linear_speed, 0.0))
        btn_s.grid(row=2, column=1, pady=2)

        # 提示标签
        tip_label = ttk.Label(chassis_frame, text="提示: 鼠标点击窗口后，直接按键盘 W/A/S/D 可平滑遥控", 
                              foreground="gray")
        tip_label.pack(pady=5)

    def update_speeds(self, _=None):
        self.linear_speed = float(self.lin_scale.get())
        self.angular_speed = float(self.ang_scale.get())
        self.lin_label.config(text=f"{self.linear_speed:.1f}")
        self.ang_label.config(text=f"{self.angular_speed:.1f}")

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop_chassis(self):
        self.pressed_keys.clear()
        self.publish_twist(0.0, 0.0)

    # ==================== 键盘遥控逻辑 (WASD) ====================
    def on_key_press(self, event):
        key = event.char.lower()
        if key in ['w', 'a', 's', 'd', ' ']:
            self.pressed_keys.add(key)
            self.process_keyboard_control()

    def on_key_release(self, event):
        key = event.char.lower()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self.process_keyboard_control()

    def process_keyboard_control(self):
        if ' ' in self.pressed_keys or not self.pressed_keys:
            self.publish_twist(0.0, 0.0)
            return

        lin = 0.0
        ang = 0.0

        if 'w' in self.pressed_keys:
            lin += self.linear_speed
        if 's' in self.pressed_keys:
            lin -= self.linear_speed
        if 'a' in self.pressed_keys:
            ang += self.angular_speed
        if 'd' in self.pressed_keys:
            ang -= self.angular_speed

        self.publish_twist(lin, ang)

    # ==================== 机械臂控制 UI 构建 ====================
    def setup_arm_ui(self):
        arm_frame = ttk.LabelFrame(self.root, text=" 🦾 机械臂 6 关节位置控制 ")
        arm_frame.pack(fill='both', expand=True, padx=15, pady=5)

        self.sliders = []
        for i in range(6):
            f = ttk.Frame(arm_frame)
            f.pack(fill='x', padx=10, pady=4)

            ttk.Label(f, text=f"Joint {i+1} (°):", width=10).pack(side='left')

            slider = ttk.Scale(
                f, from_=-180, to=180, orient='horizontal',
                command=lambda val, idx=i: self.on_arm_slider_move(idx, val)
            )
            slider.set(0)
            slider.pack(side='left', fill='x', expand=True, padx=8)

            val_label = ttk.Label(f, text="0.0°", width=8)
            val_label.pack(side='right')

            self.sliders.append((slider, val_label))

        reset_btn = ttk.Button(arm_frame, text="机械臂一键归零 (0°)", command=self.reset_arm)
        reset_btn.pack(pady=10)

    def on_arm_slider_move(self, idx, val):
        deg = float(val)
        rad = math.radians(deg)
        self.sliders[idx][1].config(text=f"{deg:.1f}°")

        msg = Float64()
        msg.data = rad
        self.arm_pubs[idx].publish(msg)

    def reset_arm(self):
        for idx, (slider, _) in enumerate(self.sliders):
            slider.set(0)
            self.on_arm_slider_move(idx, 0)

    # 主循环更新
    def update_loop(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(20, self.update_loop)

def main():
    rclpy.init()
    node = RobotGUIController()
    node.update_loop()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()