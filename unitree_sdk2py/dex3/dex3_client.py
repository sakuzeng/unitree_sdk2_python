"""
Dex3 灵巧手控制客户端

该模块提供了对宇树 Dex3-1 力控灵巧手的控制接口，基于官方DDS协议规范。
- 7 自由度关节控制（3指 + 拇指旋转）；9个传感器，每个传感器是3*4的点阵数据
- 触觉传感器数据读取  
- 左右手支持
- 多种控制模式（位置、速度、扭矩）
- 完整的初始化和控制序列
"""

import time
import threading
import contextlib
import json
import traceback
import math
import socket
import sys
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize


@dataclass
class Dex3Config:
    """Dex3 灵巧手配置参数"""
    # 关节限位 (单位: rad) - 基于官方URDF限位
    joint_limits_left: List[Tuple[float, float]] = None
    joint_limits_right: List[Tuple[float, float]] = None
    
    # 控制增益默认值 - 借鉴arm_client的参数设计
    default_kp_1: float = 0.5
    default_kp: float = 1.5
    default_kd: float = 0.1
    default_dq: float = 0.0
    default_tau_ff: float = 0.0
    
    # 时间控制参数 - 借鉴arm_client的控制逻辑
    control_dt: float = 0.02  # 控制周期 20ms
    max_joint_velocity: float = 0.5  # 最大关节速度 rad/s
    weight_rate: float = 0.2  # 权重变化率
    
    # 安全参数
    max_torque: float = 2.0  # 最大扭矩 (N·m)
    timeout_enable: bool = True
    
    def __post_init__(self):
        if self.joint_limits_left is None:
            # 左手关节限位 - 基于URDF精确值，与C++示例一致
            self.joint_limits_left = [
                (-1.0472, 1.0472),  # thumb_0: 左拇指外展/内收 (Y轴)
                (-0.6109, 1.0472),  # thumb_1: 左拇指屈曲（第一指节） (Z轴)
                (0.0, 1.7453),      # thumb_2: 左拇指屈曲（第二指节） (Z轴)
                (-1.5708, 0.0),     # middle_0: 左中指屈曲（基部） (Z轴)
                (-1.7453, 0.0),     # middle_1: 左中指屈曲（指尖） (Z轴)
                (-1.5708, 0.0),     # index_0: 左食指屈曲（基部） (Z轴)
                (-1.7453, 0.0),     # index_1: 左食指屈曲（指尖） (Z轴)
            ]
        
        if self.joint_limits_right is None:
            # 右手关节限位 - 基于URDF精确值，与C++示例一致
            self.joint_limits_right = [
                (-1.0472, 1.0472),  # thumb_0: 右拇指外展/内收 (Y轴)
                (-1.0472, 0.6109),  # thumb_1: 右拇指屈曲（第一指节） (Z轴)
                (-1.7453, 0.0),     # thumb_2: 右拇指屈曲（第二指节） (Z轴)
                (0.0, 1.5708),      # middle_0: 右中指屈曲（基部） (Z轴)
                (0.0, 1.7453),      # middle_1: 右中指屈曲（指尖） (Z轴)
                (0.0, 1.5708),      # index_0: 右食指屈曲（基部） (Z轴)
                (0.0, 1.7453),      # index_1: 右食指屈曲（指尖） (Z轴)
            ]


class Dex3Client:
    """
    Dex3 灵巧手控制客户端
    
    基于官方DDS协议规范实现，与C++版本完全兼容。
    借鉴arm_client的设计，提供完整的初始化和控制流程。
    
    Args:
        hand (str): 手的类型，"left" 或 "right"
        interface (str): 网络接口名称，如 "eth0"
        config (Dex3Config, optional): 配置参数
    
    Example:
        # 创建右手控制器
        dex3 = Dex3Client(hand="right", interface="eth0")
        
        # 执行完整的控制序列
        dex3.execute_basic_sequence()
        
        # 设置手势
        dex3.set_gesture("open")
    """
    
    def __init__(
        self, 
        hand: str = "right", 
        interface: str = "eth0",
        config: Optional[Dex3Config] = None
    ):
        if hand not in ["left", "right"]:
            raise ValueError("hand 必须是 'left' 或 'right'")
        
        self.hand = hand
        self.config = config or Dex3Config()
        self._interface = interface
        
        # DDS 通信设置 - 基于官方规范
        self._cmd_topic = f"rt/dex3/{hand}/cmd"
        self._state_topic = f"rt/dex3/{hand}/state"
        
        # 发布者和订阅者
        self._cmd_publisher: Optional[ChannelPublisher] = None
        self._state_subscriber: Optional[ChannelSubscriber] = None
        
        # 状态缓存
        self._latest_state: Optional[Any] = None
        self._state_lock = threading.Lock()
        
        # 电机和传感器数量常量 - 与C++版本一致
        self.MOTOR_MAX = 7
        self.SENSOR_MAX = 9
        
        # 控制参数 - 借鉴arm_client的控制逻辑
        # self._weight = 0.0
        # self._delta_weight = self.config.weight_rate * self.config.control_dt
        self._max_joint_delta = self.config.max_joint_velocity * self.config.control_dt
        self._sleep_duration = self.config.control_dt
        
        # 当前期望位置状态 - 重要：用于跟踪运动状态
        self._current_jpos_des = [0.0] * self.MOTOR_MAX
        
        # 预定义位置 - 基于实际弧度值
        self._init_pos = [0.0] * self.MOTOR_MAX
        self._nature_pos = self._get_nature_position()
        
        # 旋转电机状态变量 - 模拟C++静态变量
        self._rotate_count = 1
        self._rotate_dir = 1
        
        # 初始化DDS连接
        self._init_dds_connection()
    # 一会儿print
    def _get_nature_position(self) -> List[float]:
        """获取自然位置 - 基于手部类型的安全休息位置"""
        if self.hand == "left":
            # 左手自然位置 - 轻微弯曲的安全姿态
            return [0.2, -0.3, 0.5, -0.5, -0.8, -0.5, -0.8]
        else:
            # 右手自然位置 - 轻微弯曲的安全姿态  
            return [0.2, 0.3, -0.5, 0.5, 0.8, 0.5, 0.8]
    
    def _init_dds_connection(self):
        """初始化DDS连接 - 基于官方规范"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
            
            # 初始化DDS工厂 - 与C++版本一致
            if self._interface:
                ChannelFactoryInitialize(0, self._interface)
                print(f"[Dex3] 初始化DDS连接 - 接口: {self._interface}")
                print(f"[Dex3] 提示: 请确保网络接口配置正确")
            
            # 创建发布者和订阅者
            self._cmd_publisher = ChannelPublisher(self._cmd_topic, HandCmd_)
            self._cmd_publisher.Init()
            
            self._state_subscriber = ChannelSubscriber(self._state_topic, HandState_)
            self._state_subscriber.Init(self._state_callback, 10)
            
            print(f"[Dex3] 成功连接到 {self.hand} 手")
            print(f"[Dex3] 命令话题: {self._cmd_topic}")
            print(f"[Dex3] 状态话题: {self._state_topic}")
            print(f"[Dex3] 关节数量: {self.MOTOR_MAX}")
            
            # 等待连接稳定
            time.sleep(1.0)
            
        except Exception as e:
            print(f"[Dex3] DDS连接失败: {e}")
            print(f"[Dex3] 请检查网络接口配置和依赖库安装")
            raise

    def _state_callback(self, msg):
        """状态消息回调 - 基于官方HandState_结构"""
        with self._state_lock:
            self._latest_state = msg

    def read_state(self, timeout: float = 1.0) -> Optional[Any]:
        """
        读取灵巧手状态 - 基于官方HandState_结构
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            HandState_ 消息或 None
        """
        start_time = time.time()
        
        # 先等待一段时间让数据稳定
        time.sleep(0.1)
        
        while time.time() - start_time < timeout:
            with self._state_lock:
                if self._latest_state is not None:
                    return self._latest_state
            time.sleep(0.01)  # 减少CPU占用
        
        return None
    
    def _get_joint_limits(self) -> List[Tuple[float, float]]:
        """获取当前手的关节限位"""
        if self.hand == "left":
            return self.config.joint_limits_left
        else:
            return self.config.joint_limits_right
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制值在指定范围内 - 借鉴arm_client的clamp函数"""
        return max(min_val, min(max_val, value))
    
    def _create_hand_command(
        self,
        positions: List[float],
        velocities: Optional[List[float]] = None,
        torques: Optional[List[float]] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None
    ):
        """创建手部控制命令 - 借鉴arm_client的命令创建逻辑"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            if len(positions) != self.MOTOR_MAX:
                raise ValueError(f"位置数量({len(positions)})与关节数({self.MOTOR_MAX})不匹配")
            
            # 设置默认值
            velocities = velocities or [self.config.default_dq] * self.MOTOR_MAX
            torques = torques or [self.config.default_tau_ff] * self.MOTOR_MAX
            kp = kp if kp is not None else self.config.default_kp
            kd = kd if kd is not None else self.config.default_kd
            
            # 创建7个电机命令
            motor_cmds = []
            for i in range(self.MOTOR_MAX):
                motor_cmd = MotorCmd_(
                    mode=0,  # Enable
                    q=float(positions[i]),
                    dq=float(velocities[i]),
                    tau=float(torques[i]),
                    kp=float(kp),
                    kd=float(kd),
                    reserve=0  # 单个uint32
                )
                motor_cmds.append(motor_cmd)
            
            # 创建HandCmd_ - reserve是4个元素的数组
            hand_cmd = HandCmd_(
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0]  # 4个元素的数组
            )
            
            return hand_cmd
        
        except Exception as e:
            print(f"[Dex3] 创建命令消息失败: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _publish_command(self, cmd) -> bool:
        """发布命令消息 - 借鉴arm_client的发布逻辑"""
        if self._cmd_publisher is None:
            print("[Dex3] 错误: 发布者未初始化")
            return False
        
        try:
            if cmd is None:
                print("[Dex3] 错误: 命令对象为空")
                return False
            
            # 验证命令结构
            if not hasattr(cmd, 'motor_cmd'):
                print("[Dex3] 错误: 命令缺少motor_cmd字段")
                return False
            
            # 发布命令
            self._cmd_publisher.Write(cmd)
            return True
            
        except Exception as e:
            print(f"[Dex3] 发布命令失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def smooth_transition(
        self,
        start_positions: List[float],
        target_positions: List[float],
        duration: float,
        description: str = ""
    ) -> bool:
        """
        平滑过渡到目标位置 - 完全借鉴arm_client的控制逻辑
        
        这个版本使用与arm_client完全相同的控制算法：
        1. 使用current_jpos_des跟踪当前期望位置
        2. 每步限制最大变化量
        3. 逐步趋向目标位置
        """
        print(f"[Dex3] {description}...")

        time_steps = int(duration / self.config.control_dt)

        # 初始化当前期望位置为起始位置
        self._current_jpos_des = start_positions.copy()

        for i in range(time_steps):
            # 更新期望位置 - 与arm_client完全一致的逻辑
            for j in range(len(self._current_jpos_des)):
                # 计算位置差
                delta = target_positions[j] - self._current_jpos_des[j]
                # 限制每步的最大变化量
                delta = self._clamp(delta, -self._max_joint_delta, self._max_joint_delta)
                # 更新当前期望位置
                self._current_jpos_des[j] += delta

            # 创建并发送命令
            cmd = self._create_hand_command(self._current_jpos_des)
            if not self._publish_command(cmd):
                return False

            # 延时 - 对应C++的std::this_thread::sleep_for
            time.sleep(self._sleep_duration)

        print(f"[Dex3] {description}完成")
        return True
    
    def initialize_hand(self, timeout: float = 5.0) -> bool:
        """
        初始化手部到自然位置 - 完全借鉴arm_client的权重过渡逻辑
        
        使用与arm_client相同的初始化算法：
        1. 逐步增加权重从0到1（虽然Dex3不直接使用权重，但保持相同的控制节奏）
        2. 同时进行位置插值到自然位置
        """
        print("[Dex3] 按回车键开始初始化灵巧手...")
        input("按回车键继续...")

        # 获取当前关节位置
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            print("[Dex3] 无法读取当前关节位置")
            print("[Dex3] 请检查灵巧手连接和网络配置")
            return False
        print(f"[Dex3] 当前关节位置: {[f'{pos:.3f}' for pos in current_positions]}")

        # 权重过渡初始化 - 借鉴arm_client的逻辑
        print("[Dex3] 初始化灵巧手中...")
        init_time = 5.0
        time_steps = int(init_time / self.config.control_dt)
        # self._weight = 0.0  # 重置权重

        start_time = time.time()
        for i in range(time_steps):
            # 逐步增加权重（用于控制节奏）
            # self._weight += self._delta_weight
            # self._weight = self._clamp(self._weight, 0.0, 1.0)
            # if i % 50 == 0:  # 每秒打印一次
            #     print(f"初始化进度: {self._weight:.3f}")

            # 计算过渡位置 - 线性插值
            phase = 1.0 if i == time_steps - 1 else float(i) / time_steps
            transition_positions = [
                self._nature_pos[j] * phase + current_positions[j] * (1 - phase)
                for j in range(self.MOTOR_MAX)
            ]

            # 创建并发送命令
            cmd = self._create_hand_command(transition_positions)
            if not self._publish_command(cmd):
                return False

            # 精确延时
            expected_time = start_time + i * self._sleep_duration
            if time.time() < expected_time:
                time.sleep(expected_time - time.time())

        print("[Dex3] 灵巧手初始化完成")
        return True
    
    # def set_joint_angles(
    #     self, 
    #     angles: List[float], 
    #     kp: Optional[float] = None, 
    #     kd: Optional[float] = None,
    #     velocities: Optional[List[float]] = None,
    #     torques: Optional[List[float]] = None,
    #     check_limits: bool = True
    # ) -> bool:
    #     """
    #     设置关节角度 - 移除归一化，直接使用弧度值
        
    #     Args:
    #         angles: 关节角度列表（弧度）
    #         kp: 位置增益
    #         kd: 速度增益
    #         velocities: 速度列表
    #         torques: 扭矩列表
    #         check_limits: 是否检查关节限位
    #     """
    #     if len(angles) != self.MOTOR_MAX:
    #         print(f"[Dex3] 错误: 需要{self.MOTOR_MAX}个关节角度，得到{len(angles)}个")
    #         return False
        
    #     kp = kp if kp is not None else self.config.default_kp
    #     kd = kd if kd is not None else self.config.default_kd
    #     velocities = velocities or [0.0] * self.MOTOR_MAX
    #     torques = torques or [0.0] * self.MOTOR_MAX
        
    #     # 限位检查 - 直接使用弧度值
    #     final_angles = angles.copy()
    #     if check_limits:
    #         limits = self._get_joint_limits()
    #         for i, (angle, (min_val, max_val)) in enumerate(zip(angles, limits)):
    #             if angle < min_val or angle > max_val:
    #                 print(f"[Dex3] 警告: 关节{i}角度{angle:.3f}rad超出限位[{min_val:.3f}, {max_val:.3f}]rad")
    #                 final_angles[i] = self._clamp(angle, min_val, max_val)
        
    #     try:
    #         # 创建并发送命令
    #         cmd = self._create_hand_command(
    #             final_angles, 
    #             velocities, 
    #             torques, 
    #             kp, 
    #             kd
    #         )
            
    #         success = self._publish_command(cmd)
    #         if success:
    #             print(f"[Dex3] {self.hand}手关节角度设置成功")
    #             print(f"[Dex3] 角度: {[f'{a:.3f}' for a in final_angles]} rad")
    #         else:
    #             print(f"[Dex3] {self.hand}手关节角度设置失败")
            
    #         return success
        
    #     except Exception as e:
    #         print(f"[Dex3] 设置关节角度失败: {e}")
    #         import traceback
    #         traceback.print_exc()
    #         return False
    

    
    def execute_hand_control_sequence(self) -> bool:
        """
        执行手部控制序列 - 借鉴arm_client的控制流程
        
        流程：
        1. 等待用户按Enter
        2. 执行手势序列
        """
        print("[Dex3] 按回车键开始手部控制...")
        input("按回车键继续...")
        
        print("[Dex3] 开始手部控制!")

        # 手势序列
        gestures_to_test = ["open", "closed", "pinch", "point", "peace", "ok", "rest"]
        
        for gesture_name in gestures_to_test:
            print(f"执行手势: {gesture_name}")
            input("按回车键继续...")
            
            angles = Dex3Gestures.get_gesture(gesture_name, self.hand)
            if angles:
                current_positions = self.get_current_joint_positions()
                if current_positions:
                    success = self.smooth_transition(
                        current_positions, 
                        angles, 
                        2.0,  # 每个手势2秒过渡时间
                        f"执行手势: {gesture_name}"
                    )
                    if not success:
                        return False
                else:
                    return False
            else:
                print(f"[Dex3] 手势 {gesture_name} 获取失败")
                return False
        
        return True
    
    def stop_control(self) -> bool:
        """
        停止控制 - 借鉴arm_client的退出逻辑，平滑到自然位置
        """
        print("[Dex3] 停止手部控制中...")
        input("按回车键继续...")

        # 获取当前关节位置
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            print("[Dex3] 无法读取当前关节位置")
            return False

        # 平滑过渡到自然位置
        success = self.smooth_transition(
            current_positions,
            self._nature_pos,
            5.0,
            "停止控制，返回自然位置"
        )

        print("[Dex3] 手部控制已停止")
        return success
    
    def execute_basic_sequence(self) -> bool:
        """
        执行基础控制序列 - 完全借鉴arm_client的完整流程
        
        这是与arm_client完全对应的主要控制函数
        """
        print("=== Dex3 灵巧手控制演示 ===")
        print("警告: 确保灵巧手处于安全测试环境")
        print(f"控制手部: {self.hand}")
        
        try:
            # 1. 手部复位 - 对应arm_client的init部分
            if not self.initialize_hand():
                print("[Dex3] 初始化失败")
                return False
            
            # 2. 执行控制序列 - 对应arm_client的控制序列
            if not self.execute_hand_control_sequence():
                print("[Dex3] 控制序列执行失败")
                return False
            
            # 3. 退出控制 - 对应arm_client的stop control
            if not self.stop_control():
                print("[Dex3] 停止控制失败")
                return False
            
            print("[Dex3] 基础序列执行完成")
            return True
            
        except Exception as e:
            print(f"[Dex3] 执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def set_gesture(self, gesture_name: str) -> bool:
        """设置手势到预定义姿态 - 使用平滑过渡"""
        angles = Dex3Gestures.get_gesture(gesture_name, self.hand)
        if angles is None:
            return False
        
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        return self.smooth_transition(
            current_positions,
            angles,
            2.0,
            f"设置手势: {gesture_name}"
        )

    def set_joint_positions(
        self,
        positions: List[float],
        duration: float = 5.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None
    ) -> bool:
        """
        设置关节位置 - 带平滑过渡，借鉴arm_client的设计
        
        Args:
            positions: 关节位置列表（弧度）
            duration: 执行时间(秒)
            kp: 位置增益
            kd: 速度增益
        
        Returns:
            bool: 设置是否成功
        """
        if len(positions) != self.MOTOR_MAX:
            print(f"[Dex3] 错误: 需要{self.MOTOR_MAX}个关节位置，得到{len(positions)}个")
            return False
        
        # 关节限位检查 - 基于URDF文件
        limits = self._get_joint_limits()
        clamped_positions = []
        for i, (pos, (min_val, max_val)) in enumerate(zip(positions, limits)):
            if pos < min_val or pos > max_val:
                print(f"[Dex3] 警告: 关节{i}位置{pos:.3f}rad超出限位[{min_val:.3f}, {max_val:.3f}]rad")
                clamped_pos = self._clamp(pos, min_val, max_val)
                clamped_positions.append(clamped_pos)
            else:
                clamped_positions.append(pos)
        
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        return self.smooth_transition(
            current_positions,
            clamped_positions,
            duration,
            "设置关节位置"
        )
    
    def get_current_joint_positions(self, timeout: float = 2.0) -> Optional[List[float]]:
        """获取当前关节位置 - 借鉴arm_client的状态读取逻辑"""
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= self.MOTOR_MAX:
            try:
                return [float(ms.q) for ms in state.motor_state[:self.MOTOR_MAX]]
            except Exception as e:
                print(f"[Dex3] 解析关节角度失败: {e}")
        return None
    
    def get_joint_velocities(self, timeout: float = 1.0) -> Optional[List[float]]:
        """获取当前关节速度"""
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= self.MOTOR_MAX:
            try:
                return [float(ms.dq) for ms in state.motor_state[:self.MOTOR_MAX]]
            except Exception as e:
                print(f"[Dex3] 解析关节速度失败: {e}")
        return None
    
    def get_joint_torques(self, timeout: float = 1.0) -> Optional[List[float]]:
        """获取当前关节扭矩"""
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= self.MOTOR_MAX:
            try:
                return [float(ms.tau_est) for ms in state.motor_state[:self.MOTOR_MAX]]
            except Exception as e:
                print(f"[Dex3] 解析关节扭矩失败: {e}")
        return None
    
    def get_imu_data(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        获取IMU数据 - 基于官方IMUState_结构
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            Dict: IMU数据字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'imu_state'):
            try:
                imu = state.imu_state
                return {
                    'quaternion': list(imu.quaternion),         # QwQxQyQz
                    'gyroscope': list(imu.gyroscope),           # 角速度 omega_xyz
                    'accelerometer': list(imu.accelerometer),   # 加速度 acc_xyz
                    'rpy': list(imu.rpy),                       # 欧拉角
                    'temperature': imu.temperature              # IMU温度
                }
            except Exception as e:
                print(f"[Dex3] 解析IMU数据失败: {e}")
        return None
    
    def get_power_info(self, timeout: float = 1.0) -> Optional[Dict[str, float]]:
        """
        获取电源信息 - 基于官方HandState_结构
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            Dict: 电源信息字典
        """
        state = self.read_state(timeout)
        if state:
            try:
                return {
                    'voltage': float(state.power_v),  # 电源电压
                    'current': float(state.power_a),  # 电源电流
                    'system_v': float(getattr(state, 'system_v', 0.0)),  # 系统电压
                    'device_v': float(getattr(state, 'device_v', 0.0))   # 设备电压
                }
            except Exception as e:
                print(f"[Dex3] 解析电源信息失败: {e}")
        return None
    
    def print_joint_states(self):
        """打印当前关节状态 - 调试用，直接显示弧度值"""
        positions = self.get_current_joint_positions()
        velocities = self.get_joint_velocities()
        torques = self.get_joint_torques()
        
        if positions:
            print(f"[Dex3] {self.hand}手当前关节状态:")
            for i in range(self.MOTOR_MAX):
                pos = positions[i] if positions else 0.0
                vel = velocities[i] if velocities else 0.0
                tor = torques[i] if torques else 0.0
                print(f"  关节{i:2d}: 位置={pos:6.3f}rad, "
                      f"速度={vel:6.3f}rad/s, "
                      f"扭矩={tor:6.3f}Nm")
        else:
            print("[Dex3] 无法获取关节状态")

    def get_pressure_data(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        获取触觉传感器数据 - 基于官方PressSensorState_结构
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            Dict: 触觉传感器数据字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'press_sensor_state'):
            try:
                pressure_data = {}
                for i, sensor in enumerate(state.press_sensor_state):
                    # 基于官方PressSensorState_结构 - 每个传感器有12个压力值和温度值
                    pressure_data[f'sensor_{i}'] = {
                        'pressure': list(sensor.pressure),      # 12个压力值
                        'temperature': list(sensor.temperature) # 12个温度值
                    }
                return pressure_data
            except Exception as e:
                print(f"[Dex3] 解析压力数据失败: {e}")
        return None
    
    def visualize_sensor_data(
        self,
        timeout: float = 1.0,
        save_path: Optional[str] = None,
        show: bool = True,
        interval: float = 0.5,  # 刷新间隔(秒)
        duration: Optional[float] = None  # 总持续时间(秒)，None表示无限
    ) -> bool:
        """
        实时可视化显示所有9个传感器的压力数据 - 生成3x3热图网格
        
        Args:
            timeout: 读取状态的超时时间(秒)
            save_path: 保存图像路径，None表示不保存
            show: 是否显示图像
            interval: 刷新间隔(秒)
            duration: 总持续时间(秒)，None表示无限
        
        Returns:
            bool: 可视化是否成功
        """
        start_time = time.time()
        pressure_data = self.get_pressure_data(timeout)
        if not pressure_data:
            print("[Dex3] 未获取到压力数据，无法可视化")
            return False
        # 创建3x3子图网格
        plt.ion()  # 启用交互模式
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        axes = axes.flatten()  # 将2D数组展平为1D，便于索引
        
        # 初始化图像
        images = []  # 存储imshow对象以便更新
        for i in range(9):
            sensor_key = f'sensor_{i}'
            if sensor_key in pressure_data:
                pressures = np.array(pressure_data[sensor_key]['pressure']).reshape(3, 4)
                im = axes[i].imshow(pressures, cmap='hot', aspect='equal', animated=True)
                images.append(im)
                axes[i].set_title(f'传感器 {i} 压力热图')
                axes[i].set_xlabel('列 (Column)')
                axes[i].set_ylabel('行 (Row)')
                plt.colorbar(im, ax=axes[i], label='压力值')
            else:
                axes[i].text(0.5, 0.5, f'无数据\n(sensor_{i})', 
                             ha='center', va='center', transform=axes[i].transAxes)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"[Dex3] 初始图像已保存到 {save_path}")
        
        if show:
            plt.tight_layout()
            while duration is None or (time.time() - start_time) < duration:
                pressure_data = self.get_pressure_data(timeout)
                if not pressure_data:
                    print("[Dex3] 未能更新压力数据")
                    break
                
                for i in range(9):
                    sensor_key = f'sensor_{i}'
                    if sensor_key in pressure_data and len(images) > i:
                        pressures = np.array(pressure_data[sensor_key]['pressure']).reshape(3, 4)
                        images[i].set_array(pressures)
                
                plt.draw()
                plt.pause(interval)  # 控制刷新间隔
                
                # 可选：检查退出条件（如按键）
                if plt.waitforbuttonpress(timeout=0.1):  # 检测键盘输入
                    break
            
            plt.ioff()  # 关闭交互模式
            plt.show(block=False)  # 非阻塞显示
        
        return True
    


    # 保留原有的电机控制方法，但使用新的命令创建逻辑
    # def stop_motors(self) -> bool:
    #     """停止所有电机"""
    #     try:
    #         from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
    #         # 创建7个电机命令
    #         motor_cmds = []
    #         for i in range(self.MOTOR_MAX):
    #             motor_cmd = MotorCmd_(
    #                 mode=0,  # Disable
    #                 q=0.0,
    #                 dq=0.0,
    #                 tau=0.0,
    #                 kp=0.0,
    #                 kd=0.0,
    #                 reserve=0  # 单个uint32
    #             )
    #             motor_cmds.append(motor_cmd)
            
    #         hand_cmd = HandCmd_(
    #             motor_cmd=motor_cmds,
    #             reserve=[0, 0, 0, 0]  # 4个元素的数组
    #         )
            
    #         success = self._publish_command(hand_cmd)
    #         if success:
    #             print(f"[Dex3] {self.hand}手电机已停止")
    #         return success
            
    #     except Exception as e:
    #         print(f"[Dex3] 停止电机失败: {e}")
    #         import traceback
    #         traceback.print_exc()
    #         return False


class Dex3Gestures:
    """预定义的手势库 - 使用弧度值，不使用归一化"""
    
    @staticmethod
    def get_gesture(gesture_name: str, hand_type: str = "right") -> Optional[List[float]]:
        """
        获取预定义手势的关节角度 - 直接使用弧度值
        
        Args:
            gesture_name: 手势名称
            hand_type: 手的类型 ("left" 或 "right")
        
        Returns:
            7个关节角度列表（弧度）或None
        """
        # 基础手势定义 - 直接使用弧度值，基于URDF限位
        base_gestures = {
            "open": [0.0, -0.3, 0.0, -0.1, -0.1, -0.1, -0.1],      # 张开
            "closed": [0.0, 0.8, 1.2, 1.2, 1.2, 1.2, 1.2],        # 握拳
            "pinch": [0.5, 0.6, 0.5, -0.1, -0.1, 0.8, 0.6],       # 捏取
            "point": [0.0, 0.8, 1.0, 1.0, 1.0, -0.1, -0.1],       # 指点
            "peace": [0.0, 0.8, 1.0, -0.1, -0.1, -0.1, -0.1],     # 胜利手势
            "ok": [0.5, 0.6, 0.5, 1.0, 1.0, 0.8, 0.6],            # OK手势
            "rest": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # 休息位
        }
        
        if gesture_name not in base_gestures:
            print(f"[Dex3Gestures] 未知手势: {gesture_name}")
            print(f"[Dex3Gestures] 可用手势: {list(base_gestures.keys())}")
            return None
        
        angles = base_gestures[gesture_name].copy()
        
        # 左手需要镜像某些角度
        if hand_type == "left":
            # 拇指旋转镜像，其他指关节调整符号以适应左手坐标系
            angles[0] = -angles[0]  # 拇指旋转镜像
            # 其他关节根据左手URDF限位调整
            for i in [1, 3, 4, 5, 6]:
                angles[i] = -abs(angles[i])  # 左手弯曲方向为负
        
        return angles


@contextlib.contextmanager
def dex3_connection(hand="right", interface="eth0"):
    """Dex3 连接上下文管理器 - 借鉴arm_client的设计"""
    dex3 = None
    try:
        dex3 = Dex3Client(hand=hand, interface=interface)
        yield dex3
    finally:
        if dex3:
            dex3.stop_motors()  # 安全停止
            print("Dex3 连接已安全关闭")


def test_dex3_basic_control():
    """基础控制测试 - 借鉴arm_client的测试结构"""
    print("=== Dex3 基础控制测试 ===")
    print("警告: 确保灵巧手处于安全测试环境")
    print("注意: 请根据实际网络配置修改接口参数")
    
    try:
        # 创建右手控制器
        dex3 = Dex3Client(hand="right", interface="eth0")
        
        # 等待连接稳定
        print("等待连接稳定...")
        time.sleep(3.0)
        
        # 执行基础序列 - 使用借鉴的完整流程
        dex3.execute_basic_sequence()
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"测试失败: {e}")
        print("请检查:")
        print("1. 网络接口配置（默认eth0）")
        print("2. 灵巧手连接状态")
        print("3. DDS通信设置")
        import traceback
        traceback.print_exc()


def main():
    """
    主函数 - 借鉴arm_client的主函数结构
    """
    print(" --- Unitree Robotics --- ")
    print("     Dex3 Hand Control Example      \n")
    
    # 获取手部选择
    hand_input = input("Please input the hand id (L for left hand, R for right hand): ").strip().upper()
    
    if hand_input == "L":
        hand_side = "left"
    elif hand_input == "R":
        hand_side = "right"
    else:
        print("Invalid hand id. Please input 'L' or 'R'.")
        return -1
    
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        print("Example: python3 dex3_client.py eth0")
        print("\n注意: 请根据实际网络配置修改接口参数")
        return -1
    
    network_interface = sys.argv[1]
    
    try:
        # 创建Dex3客户端
        with dex3_connection(hand=hand_side, interface=network_interface) as dex3:
            print(f"\n成功初始化 {hand_side} 手控制器")
            print(f"网络接口: {network_interface}")
            print("Commands:")
            print("  1 - 执行基础序列（arm_client风格）")
            print("  2 - 设置预定义手势")
            print("  3 - 查看当前状态")
            print("  4 - 打印关节详细状态")
            print("  5 - 可视化传感器数据")
            print("  q - 退出")
            
            # 简单的命令循环
            while True:
                try:
                    cmd = input("\n请输入命令: ").strip().lower()
                    
                    if cmd == 'q':
                        print("退出程序...")
                        break
                    elif cmd == '1':
                        print("执行基础序列（arm_client风格）...")
                        dex3.execute_basic_sequence()
                    elif cmd == '2':
                        print("可用手势: open, closed, pinch, point, peace, ok, rest")
                        gesture = input("请输入手势名称: ").strip()
                        dex3.set_gesture(gesture)
                    elif cmd == '3':
                        print("读取当前状态...")
                        positions = dex3.get_current_joint_positions()
                        if positions:
                            print(f"当前关节位置: {[f'{pos:.3f}' for pos in positions]} rad")
                        else:
                            print("无法读取关节位置")
                    elif cmd == '4':
                        print("关节详细状态:")
                        dex3.print_joint_states()
                    elif cmd == '5':
                        print("开始可视化传感器数据（10秒）...")
                        success = dex3.visualize_sensor_data(interval=0.2, duration=10.0)
                        if not success:
                            print("传感器数据可视化失败")
                    else:
                        print("未知命令")
                        
                except KeyboardInterrupt:
                    print("\n收到中断信号，停止控制...")
                    dex3.stop_control()
                    break
        
        print("程序正常退出")
        return 0
        
    except Exception as e:
        print(f"程序执行失败: {e}")
        print("请检查:")
        print("1. 网络接口配置")
        print("2. 灵巧手连接状态") 
        print("3. unitree_sdk2py库安装")
        import traceback
        traceback.print_exc()
        return -1


if __name__ == "__main__":
    if len(sys.argv) == 1:
        # 如果没有参数，运行测试函数
        test_dex3_basic_control()
    else:
        # 如果有参数，运行主函数
        sys.exit(main())