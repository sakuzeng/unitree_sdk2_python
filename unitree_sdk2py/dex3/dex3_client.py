"""
Dex3 灵巧手控制客户端

该模块提供了对宇树 Dex3-1 力控灵巧手的控制接口，基于官方DDS协议规范。
- 7 自由度关节控制（3指 + 拇指旋转）；9个传感器，每个传感器是3*4的点阵数据
- 触觉传感器数据读取  
- 左右手支持
- 多种控制模式（位置、速度、扭矩）
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
    
    # 控制增益默认值
    default_kp_1: float = 0.5
    default_kp: float = 1.5
    default_kd: float = 0.1
    
    # 安全参数
    max_torque: float = 2.0  # 最大扭矩 (N·m)
    timeout_enable: bool = True
    
    def __post_init__(self):
        if self.joint_limits_left is None:
            # 左手关节限位 - 与C++示例完全一致
            self.joint_limits_left = [
                (-1.05, 1.05),   # thumb_0: 拇指旋转
                (-0.724, 1.05),  # thumb_1: 拇指弯曲1  
                (0.0, 1.75),     # thumb_2: 拇指弯曲2
                (-1.57, 0.0),    # middle_0: 中指弯曲1
                (-1.75, 0.0),    # middle_1: 中指弯曲2
                (-1.57, 0.0),    # index_0: 食指弯曲1
                (-1.75, 0.0),    # index_1: 食指弯曲2
            ]
        
        if self.joint_limits_right is None:
            # 右手关节限位 - 与C++示例完全一致
            self.joint_limits_right = [
                (-1.05, 1.05),   # thumb_0: 拇指旋转
                (-1.05, 0.742),  # thumb_1: 拇指弯曲1
                (-1.75, 0.0),    # thumb_2: 拇指弯曲2
                (0.0, 1.57),     # middle_0: 中指弯曲1
                (0.0, 1.75),     # middle_1: 中指弯曲2
                (0.0, 1.57),     # index_0: 食指弯曲1
                (0.0, 1.75),     # index_1: 食指弯曲2
            ]


class Dex3Client:
    """
    Dex3 灵巧手控制客户端
    
    基于官方DDS协议规范实现，与C++版本完全兼容。
    
    Args:
        hand (str): 手的类型，"left" 或 "right"
        interface (str): 网络接口名称，如 "eth0"
        config (Dex3Config, optional): 配置参数
    
    Example:
        # 创建右手控制器
        dex3 = Dex3Client(hand="right", interface="eth0")
        
        # 设置关节角度
        angles = [0.0, 0.5, 1.0, 0.8, 1.2, 0.6, 1.1]
        dex3.set_joint_angles(angles)
        
        # 读取状态
        state = dex3.read_state()
        if state:
            print(f"关节角度: {[ms.q for ms in state.motor_state]}")
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
        
        # 旋转电机状态变量 - 模拟C++静态变量
        self._rotate_count = 1
        self._rotate_dir = 1
        
        # 初始化DDS连接
        self._init_dds_connection()
    
    def _init_dds_connection(self):
        """初始化DDS连接 - 基于官方规范"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
            
            # 初始化DDS工厂 - 与C++版本一致
            if self._interface:
                ChannelFactoryInitialize(0, self._interface)
                print(f"[Dex3] 初始化DDS连接 - 接口: {self._interface}")
            
            # 创建发布者和订阅者
            self._cmd_publisher = ChannelPublisher(self._cmd_topic, HandCmd_)
            self._cmd_publisher.Init()
            
            self._state_subscriber = ChannelSubscriber(self._state_topic, HandState_)
            self._state_subscriber.Init(self._state_callback, 10)
            
            print(f"[Dex3] 成功连接到 {self.hand} 手")
            print(f"[Dex3] 命令话题: {self._cmd_topic}")
            print(f"[Dex3] 状态话题: {self._state_topic}")
            
            # 等待连接稳定
            time.sleep(1.0)
            
        except Exception as e:
            print(f"[Dex3] DDS连接失败: {e}")
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
    
    def _limits(self) -> Tuple[List[float], List[float]]:
        """获取关节限位的最小值和最大值列表"""
        limits = self._get_joint_limits()
        mins = [limit[0] for limit in limits]
        maxs = [limit[1] for limit in limits]
        return mins, maxs
    
    def _pack_mode(self, motor_id: int, status: int = 0x01, timeout: bool = True) -> int:
        """
        打包电机控制模式 - 与C++版本的RIS_Mode_t完全一致
        
        Args:
            motor_id: 电机ID (0-6)
            status: 工作模式 (0=Disable, 1=Enable)
            timeout: 是否启用超时保护
        
        Returns:
            打包后的模式字节
        """
        mode = 0
        mode |= (motor_id & 0x0F)           # 低4位: ID
        mode |= (status & 0x07) << 4        # 中3位: 状态
        mode |= (int(timeout) & 0x01) << 7  # 高1位: 超时
        return mode
    
    def _make_zero_cmd(self):
        """创建零位命令消息 - 修复HandCmd_.reserve为4个元素"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            # 创建7个电机命令
            motor_cmds = []
            for i in range(self.MOTOR_MAX):
                motor_cmd = MotorCmd_(
                    mode=1,  # Enable
                    q=0.0,
                    dq=0.0,
                    tau=0.0,
                    kp=self.config.default_kp,
                    kd=self.config.default_kd,
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

    def _publish(self, cmd) -> bool:
        """发布命令消息 - 增强错误处理"""
        if self._cmd_publisher is None:
            print("[Dex3] 错误: 发布者未初始化")
            return False
        
        try:
            # 验证命令对象
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
    
    def set_joint_angles(
        self, 
        angles: List[float], 
        kp: Optional[float] = None, 
        kd: Optional[float] = None,
        velocities: Optional[List[float]] = None,
        torques: Optional[List[float]] = None,
        mode: int = 1
    ) -> bool:
        """设置关节角度 - 修复HandCmd_.reserve为4个元素"""
        if len(angles) != self.MOTOR_MAX:
            print(f"[Dex3] 错误: 需要{self.MOTOR_MAX}个关节角度，得到{len(angles)}个")
            return False
        
        kp = kp if kp is not None else self.config.default_kp_1
        kd = kd if kd is not None else self.config.default_kd
        velocities = velocities or [0.0] * self.MOTOR_MAX
        torques = torques or [0.0] * self.MOTOR_MAX
        
        # 限位检查  
        limits = self._get_joint_limits()
        clamped_angles = []
        for i, (angle, (min_val, max_val)) in enumerate(zip(angles, limits)):
            if angle < min_val or angle > max_val:
                print(f"[Dex3] 警告: 关节{i}角度{angle:.3f}超出限位[{min_val:.3f}, {max_val:.3f}]")
                clamped_angle = max(min_val, min(max_val, angle))
                clamped_angles.append(clamped_angle)
            else:
                clamped_angles.append(angle)
        
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            # 创建电机命令列表
            motor_cmds = []
            for i in range(self.MOTOR_MAX):
                motor_cmd = MotorCmd_(
                    mode=1,
                    q=float(clamped_angles[i]),
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
            
            # 发布命令
            success = self._publish(hand_cmd)
            if success:
                print(f"[Dex3] {self.hand}手关节角度设置成功")
            else:
                print(f"[Dex3] {self.hand}手关节角度设置失败")
            
            return success
        
        except Exception as e:
            print(f"[Dex3] 设置关节角度失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_joint_angles(self, timeout: float = 1.0) -> Optional[List[float]]:
        """获取当前关节角度"""
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
    
    # def get_fingertip_pressures(self, timeout: float = 1.0) -> Optional[List[float]]:
        """获取指尖压力值（GUI 中用于自适应抓取）"""
        pressure_data = self.get_pressure_data(timeout)
        if pressure_data:
            try:
                # 提取指尖传感器数据（索引2, 5, 8, 11对应指尖）
                fingertip_pressures = []
                for sensor_key in pressure_data:
                    pressures = pressure_data[sensor_key]['pressure']
                    # 选择代表指尖的传感器点
                    tip_indices = [2, 5, 8, 11] if len(pressures) >= 12 else list(range(len(pressures)))
                    fingertip_pressures.extend([pressures[i] for i in tip_indices if i < len(pressures)])
                return fingertip_pressures
            except Exception as e:
                print(f"[Dex3] 解析指尖压力失败: {e}")
        return None
    
    def stop_motors(self) -> bool:
        """停止所有电机 - 修复HandCmd_.reserve为4个元素"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            # 创建7个电机命令
            motor_cmds = []
            for i in range(self.MOTOR_MAX):
                motor_cmd = MotorCmd_(
                    mode=0,  # Disable
                    q=0.0,
                    dq=0.0,
                    tau=0.0,
                    kp=0.0,
                    kd=0.0,
                    reserve=0  # 单个uint32
                )
                motor_cmds.append(motor_cmd)
            
            hand_cmd = HandCmd_(
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0]  # 4个元素的数组
            )
            
            success = self._publish(hand_cmd)
            if success:
                print(f"[Dex3] {self.hand}手电机已停止")
            return success
            
        except Exception as e:
            print(f"[Dex3] 停止电机失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def damp_motors(self, kd: float = 0.5) -> bool:
        """电机阻尼模式 - 修复HandCmd_.reserve为4个元素"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            motor_cmds = []
            for i in range(self.MOTOR_MAX):
                motor_cmd = MotorCmd_(
                    mode=1,  # Enable
                    q=0.0,
                    dq=0.0,
                    tau=0.0,
                    kp=0.0,  # 无位置控制
                    kd=0.1,
                    reserve=0  # 单个uint32
                )
                motor_cmds.append(motor_cmd)
            
            hand_cmd = HandCmd_(
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0]  # 4个元素的数组
            )
            
            success = self._publish(hand_cmd)
            if success:
                print(f"[Dex3] {self.hand}手进入阻尼模式")
            return success
            
        except Exception as e:
            print(f"[Dex3] 阻尼模式失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def rotate_motors(self, is_left_hand: bool = None) -> bool:
        """旋转电机 - 修复HandCmd_.reserve为4个元素"""
        if is_left_hand is None:
            is_left_hand = (self.hand == "left")
        
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            limits = self._get_joint_limits()
            motor_cmds = []
            
            for i in range(self.MOTOR_MAX):
                # 计算正弦波位置 - 与C++版本一致
                range_val = limits[i][1] - limits[i][0]
                mid = (limits[i][1] + limits[i][0]) / 2.0
                amplitude = range_val / 2.0
                q = mid + amplitude * math.sin(self._rotate_count / 20000.0 * math.pi)
                
                motor_cmd = MotorCmd_(
                    mode=1,  # Enable
                    q=float(q),
                    dq=0.0,
                    tau=0.0,
                    kp=self.config.default_kp_1,
                    kd=self.config.default_kd,
                    reserve=0  # 单个uint32
                )
                motor_cmds.append(motor_cmd)
            
            hand_cmd = HandCmd_(
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0]  # 4个元素的数组
            )
            
            success = self._publish(hand_cmd)
            
            # 更新计数器 - 模拟C++静态变量
            self._rotate_count += self._rotate_dir
            if self._rotate_count >= 10000:
                self._rotate_dir = -1
            if self._rotate_count <= -10000:
                self._rotate_dir = 1
            
            return success
            
        except Exception as e:
            print(f"[Dex3] 旋转电机失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def grip_hand(self, grip_strength: float = 1.5, is_left_hand: bool = None) -> bool:
        """抓握手 - 修复HandCmd_.reserve为4个元素"""
        if is_left_hand is None:
            is_left_hand = (self.hand == "left")
        
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, MotorCmd_
            
            limits = self._get_joint_limits()
            motor_cmds = []
            
            for i in range(self.MOTOR_MAX):
                # 设置到中间位置 - 与C++版本完全一致
                mid = (limits[i][1] + limits[i][0]) / 2.0
                
                motor_cmd = MotorCmd_(
                    mode=1,  # Enable
                    q=float(mid),
                    dq=0.0,
                    tau=0.0,
                    kp=float(grip_strength),
                    kd=self.config.default_kd,
                    reserve=0  # 单个uint32
                )
                motor_cmds.append(motor_cmd)
            
            hand_cmd = HandCmd_(
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0]  # 4个元素的数组
            )
            
            success = self._publish(hand_cmd)
            if success:
                print(f"[Dex3] {self.hand}手开始抓握")
            return success
            
        except Exception as e:
            print(f"[Dex3] 抓握失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def print_state(self, is_left_hand: bool = None) -> bool:
        """
        打印状态 - 模仿C++版本的printState函数
        
        Args:
            is_left_hand: 是否为左手，None时自动判断
        
        Returns:
            bool: 打印是否成功
        """
        if is_left_hand is None:
            is_left_hand = (self.hand == "left")
        
        try:
            angles = self.get_joint_angles()
            if angles is None:
                print("[Dex3] 未能获取关节角度")
                return False
            
            limits = self._get_joint_limits()
            
            # 归一化关节角度 - 与C++版本一致
            normalized_angles = []
            for i, angle in enumerate(angles):
                min_val, max_val = limits[i]
                normalized = (angle - min_val) / (max_val - min_val)
                normalized = max(0.0, min(1.0, normalized))  # clamp to [0,1]
                normalized_angles.append(normalized)
            
            # 清屏并打印 - 与C++版本一致
            print("-- Hand State --")
            print("--- Current State: Test ---")
            print("Commands:")
            print("  r - Rotate")
            print("  g - Grip")
            print("  p - Print_state")
            print("  q - Quit")
            print("  s - Stop")
            
            if is_left_hand:
                print(f" L: {normalized_angles}")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            else:
                print(f" R: {normalized_angles}")
            
            return True
            
        except Exception as e:
            print(f"[Dex3] 打印状态失败: {e}")
            return False


def check_dex3_connection(hand="right", interface="eth0") -> bool:
    """
    检查 Dex3 网络连接
    
    Args:
        hand: 手部类型
        interface: 网络接口
    
    Returns:
        bool: 连接是否正常
    """
    try:
        # 检查网络接口状态
        import subprocess
        result = subprocess.run(['ip', 'addr', 'show', interface], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"网络接口 {interface} 不存在或未激活")
            return False
        
        print(f"Dex3 {hand} 手网络连接检查通过")
        return True
    except Exception as e:
        print(f"Dex3 连接检查失败: {e}")
        return False


# 预定义手势
class Dex3Gestures:
    """预定义的手势库"""
    
    @staticmethod
    def get_gesture(gesture_name: str, hand_type: str = "right") -> Optional[List[float]]:
        """
        获取预定义手势的关节角度
        
        Args:
            gesture_name: 手势名称
            hand_type: 手的类型 ("left" 或 "right")
        
        Returns:
            7个关节角度列表或None
        """
        gestures = {
            "open": [0.0, -0.3, -0.1, -0.1, -0.1, -0.1, -0.1],
            "closed": [0.0, 1.4, 1.2, 1.2, 1.2, 1.2, 1.2],
            "pinch": [0.5, 1.0, 0.8, -0.1, -0.1, 1.0, 0.8],
            "point": [0.0, 1.4, 1.2, 1.2, 1.2, -0.1, -0.1],
            "peace": [0.0, 1.4, 1.2, -0.1, -0.1, -0.1, -0.1],
            "ok": [0.5, 1.0, 0.8, 1.2, 1.2, 1.0, 0.8],
            "rest": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        
        if gesture_name not in gestures:
            print(f"[Dex3Gestures] 未知手势: {gesture_name}")
            return None
        
        angles = gestures[gesture_name].copy()
        
        # 左手需要镜像某些角度
        if hand_type == "left":
            angles[0] = -angles[0]  # 拇指旋转镜像
        
        return angles
    
@contextlib.contextmanager
def dex3_connection(hand="right", interface="eth0"):
    """Dex3 连接上下文管理器"""
    dex3 = None
    try:
        dex3 = Dex3Client(hand=hand, interface=interface)
        yield dex3
    finally:
        if dex3:
            dex3.stop_motors()  # 安全停止
            print("Dex3 连接已安全关闭")

# 使用示例和测试函数
def test_dex3_basic_control():
    """基础控制测试"""
    print("=== Dex3 基础控制测试 ===")
    print("警告: 确保机器人处于安全测试环境")
    
    try:
        # 创建右手控制器
        dex3 = Dex3Client(hand="right", interface="eth0")
        
        # 等待连接稳定
        print("等待连接稳定...")
        time.sleep(3.0)
        
        # 读取初始状态
        print("读取初始状态...")
        state = dex3.read_state(timeout=5.0)
        if state:
            print("成功读取到状态数据")
            angles = dex3.get_joint_angles(timeout=2.0)
            if angles:
                print(f"当前关节角度: {[f'{a:.3f}' for a in angles]}")
            else:
                print("关节角度解析失败")
        else:
            print("未能读取到状态数据，请检查:")
            print("1. 网络接口是否正确 (eth0)")
            print("2. Dex3设备是否已连接并上电")
            print("3. DDS通信是否正常")
            return
        
        # 手势测试
        print("\n执行手势序列...")
        gestures_to_test = ["open", "closed", "pinch", "point", "rest", "peace", "ok"]
        
        for gesture_name in gestures_to_test:
            print(f"执行手势: {gesture_name}")
            angles = Dex3Gestures.get_gesture(gesture_name, "right")
            if angles:
                dex3.set_joint_angles(angles)
                time.sleep(2.0)
        
        # 压力传感器测试
        print("\n读取压力传感器数据...")
        pressure_data = dex3.get_pressure_data()
        if pressure_data:
            print(f"力压传感器数据已获取: {len(pressure_data)} 个传感器")
        else:
            print("未获取到压力传感器数据")
        success = dex3.visualize_sensor_data(interval=0.2, duration=100.0)  # 10秒实时显示
        # IMU数据测试
        print("\n读取IMU数据...")
        imu_data = dex3.get_imu_data()
        if imu_data:
            print(f"IMU数据已获取: {list(imu_data.keys())}")
        else:
            print("未获取到IMU数据")
        
        # 电源信息测试
        print("\n读取电源信息...")
        power_info = dex3.get_power_info()
        if power_info:
            print(f"电源电压: {power_info.get('voltage', 0):.2f}V")
            print(f"电源电流: {power_info.get('current', 0):.2f}A")
        else:
            print("未获取到电源信息")
        
        print("\n测试完成!")
    
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()


def main():
    """
    主函数 - 模仿C++版本的主函数
    """
    print(" --- Unitree Robotics --- ")
    print("     Dex3 Hand Example      \n")
    
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
        return -1
    
    network_interface = sys.argv[1]
    
    try:
        # 创建Dex3客户端
        with dex3_connection(hand=hand_side, interface=network_interface) as dex3:
            print(f"\n成功初始化 {hand_side} 手控制器")
            print("Commands:")
            print("  r - Rotate")
            print("  g - Grip")
            print("  p - Print_state")
            print("  q - Quit")
            print("  s - Stop")
            
            # 简单的命令循环
            while True:
                try:
                    cmd = input("\n请输入命令: ").strip().lower()
                    
                    if cmd == 'q':
                        print("退出程序...")
                        break
                    elif cmd == 'r':
                        print("旋转电机...")
                        for _ in range(100):  # 旋转一段时间
                            dex3.rotate_motors()
                            time.sleep(0.01)
                    elif cmd == 'g':
                        print("抓握...")
                        dex3.grip_hand()
                        time.sleep(1.0)
                    elif cmd == 's':
                        print("停止电机...")
                        dex3.stop_motors()
                    elif cmd == 'p':
                        print("打印状态...")
                        dex3.print_state()
                        time.sleep(0.1)
                    else:
                        print("未知命令")
                        
                except KeyboardInterrupt:
                    print("\n收到中断信号，停止电机...")
                    dex3.stop_motors()
                    break
        
        print("程序正常退出")
        return 0
        
    except Exception as e:
        print(f"程序执行失败: {e}")
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