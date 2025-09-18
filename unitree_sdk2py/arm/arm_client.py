"""
G1 机器人手臂控制客户端

该模块提供了对宇树 G1 机器人手臂的控制接口，基于官方DDS协议规范。
- 17自由度手臂控制（双臂14DOF + 腰部3DOF，可选择固定腰部）
- 高层运控DDS接口
- 安全权重过渡控制
- 多种控制模式（位置、速度、扭矩）
- 预定义手势库
"""

import time
import threading
import contextlib
import math
import sys
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize


@dataclass
class G1ArmConfig:
    """G1 手臂配置参数"""
    # 控制参数 - 与C++例程一致
    default_kp: float = 60.0
    default_kd: float = 1.5
    default_dq: float = 0.0
    default_tau_ff: float = 0.0
    
    # 时间控制参数 - 与C++例程一致
    control_dt: float = 0.02  # 控制周期 20ms
    max_joint_velocity: float = 0.5  # 最大关节速度 rad/s
    weight_rate: float = 0.2  # 权重变化率
    
    # 安全参数
    enable_waist_control: bool = False  # 是否启用腰部控制，默认固定
    timeout_enable: bool = True
    
    # 关节限位 (单位: rad) - 基于G1 URDF
    joint_limits: List[Tuple[float, float]] = None
    
    def __post_init__(self):
        if self.joint_limits is None:
            # G1手臂关节限位 - 基于URDF文件，按照手臂关节顺序
            self.joint_limits = [
                # Left arm (7 joints) - 基于URDF中的joint limit
                (-3.0892, 2.6704),   # left_shoulder_pitch_joint
                (-1.5882, 2.2515),   # left_shoulder_roll_joint  
                (-2.618, 2.618),     # left_shoulder_yaw_joint
                (-1.0472, 2.0944),   # left_elbow_joint
                (-1.972222054, 1.972222054),  # left_wrist_roll_joint
                (-1.614429558, 1.614429558),  # left_wrist_pitch_joint
                (-1.614429558, 1.614429558),  # left_wrist_yaw_joint
                
                # Right arm (7 joints) - 基于URDF中的joint limit
                (-3.0892, 2.6704),   # right_shoulder_pitch_joint
                (-2.2515, 1.5882),   # right_shoulder_roll_joint (注意左右对称)
                (-2.618, 2.618),     # right_shoulder_yaw_joint
                (-1.0472, 2.0944),   # right_elbow_joint
                (-1.972222054, 1.972222054),  # right_wrist_roll_joint
                (-1.614429558, 1.614429558),  # right_wrist_pitch_joint
                (-1.614429558, 1.614429558),  # right_wrist_yaw_joint
                
                # Waist (3 joints) - 基于URDF中的joint limit
                (-2.618, 2.618),     # waist_yaw_joint
                (-0.52, 0.52),       # waist_roll_joint
                (-0.52, 0.52),       # waist_pitch_joint
            ]


class JointIndex:
    """G1机器人关节索引枚举 - 与C++版本完全一致"""
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnkle = 4
    kLeftAnkleRoll = 5
    
    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnkle = 10
    kRightAnkleRoll = 11
    
    # Waist
    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14
    
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristYaw = 21
    
    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    
    # Reserved joints
    kNotUsedJoint = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


class G1ArmClient:
    """
    G1 机器人手臂控制客户端
    
    基于官方高层运控DDS接口实现，与C++版本完全兼容。
    支持固定腰部的安全手臂控制。
    
    Args:
        interface (str): 网络接口名称，默认 "eth0"，用户需根据实际情况修改
        config (G1ArmConfig, optional): 配置参数
    
    Example:
        # 创建手臂控制器（默认固定腰部）
        arm = G1ArmClient(interface="eth0")
        
        # 执行基础动作序列
        arm.execute_basic_sequence()
        
        # 设置手臂到特定姿态
        arm.set_arm_pose("open_arms")
    
    Warning:
        使用前请确保：
        1. 机器人处于安全测试环境并悬挂
        2. 网络接口配置正确（默认eth0，用户需根据实际修改）
        3. 关闭sport_mode服务（如适用）
    """
    
    def __init__(
        self,
        interface: str = "eth0",
        config: Optional[G1ArmConfig] = None
    ):
        self.config = config or G1ArmConfig()
        self._interface = interface
        
        # DDS 通信设置 - 基于官方规范
        self._cmd_topic = "rt/arm_sdk"
        self._state_topic = "rt/lowstate"
        
        # 发布者和订阅者
        self._cmd_publisher: Optional[ChannelPublisher] = None
        self._state_subscriber: Optional[ChannelSubscriber] = None
        
        # 状态缓存
        self._latest_state: Optional[Any] = None
        self._state_lock = threading.Lock()
        
        # 手臂关节索引 - 与C++版本完全一致
        self._arm_joints = [
            # Left arm
            JointIndex.kLeftShoulderPitch, JointIndex.kLeftShoulderRoll,
            JointIndex.kLeftShoulderYaw, JointIndex.kLeftElbow,
            JointIndex.kLeftWristRoll, JointIndex.kLeftWristPitch,
            JointIndex.kLeftWristYaw,
            # Right arm
            JointIndex.kRightShoulderPitch, JointIndex.kRightShoulderRoll,
            JointIndex.kRightShoulderYaw, JointIndex.kRightElbow,
            JointIndex.kRightWristRoll, JointIndex.kRightWristPitch,
            JointIndex.kRightWristYaw,
        ]
        
        # 如果启用腰部控制，添加腰部关节
        if self.config.enable_waist_control:
            self._arm_joints.extend([
                JointIndex.kWaistYaw,
                JointIndex.kWaistRoll,
                JointIndex.kWaistPitch
            ])
        
        self.ARM_JOINT_COUNT = len(self._arm_joints)
        
        # 控制参数 - 与C++例程一致
        self._weight = 0.0
        self._delta_weight = self.config.weight_rate * self.config.control_dt
        self._max_joint_delta = self.config.max_joint_velocity * self.config.control_dt
        self._sleep_duration = self.config.control_dt
        
        # 预定义位置 - 与C++例程一致
        self._init_pos = [0.0] * self.ARM_JOINT_COUNT
        # 获得双臂水平张开的电机数据
        self._target_pos = self._get_target_positions()
        
        # 当前期望位置状态 - 重要：用于跟踪运动状态
        self._current_jpos_des = [0.0] * self.ARM_JOINT_COUNT
        
        # 初始化DDS连接
        self._init_dds_connection()
    
    def _get_target_positions(self) -> List[float]:
        """获取目标位置 - 双臂水平张开的电机数据，基于URDF关节限位优化"""
        kPi_2 = math.pi / 2
        
        if self.config.enable_waist_control:
            # 17关节：14手臂 + 3腰部
            return [
                # Left arm - 优化后的安全姿态
                0.0, kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
                # Right arm - 考虑右臂roll关节限位的镜像  
                0.0, -kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
                # Waist (保持零位以确保安全)
                0.0, 0.0, 0.0
            ]
        else:
            # 14关节：仅手臂
            return [
                # Left arm
                0.0, kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
                # Right arm
                0.0, -kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
            ]
    
    def _init_dds_connection(self):
        """初始化DDS连接 - 基于官方规范"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            
            # 初始化DDS工厂 - 与C++版本一致
            if self._interface:
                ChannelFactoryInitialize(0, self._interface)
                print(f"[G1Arm] 初始化DDS连接 - 接口: {self._interface}")
                print(f"[G1Arm] 提示: 请确保网络接口配置正确")
            
            # 创建发布者和订阅者
            self._cmd_publisher = ChannelPublisher(self._cmd_topic, LowCmd_)
            self._cmd_publisher.Init()
            
            self._state_subscriber = ChannelSubscriber(self._state_topic, LowState_)
            self._state_subscriber.Init(self._state_callback, 10)
            
            print(f"[G1Arm] 成功连接到G1机器人")
            print(f"[G1Arm] 命令话题: {self._cmd_topic}")
            print(f"[G1Arm] 状态话题: {self._state_topic}")
            print(f"[G1Arm] 手臂关节数: {self.ARM_JOINT_COUNT}")
            print(f"[G1Arm] 腰部控制: {'启用' if self.config.enable_waist_control else '禁用（推荐）'}")
            
            # 等待连接稳定
            time.sleep(1.0)
            
        except Exception as e:
            print(f"[G1Arm] DDS连接失败: {e}")
            print(f"[G1Arm] 请检查网络接口配置和依赖库安装")
            raise
    
    def _state_callback(self, msg):
        """状态消息回调 - 基于官方LowState_结构"""
        with self._state_lock:
            self._latest_state = msg
    
    def read_state(self, timeout: float = 1.0) -> Optional[Any]:
        """
        读取机器人状态 - 基于官方LowState_结构
        
        Args:
            timeout: 超时时间(秒)
        
        Returns:
            LowState_ 消息或 None
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
    
    def get_current_joint_positions(self, timeout: float = 2.0) -> Optional[List[float]]:
        """获取当前手臂关节位置"""
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= 35:
            try:
                positions = []
                for joint_idx in self._arm_joints:
                    positions.append(float(state.motor_state[joint_idx].q))
                return positions
            except Exception as e:
                print(f"[G1Arm] 解析关节位置失败: {e}")
        return None
    
    def _create_arm_command(
        self,
        positions: List[float],
        velocities: Optional[List[float]] = None,
        torques: Optional[List[float]] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        weight: Optional[float] = None
    ):
        """创建手臂控制命令"""
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, MotorCmd_
            
            if len(positions) != self.ARM_JOINT_COUNT:
                raise ValueError(f"位置数量({len(positions)})与关节数({self.ARM_JOINT_COUNT})不匹配")
            
            # 设置默认值
            velocities = velocities or [self.config.default_dq] * self.ARM_JOINT_COUNT
            torques = torques or [self.config.default_tau_ff] * self.ARM_JOINT_COUNT
            kp = kp if kp is not None else self.config.default_kp
            kd = kd if kd is not None else self.config.default_kd
            weight = weight if weight is not None else self._weight
            
            # 创建35个电机命令数组
            motor_cmds = []
            for i in range(35):
                motor_cmd = MotorCmd_(
                    mode=0,  # 默认禁用
                    q=0.0,
                    dq=0.0,
                    tau=0.0,
                    kp=0.0,
                    kd=0.0,
                    reserve=[0, 0, 0]
                )
                motor_cmds.append(motor_cmd)
            
            # 设置权重 - 使用kNotUsedJoint作为权重控制，与C++例程完全一致
            motor_cmds[JointIndex.kNotUsedJoint].mode = 1
            motor_cmds[JointIndex.kNotUsedJoint].q = float(weight)
            
            # 设置手臂关节命令
            for i, joint_idx in enumerate(self._arm_joints):
                motor_cmds[joint_idx].mode = 1  # Enable
                motor_cmds[joint_idx].q = float(positions[i])
                motor_cmds[joint_idx].dq = float(velocities[i])
                motor_cmds[joint_idx].tau = float(torques[i])
                motor_cmds[joint_idx].kp = float(kp)
                motor_cmds[joint_idx].kd = float(kd)
            
            # 创建LowCmd_
            low_cmd = LowCmd_(
                mode_pr=0,  # 默认PR模式
                mode_machine=0,  # 默认模式
                motor_cmd=motor_cmds,
                reserve=[0, 0, 0, 0],
                crc=0
            )
            
            return low_cmd
            
        except Exception as e:
            print(f"[G1Arm] 创建命令失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _publish_command(self, cmd) -> bool:
        """发布命令消息"""
        if self._cmd_publisher is None:
            print("[G1Arm] 错误: 发布者未初始化")
            return False
        
        try:
            if cmd is None:
                print("[G1Arm] 错误: 命令对象为空")
                return False
            
            self._cmd_publisher.Write(cmd)
            return True
            
        except Exception as e:
            print(f"[G1Arm] 发布命令失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制值在指定范围内 - 对应C++的std::clamp"""
        return max(min_val, min(max_val, value))
    
    def smooth_transition(
        self,
        start_positions: List[float],
        target_positions: List[float],
        duration: float,
        description: str = ""
    ) -> bool:
        """
        平滑过渡到目标位置 - 完全模仿C++例程的控制逻辑

        这个版本使用与C++例程完全相同的控制算法：
        1. 使用current_jpos_des跟踪当前期望位置
        2. 每步限制最大变化量
        3. 逐步趋向目标位置
        """
        print(f"[G1Arm] {description}...")

        time_steps = int(duration / self.config.control_dt)

        # 初始化当前期望位置为起始位置
        self._current_jpos_des = start_positions.copy()

        for i in range(time_steps):
            # 更新期望位置 - 与C++例程完全一致的逻辑
            for j in range(len(self._current_jpos_des)):
                # 计算位置差
                delta = target_positions[j] - self._current_jpos_des[j]
                # 限制每步的最大变化量 - 对应C++的std::clamp
                delta = self._clamp(delta, -self._max_joint_delta, self._max_joint_delta)
                # 更新当前期望位置
                self._current_jpos_des[j] += delta

            # 创建并发送命令
            cmd = self._create_arm_command(self._current_jpos_des)
            if not self._publish_command(cmd):
                return False

            # 延时 - 对应C++的std::this_thread::sleep_for
            time.sleep(self._sleep_duration)

        print(f"[G1Arm] {description}完成")
        return True
        
    def initialize_arms(self, timeout: float = 5.0) -> bool:
        """
        初始化手臂到零位 - 完全模仿C++例程的权重过渡逻辑

        这个版本使用与C++例程完全相同的初始化算法：
        1. 逐渐增加权重从0到1
        2. 同时进行位置插值
        3. 权重使用weight*weight的平方关系
        """
        print("[G1Arm] 按回车键开始初始化手臂...")
        input()

        # 获取当前关节位置
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            print("[G1Arm] 无法读取当前关节位置")
            print("[G1Arm] 请检查机器人连接和网络配置")
            return False

        print(f"[G1Arm] 当前关节位置: {[f'{pos:.3f}' for pos in current_positions]}")

        # 权重过渡初始化 - 与C++例程完全一致
        print("[G1Arm] 初始化手臂中...")
        init_time = 5.0  # 与C++例程一致
        time_steps = int(init_time / self.config.control_dt)

        # 重置权重
        weight = 0.0

        for i in range(time_steps):
            # 增加权重 - 与C++例程完全一致
            weight += self._delta_weight
            weight = self._clamp(weight, 0.0, 1.0)
            print(f"Weight: {weight:.3f}")

            # 计算过渡位置 - 线性插值
            phase = float(i) / time_steps
            transition_positions = []
            for j in range(self.ARM_JOINT_COUNT):
                pos = self._init_pos[j] * phase + current_positions[j] * (1 - phase)
                transition_positions.append(pos)

            # 创建并发送命令 - 注意：权重使用weight*weight的平方关系（与C++例程一致）
            cmd = self._create_arm_command(transition_positions, weight=weight * weight)
            if not self._publish_command(cmd):
                return False

            # 延时
            time.sleep(self._sleep_duration)

        # 更新内部权重状态
        self._weight = 1.0

        print("[G1Arm] 手臂初始化完成")
        return True
    
    def execute_arm_control_sequence(self) -> bool:
        """
        执行手臂控制序列 - 完全模仿C++例程的控制流程
        
        流程：
        1. 等待用户按Enter
        2. 张开双臂（5秒）
        3. 放下双臂（5秒）
        """
        print("[G1Arm] 按回车键开始手臂控制...")
        input()
        
        print("[G1Arm] 开始手臂控制!")
        
        # 张开双臂 - 使用C++风格的平滑过渡
        success = self.smooth_transition(
            self._init_pos, 
            self._target_pos, 
            5.0,  # 与C++例程一致的时间
            "张开双臂"
        )
        if not success:
            return False
        
        # 放下双臂 - 使用C++风格的平滑过渡
        success = self.smooth_transition(
            self._target_pos, 
            self._init_pos, 
            5.0,  # 与C++例程一致的时间
            "放下双臂"
        )
        
        return success
    
    def stop_control(self) -> bool:
        """
        停止控制 - 完全模仿C++例程的退出逻辑
        
        使用与C++例程完全相同的权重递减算法
        """
        print("[G1Arm] 停止手臂控制中...")
        
        stop_time = 2.0  # 与C++例程一致
        time_steps = int(stop_time / self.config.control_dt)
        
        for i in range(time_steps):
            # 减小权重 - 与C++例程完全一致
            self._weight -= self._delta_weight
            self._weight = self._clamp(self._weight, 0.0, 1.0)
            
            # 创建并发送命令
            cmd = self._create_arm_command(self._init_pos, weight=self._weight)
            if not self._publish_command(cmd):
                return False
            
            # 延时
            time.sleep(self._sleep_duration)
        
        # 最终设置权重为0 - 确保电机进入自由状态
        cmd = self._create_arm_command(self._init_pos, weight=0.0)
        self._publish_command(cmd)
        
        print("[G1Arm] 手臂控制已停止")
        return True
    
    def execute_basic_sequence(self) -> bool:
        """
        执行基础控制序列 - 完全模仿C++例程的完整流程
        
        这是与C++例程完全对应的主要控制函数
        """
        print("=== G1 手臂控制演示 ===")
        print("警告: 确保机器人处于安全测试环境并悬挂")
        print("注意: 腰部已固定以确保安全")
        
        try:
            # 1. 双臂复位 - 对应C++例程的init部分
            if not self.initialize_arms():
                print("[G1Arm] 初始化失败")
                return False
            
            # 2. 执行控制序列 - 对应C++例程的张开和放下
            if not self.execute_arm_control_sequence():
                print("[G1Arm] 控制序列执行失败")
                return False
            
            # 3. 退出控制 - 对应C++例程的stop control
            if not self.stop_control():
                print("[G1Arm] 停止控制失败")
                return False
            
            print("[G1Arm] 基础序列执行完成")
            return True
            
        except Exception as e:
            print(f"[G1Arm] 执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_current_joint_positions(self, timeout: float = 2.0) -> Optional[List[float]]:
        """获取当前手臂关节位置"""
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= 35:
            try:
                positions = []
                for joint_idx in self._arm_joints:
                    positions.append(float(state.motor_state[joint_idx].q))
                return positions
            except Exception as e:
                print(f"[G1Arm] 解析关节位置失败: {e}")
        return None
    
    def set_arm_pose(self, pose_name: str) -> bool:
        """设置手臂到预定义姿态"""
        pose = G1ArmGestures.get_pose(pose_name, self.config.enable_waist_control)
        if pose is None:
            return False
        
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        # 设置权重为1.0以确保控制生效
        self._weight = 1.0
        
        return self.smooth_transition(
            current_positions,
            pose,
            3.0,
            f"设置姿态: {pose_name}"
        )
    
    def set_joint_positions(
        self,
        positions: List[float],
        duration: float = 3.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None
    ) -> bool:
        """
        设置关节位置 - 带安全限位检查
        
        Args:
            positions: 关节位置列表
            duration: 执行时间(秒)
            kp: 位置增益
            kd: 速度增益
        
        Returns:
            bool: 设置是否成功
        """
        if len(positions) != self.ARM_JOINT_COUNT:
            print(f"[G1Arm] 错误: 需要{self.ARM_JOINT_COUNT}个关节位置，得到{len(positions)}个")
            return False
        
        # 关节限位检查 - 基于URDF文件
        limits = self.config.joint_limits[:self.ARM_JOINT_COUNT]
        clamped_positions = []
        for i, (pos, (min_val, max_val)) in enumerate(zip(positions, limits)):
            if pos < min_val or pos > max_val:
                print(f"[G1Arm] 警告: 关节{i}位置{pos:.3f}超出限位[{min_val:.3f}, {max_val:.3f}]")
                clamped_pos = max(min_val, min(max_val, pos))
                clamped_positions.append(clamped_pos)
            else:
                clamped_positions.append(pos)
        
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        # 设置权重为1.0以确保控制生效
        self._weight = 1.0
        
        return self.smooth_transition(
            current_positions,
            clamped_positions,
            duration,
            "设置关节位置"
        )
    
    def get_joint_states(self, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """
        获取详细的关节状态信息
        
        Returns:
            包含位置、速度、扭矩等信息的字典
        """
        state = self.read_state(timeout)
        if state and hasattr(state, 'motor_state') and len(state.motor_state) >= 35:
            try:
                joint_states = {
                    'positions': [],
                    'velocities': [],
                    'torques': [],
                    'temperatures': [],
                    'modes': []
                }
                
                for joint_idx in self._arm_joints:
                    motor_state = state.motor_state[joint_idx]
                    joint_states['positions'].append(float(motor_state.q))
                    joint_states['velocities'].append(float(motor_state.dq))
                    joint_states['torques'].append(float(motor_state.tau_est))
                    joint_states['temperatures'].append(motor_state.temperature[0] if hasattr(motor_state, 'temperature') else 0)
                    joint_states['modes'].append(int(motor_state.mode))
                
                return joint_states
            except Exception as e:
                print(f"[G1Arm] 解析关节状态失败: {e}")
        return None
    
    def print_joint_states(self):
        """打印当前关节状态 - 调试用"""
        states = self.get_joint_states()
        if states:
            print("[G1Arm] 当前关节状态:")
            for i in range(self.ARM_JOINT_COUNT):
                print(f"  关节{i:2d}: 位置={states['positions'][i]:6.3f}, "
                      f"速度={states['velocities'][i]:6.3f}, "
                      f"扭矩={states['torques'][i]:6.3f}")
        else:
            print("[G1Arm] 无法获取关节状态")


class G1ArmGestures:
    """预定义的手臂姿态库 - 基于URDF关节限位优化"""
    
    @staticmethod
    def get_pose(pose_name: str, include_waist: bool = False) -> Optional[List[float]]:
        """
        获取预定义姿态的关节角度
        
        Args:
            pose_name: 姿态名称
            include_waist: 是否包含腰部关节
        
        Returns:
            关节角度列表或None
        """
        kPi_2 = math.pi / 2
        kPi_4 = math.pi / 4
        kPi_6 = math.pi / 6
        
        # 基础手臂姿态 (14 DOF) - 基于URDF限位优化
        arm_poses = {
            "rest": [0.0] * 14,  # 零位
            "open_arms": [
                # Left arm - 水平张开
                0.0, kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
                # Right arm - 水平张开（考虑roll关节限位）
                0.0, -kPi_2, 0.0, kPi_2, 0.0, 0.0, 0.0,
            ],
            "forward_reach": [
                # Left arm - 向前伸展（安全角度）
                kPi_2, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0,
                # Right arm - 向前伸展
                kPi_2, -0.3, 0.0, 0.5, 0.0, 0.0, 0.0,
            ],
            "up_reach": [
                # Left arm - 向上伸展（安全角度）
                -kPi_6, 0.5, 0.0, 0.3, 0.0, 0.0, 0.0,
                # Right arm - 向上伸展
                -kPi_6, -0.5, 0.0, 0.3, 0.0, 0.0, 0.0,
            ],
            "defensive": [
                # Left arm - 防守姿态（安全角度）
                -kPi_4, kPi_4, 0.0, kPi_2, 0.0, 0.0, 0.0,
                # Right arm - 防守姿态
                -kPi_4, -kPi_4, 0.0, kPi_2, 0.0, 0.0, 0.0,
            ],
            "greeting": [
                # Left arm - 打招呼姿态
                0.0, 0.8, 0.0, 1.2, 0.0, 0.0, 0.0,
                # Right arm - 保持自然
                0.0, -0.3, 0.0, 0.3, 0.0, 0.0, 0.0,
            ],
        }
        
        if pose_name not in arm_poses:
            print(f"[G1ArmGestures] 未知姿态: {pose_name}")
            print(f"[G1ArmGestures] 可用姿态: {list(arm_poses.keys())}")
            return None
        
        pose = arm_poses[pose_name].copy()
        
        # 如果包含腰部，添加腰部零位（推荐保持固定）
        if include_waist:
            pose.extend([0.0, 0.0, 0.0])  # 腰部保持零位
        
        return pose


@contextlib.contextmanager
def g1_arm_connection(interface="eth0", enable_waist=False):
    """G1 手臂连接上下文管理器"""
    arm = None
    try:
        config = G1ArmConfig(enable_waist_control=enable_waist)
        arm = G1ArmClient(interface=interface, config=config)
        yield arm
    finally:
        if arm:
            arm.stop_control()  # 安全停止
            print("G1 手臂连接已安全关闭")


def test_g1_arm_basic_control():
    """基础控制测试"""
    print("=== G1 手臂基础控制测试 ===")
    print("警告: 确保机器人处于安全测试环境")
    print("注意: 请根据实际网络配置修改接口参数")
    
    try:
        # 创建手臂控制器（不包含腰部，更安全）
        arm = G1ArmClient(interface="eth0")
        
        # 等待连接稳定
        print("等待连接稳定...")
        time.sleep(3.0)
        
        # 执行基础序列 - 使用C++风格的完整流程
        arm.execute_basic_sequence_c_style()
        
        # 测试预定义姿态
        print("\n测试预定义姿态...")
        poses_to_test = ["rest", "open_arms", "forward_reach", "defensive", "greeting"]
        
        for pose_name in poses_to_test:
            print(f"设置姿态: {pose_name}")
            arm.set_arm_pose(pose_name)
            time.sleep(2.0)
        
        # 返回零位
        arm.set_arm_pose("rest")
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"测试失败: {e}")
        print("请检查:")
        print("1. 网络接口配置（默认eth0）")
        print("2. 机器人连接状态")
        print("3. DDS通信设置")
        import traceback
        traceback.print_exc()


def main():
    """主函数 - 模仿C++版本的主函数"""
    print(" --- Unitree Robotics --- ")
    print("     G1 Arm Control Example      \n")
    
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface [enable_waist]")
        print("Example: python3 arm_client.py eth0")
        print("Example: python3 arm_client.py eth0 true  # 启用腰部控制（不推荐）")
        print("\n注意: 请根据实际网络配置修改接口参数")
        return -1
    
    network_interface = sys.argv[1]
    enable_waist = len(sys.argv) > 2 and sys.argv[2].lower() == "true"
    
    if enable_waist:
        print("警告: 启用腰部控制可能影响机器人稳定性")
        confirm = input("确认继续? (y/N): ").strip().lower()
        if confirm != 'y':
            print("已取消")
            return 0
    
    try:
        # 创建G1手臂客户端
        with g1_arm_connection(interface=network_interface, enable_waist=enable_waist) as arm:
            print(f"\n成功初始化 G1 手臂控制器")
            print(f"网络接口: {network_interface}")
            print(f"腰部控制: {'启用' if enable_waist else '禁用（推荐）'}")
            print("Commands:")
            print("  1 - 执行基础序列（C++风格）")
            print("  2 - 设置预定义姿态")
            print("  3 - 查看当前状态")
            print("  4 - 打印关节详细状态")
            print("  q - 退出")
            
            # 简单的命令循环
            while True:
                try:
                    cmd = input("\n请输入命令: ").strip().lower()
                    
                    if cmd == 'q':
                        print("退出程序...")
                        break
                    elif cmd == '1':
                        print("执行基础序列（C++风格）...")
                        arm.execute_basic_sequence_c_style()
                    elif cmd == '2':
                        print("可用姿态: rest, open_arms, forward_reach, defensive, greeting")
                        pose = input("请输入姿态名称: ").strip()
                        arm.set_arm_pose(pose)
                    elif cmd == '3':
                        print("读取当前状态...")
                        positions = arm.get_current_joint_positions()
                        if positions:
                            print(f"当前关节位置: {[f'{pos:.3f}' for pos in positions]}")
                        else:
                            print("无法读取关节位置")
                    elif cmd == '4':
                        print("关节详细状态:")
                        arm.print_joint_states()
                    else:
                        print("未知命令")
                        
                except KeyboardInterrupt:
                    print("\n收到中断信号，停止控制...")
                    arm.stop_control()
                    break
        
        print("程序正常退出")
        return 0
        
    except Exception as e:
        print(f"程序执行失败: {e}")
        print("请检查:")
        print("1. 网络接口配置")
        print("2. 机器人连接状态") 
        print("3. unitree_sdk2py库安装")
        import traceback
        traceback.print_exc()
        return -1


if __name__ == "__main__":
    if len(sys.argv) == 1:
        # 如果没有参数，运行测试函数
        test_g1_arm_basic_control()
    else:
        # 如果有参数，运行主函数
        sys.exit(main())