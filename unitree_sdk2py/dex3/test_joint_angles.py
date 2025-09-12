#!/usr/bin/env python3
"""
Dex3 关节角度设置和获取测试脚本

该脚本用于验证 set_joint_angles 和 get_joint_angles 函数的正确性：
1. 测试能否正确获取当前关节角度
2. 测试能否正确设置关节角度
3. 验证设置后的角度是否与目标角度一致
4. 测试限位保护功能
5. 测试不同的控制参数
"""

import sys
import time
import math
from typing import List, Optional
from unitree_sdk2py.dex3.dex3_client import Dex3Client, dex3_connection


def test_get_joint_angles(dex3: Dex3Client) -> bool:
    """
    测试获取关节角度功能
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试获取关节角度 ===")
    
    # 测试多次获取，确保数据稳定性
    for i in range(5):
        print(f"第 {i+1} 次获取关节角度...")
        angles = dex3.get_joint_angles(timeout=2.0)
        
        if angles is None:
            print(f"  ❌ 第 {i+1} 次获取失败")
            return False
        
        if len(angles) != 7:
            print(f"  ❌ 角度数量错误: 期望7个，获得{len(angles)}个")
            return False
        
        print(f"  ✅ 成功获取: {[f'{a:.3f}' for a in angles]}")
        
        # 检查角度是否在合理范围内
        limits = dex3._get_joint_limits()
        for j, (angle, (min_val, max_val)) in enumerate(zip(angles, limits)):
            if not (min_val <= angle <= max_val):
                print(f"  ⚠️  关节{j}角度{angle:.3f}超出限位[{min_val:.3f}, {max_val:.3f}]")
        
        time.sleep(0.5)
    
    print("✅ 获取关节角度测试通过\n")
    return True


def test_set_joint_angles_basic(dex3: Dex3Client) -> bool:
    """
    测试基础关节角度设置功能
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试基础关节角度设置 ===")
    
    # 获取初始角度
    initial_angles = dex3.get_joint_angles(timeout=2.0)
    if initial_angles is None:
        print("❌ 无法获取初始角度")
        return False
    
    print(f"初始角度: {[f'{a:.3f}' for a in initial_angles]}")
    
    # 测试设置到零位
    print("设置到零位...")
    zero_angles = [0.0] * 7
    success = dex3.set_joint_angles(zero_angles)
    
    if not success:
        print("❌ 设置零位失败")
        return False
    
    # 等待执行
    time.sleep(2.0)
    
    # 验证设置结果
    current_angles = dex3.get_joint_angles(timeout=2.0)
    if current_angles is None:
        print("❌ 设置后无法获取角度")
        return False
    
    print(f"设置后角度: {[f'{a:.3f}' for a in current_angles]}")
    
    # 检查误差
    max_error = 0.0
    for i, (target, actual) in enumerate(zip(zero_angles, current_angles)):
        error = abs(target - actual)
        max_error = max(max_error, error)
        print(f"  关节{i}: 目标={target:.3f}, 实际={actual:.3f}, 误差={error:.3f}")
    
    print(f"最大误差: {max_error:.3f} rad")
    
    if max_error > 0.1:  # 允许0.1弧度的误差
        print("⚠️  误差较大，可能需要更长的稳定时间")
    else:
        print("✅ 设置精度良好")
    
    print("✅ 基础关节角度设置测试通过\n")
    return True


def test_set_joint_angles_sequence(dex3: Dex3Client) -> bool:
    """
    测试关节角度序列设置
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试关节角度序列设置 ===")
    
    # 获取关节限位
    limits = dex3._get_joint_limits()
    
    # 定义测试序列（安全的角度值）
    test_sequences = [
        {
            'name': '中间位置',
            'angles': [(min_val + max_val) / 2.0 for min_val, max_val in limits]
        },
        {
            'name': '25%位置',
            'angles': [min_val + 0.25 * (max_val - min_val) for min_val, max_val in limits]
        },
        {
            'name': '75%位置', 
            'angles': [min_val + 0.75 * (max_val - min_val) for min_val, max_val in limits]
        },
        {
            'name': '零位',
            'angles': [0.0] * 7
        }
    ]
    
    for seq in test_sequences:
        print(f"设置到{seq['name']}...")
        print(f"目标角度: {[f'{a:.3f}' for a in seq['angles']]}")
        
        # 设置角度
        success = dex3.set_joint_angles(seq['angles'])
        if not success:
            print(f"❌ 设置{seq['name']}失败")
            return False
        
        # 等待稳定
        time.sleep(3.0)
        
        # 验证结果
        actual_angles = dex3.get_joint_angles(timeout=2.0)
        if actual_angles is None:
            print(f"❌ 设置{seq['name']}后无法获取角度")
            return False
        
        print(f"实际角度: {[f'{a:.3f}' for a in actual_angles]}")
        
        # 计算误差
        errors = [abs(target - actual) for target, actual in zip(seq['angles'], actual_angles)]
        max_error = max(errors)
        print(f"最大误差: {max_error:.3f} rad")
        
        if max_error > 0.15:
            print(f"⚠️  {seq['name']}误差较大: {max_error:.3f}")
        else:
            print(f"✅ {seq['name']}设置成功")
        
        print()
    
    print("✅ 关节角度序列设置测试通过\n")
    return True


def test_joint_limits_protection(dex3: Dex3Client) -> bool:
    """
    测试关节限位保护功能
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试关节限位保护 ===")
    
    # 获取关节限位
    limits = dex3._get_joint_limits()
    
    # 测试超出上限
    print("测试超出上限保护...")
    over_max_angles = [max_val + 0.5 for min_val, max_val in limits]
    print(f"尝试设置超限角度: {[f'{a:.3f}' for a in over_max_angles]}")
    
    success = dex3.set_joint_angles(over_max_angles)
    if not success:
        print("❌ 设置超限角度失败")
        return False
    
    time.sleep(2.0)
    
    # 验证是否被限制
    actual_angles = dex3.get_joint_angles(timeout=2.0)
    if actual_angles is None:
        print("❌ 超限测试后无法获取角度")
        return False
    
    print(f"实际设置角度: {[f'{a:.3f}' for a in actual_angles]}")
    
    # 检查是否在限位内
    all_within_limits = True
    for i, (angle, (min_val, max_val)) in enumerate(zip(actual_angles, limits)):
        if not (min_val <= angle <= max_val):
            print(f"❌ 关节{i}仍超出限位: {angle:.3f} not in [{min_val:.3f}, {max_val:.3f}]")
            all_within_limits = False
    
    if all_within_limits:
        print("✅ 关节限位保护功能正常")
    else:
        print("❌ 关节限位保护功能异常")
        return False
    
    # 测试超出下限
    print("\n测试超出下限保护...")
    under_min_angles = [min_val - 0.5 for min_val, max_val in limits]
    print(f"尝试设置超限角度: {[f'{a:.3f}' for a in under_min_angles]}")
    
    success = dex3.set_joint_angles(under_min_angles)
    time.sleep(2.0)
    
    actual_angles = dex3.get_joint_angles(timeout=2.0)
    print(f"实际设置角度: {[f'{a:.3f}' for a in actual_angles]}")
    
    # 再次检查限位
    all_within_limits = True
    for i, (angle, (min_val, max_val)) in enumerate(zip(actual_angles, limits)):
        if not (min_val <= angle <= max_val):
            print(f"❌ 关节{i}仍超出限位: {angle:.3f} not in [{min_val:.3f}, {max_val:.3f}]")
            all_within_limits = False
    
    if all_within_limits:
        print("✅ 下限保护功能正常")
    else:
        print("❌ 下限保护功能异常")
        return False
    
    print("✅ 关节限位保护测试通过\n")
    return True


def test_control_parameters(dex3: Dex3Client) -> bool:
    """
    测试不同控制参数的效果
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试控制参数 ===")
    
    # 获取中间位置
    limits = dex3._get_joint_limits()
    target_angles = [(min_val + max_val) / 2.0 for min_val, max_val in limits]
    
    # 测试不同的kp, kd参数
    control_params = [
        {'name': '低增益', 'kp': 0.5, 'kd': 0.05},
        {'name': '中等增益', 'kp': 1.5, 'kd': 0.1},
        {'name': '高增益', 'kp': 3.0, 'kd': 0.2}
    ]
    
    for params in control_params:
        print(f"测试{params['name']} (kp={params['kp']}, kd={params['kd']})...")
        
        # 先移动到零位
        dex3.set_joint_angles([0.0] * 7)
        time.sleep(2.0)
        
        # 记录开始时间
        start_time = time.time()
        
        # 设置目标角度
        success = dex3.set_joint_angles(
            target_angles, 
            kp=params['kp'], 
            kd=params['kd']
        )
        
        if not success:
            print(f"❌ {params['name']}设置失败")
            return False
        
        # 监控收敛过程
        convergence_time = None
        for i in range(50):  # 最多监控5秒
            time.sleep(0.1)
            current_angles = dex3.get_joint_angles(timeout=0.5)
            
            if current_angles is None:
                continue
            
            # 计算误差
            errors = [abs(target - actual) for target, actual in zip(target_angles, current_angles)]
            max_error = max(errors)
            
            if max_error < 0.05:  # 收敛阈值
                convergence_time = time.time() - start_time
                break
        
        if convergence_time:
            print(f"  ✅ {params['name']}收敛时间: {convergence_time:.2f}秒")
        else:
            print(f"  ⚠️  {params['name']}未在5秒内收敛")
        
        time.sleep(1.0)
    
    print("✅ 控制参数测试通过\n")
    return True


def test_real_time_tracking(dex3: Dex3Client) -> bool:
    """
    测试实时跟踪性能
    
    Args:
        dex3: Dex3客户端实例
    
    Returns:
        bool: 测试是否成功
    """
    print("=== 测试实时跟踪性能 ===")
    
    # 获取限位
    limits = dex3._get_joint_limits()
    
    print("执行正弦波跟踪测试...")
    
    # 记录跟踪数据
    tracking_data = []
    
    start_time = time.time()
    duration = 10.0  # 10秒测试
    
    step_count = 0
    while time.time() - start_time < duration:
        t = time.time() - start_time
        
        # 生成正弦波目标角度
        target_angles = []
        for min_val, max_val in limits:
            mid = (min_val + max_val) / 2.0
            amplitude = (max_val - min_val) * 0.3  # 30%幅度
            angle = mid + amplitude * math.sin(2 * math.pi * 0.2 * t)  # 0.2Hz频率
            target_angles.append(angle)
        
        # 设置目标角度
        dex3.set_joint_angles(target_angles, kp=2.0, kd=0.15)
        
        # 获取实际角度
        actual_angles = dex3.get_joint_angles(timeout=0.1)
        
        if actual_angles:
            # 计算跟踪误差
            errors = [abs(target - actual) for target, actual in zip(target_angles, actual_angles)]
            max_error = max(errors)
            
            tracking_data.append({
                'time': t,
                'max_error': max_error,
                'target': target_angles.copy(),
                'actual': actual_angles.copy()
            })
        
        step_count += 1
        time.sleep(0.05)  # 20Hz控制频率
    
    # 分析跟踪性能
    if tracking_data:
        avg_error = sum(d['max_error'] for d in tracking_data) / len(tracking_data)
        max_error = max(d['max_error'] for d in tracking_data)
        
        print(f"跟踪步数: {step_count}")
        print(f"有效数据点: {len(tracking_data)}")
        print(f"平均跟踪误差: {avg_error:.4f} rad")
        print(f"最大跟踪误差: {max_error:.4f} rad")
        
        if avg_error < 0.1:
            print("✅ 跟踪性能良好")
        else:
            print("⚠️  跟踪误差较大")
    else:
        print("❌ 未获取到跟踪数据")
        return False
    
    print("✅ 实时跟踪测试通过\n")
    return True


def main():
    """主测试函数"""
    print("=" * 60)
    print("    Dex3 关节角度设置和获取功能测试")
    print("=" * 60)
    print()
    
    if len(sys.argv) < 2:
        print("用法: python3 test_joint_angles.py <网络接口>")
        print("示例: python3 test_joint_angles.py eth0")
        return -1
    
    network_interface = sys.argv[1]
    
    # 获取手部选择
    hand_input = input("请选择测试的手部 (L for left, R for right): ").strip().upper()
    
    if hand_input == "L":
        hand_side = "left"
    elif hand_input == "R":
        hand_side = "right"
    else:
        print("❌ 无效的手部选择，请输入 'L' 或 'R'")
        return -1
    
    print(f"\n开始测试 {hand_side} 手...")
    print("⚠️  请确保机器人处于安全环境!")
    input("按 Enter 键继续...")
    
    try:
        # 使用上下文管理器确保安全退出
        with dex3_connection(hand=hand_side, interface=network_interface) as dex3:
            print(f"✅ 成功连接到 {hand_side} 手\n")
            
            # 等待连接稳定
            print("等待连接稳定...")
            time.sleep(2.0)
            
            # 执行测试序列
            tests = [
                ("获取关节角度", test_get_joint_angles),
                ("基础角度设置", test_set_joint_angles_basic),
                ("角度序列设置", test_set_joint_angles_sequence),
                ("限位保护", test_joint_limits_protection),
                ("控制参数", test_control_parameters),
                ("实时跟踪", test_real_time_tracking)
            ]
            
            passed_tests = 0
            total_tests = len(tests)
            
            for test_name, test_func in tests:
                print(f"开始 {test_name} 测试...")
                try:
                    if test_func(dex3):
                        passed_tests += 1
                        print(f"✅ {test_name} 测试通过")
                    else:
                        print(f"❌ {test_name} 测试失败")
                except Exception as e:
                    print(f"❌ {test_name} 测试异常: {e}")
                    import traceback
                    traceback.print_exc()
                
                print("-" * 50)
            
            # 测试总结
            print("\n" + "=" * 60)
            print("测试结果总结")
            print("=" * 60)
            print(f"总测试数: {total_tests}")
            print(f"通过测试: {passed_tests}")
            print(f"失败测试: {total_tests - passed_tests}")
            print(f"成功率: {passed_tests/total_tests*100:.1f}%")
            
            if passed_tests == total_tests:
                print("🎉 所有测试通过! set_joint_angles 和 get_joint_angles 功能正常")
            else:
                print("⚠️  部分测试失败，请检查相关功能")
            
            # 最后安全停止
            print("\n最后安全停止电机...")
            dex3.stop_motors()
            time.sleep(1.0)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n收到中断信号，安全退出...")
        return 0
    except Exception as e:
        print(f"测试执行失败: {e}")
        import traceback
        traceback.print_exc()
        return -1


if __name__ == "__main__":
    sys.exit(main())