from dex3_client import Dex3Client

def test_motor_control():
    """测试获取和设置电机数据的功能"""
    print("=== 测试电机数据获取和设置 ===")

    try:
        # 创建右手控制器
        dex3 = Dex3Client(hand="right", interface="eth0")

        # 等待连接稳定
        print("等待连接稳定...")
        import time
        time.sleep(3.0)

        # 获取电机数据
        print("获取电机数据...")
        motor_data = dex3.get_motor_data(timeout=2.0)
        if motor_data:
            for i, data in enumerate(motor_data):
                print(f"电机 {i}: 角度={data['q']:.3f}, 速度={data['dq']:.3f}, 扭矩={data['tau']:.3f}")
        else:
            print("未能获取电机数据")

        # 设置电机数据
        print("\n设置电机数据...")
        target_data = [
            {'q': 0.5, 'dq': 0.0, 'tau': 0.0},
            {'q': 0.6, 'dq': 0.0, 'tau': 0.0},
            {'q': 0.7, 'dq': 0.0, 'tau': 0.0},
            {'q': 0.8, 'dq': 0.0, 'tau': 0.0},
            {'q': 0.9, 'dq': 0.0, 'tau': 0.0},
            {'q': 1.0, 'dq': 0.0, 'tau': 0.0},
            {'q': 1.1, 'dq': 0.0, 'tau': 0.0},
        ]
        success = dex3.set_motor_data(target_data)
        if success:
            print("电机数据设置成功")
        else:
            print("电机数据设置失败")

    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_motor_control()