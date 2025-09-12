"""
Dex3 灵巧手控制模块

该模块提供了对宇树 Dex3-1 力控灵巧手的完整控制接口。

主要功能:
- 7自由度关节控制
- 触觉传感器数据读取
- 预定义手势库
- 左右手支持
- 多种控制模式

使用示例:
    from unitree_sdk2py.dex3 import Dex3Client, Dex3Gestures
    
    # 创建控制器
    dex3 = Dex3Client(hand="right", interface="eth0")
    
    # 执行手势
    angles = Dex3Gestures.get_gesture("open", "right")
    dex3.set_joint_angles(angles)
    
    # 读取状态
    state = dex3.read_state()
"""

from .dex3_client import Dex3Client, Dex3Config, Dex3Gestures

__all__ = [
    "Dex3Client",
    "Dex3Config", 
    "Dex3Gestures",
]

# 版本信息
__version__ = "1.0.0"