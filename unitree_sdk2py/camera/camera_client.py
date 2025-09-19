#!/usr/bin/env python3
"""
realsense_viewer.py
===================

从本地 RealSense 设备（例如 D435i）捕获 RGB 和深度图像，并在一个 OpenCV 窗口中显示合成的图像（RGB 和彩色深度图）。
支持设备可用性检查、USB 重置和权限验证。按 ESC 或 Q 键退出。
新增功能：
1. get_images()：返回最新的 RGB、原始深度和伪彩色深度图像。
2. save_images()：保存 RGB 和深度图像到指定目录（以时间戳命名）。

功能：
1. 初始化 RealSense 设备，捕获 640x480@30FPS 的 RGB 和深度流。
2. 应用空间和时间滤波器以提高深度图像质量。
3. 将深度图像转换为伪彩色（JET 色图）并与 RGB 图像水平拼接。
4. 显示 FPS 信息。
5. 包含设备占用检查和重试机制。
6. 提供图像获取和保存接口。

依赖：
- pyrealsense2 (`pip install pyrealsense2`)
- opencv-python (`pip install opencv-python`)
- numpy
- 系统命令：lsof（用于设备检查，可选）

运行：
    python realsense_viewer.py [--width WIDTH] [--height HEIGHT] [--fps FPS] [--output-dir OUTPUT_DIR]

示例：
    python realsense_viewer.py --width 640 --height 480 --fps 30 --output-dir ./images
    # 按 S 键保存当前图像
    # 使用 get_images() 获取图像数据
"""

from __future__ import annotations

import argparse
import sys
import time
import subprocess
import os
import grp
from typing import Optional, Tuple
from datetime import datetime

try:
    import pyrealsense2 as rs
    import numpy as np
    import cv2
except ImportError as exc:
    raise SystemExit(
        "依赖缺失。请安装：pip install pyrealsense2 opencv-python numpy"
    ) from exc


def check_device_availability() -> bool:
    """
    检查 RealSense 设备是否被其他进程占用。

    Returns:
        bool: 如果设备可用，返回 True；否则返回 False。
    """
    try:
        result = subprocess.run(['lsof', '/dev/video*'], capture_output=True, text=True)
        if result.stdout:
            print("[realsense_viewer] 警告: 检测到摄像头设备被占用:")
            print(result.stdout)
            return False
        return True
    except FileNotFoundError:
        print("[realsense_viewer] lsof 命令不可用，跳过设备检查")
        return True
    except Exception as e:
        print(f"[realsense_viewer] 设备检查失败: {e}")
        return True


def check_video_permissions() -> bool:
    """
    检查当前用户是否有访问视频设备的权限。

    Returns:
        bool: 如果有权限返回 True，否则返回 False。
    """
    try:
        video_gid = grp.getgrnam('video').gr_gid
        user_groups = os.getgroups()
        if video_gid in user_groups:
            return True
        else:
            print("[realsense_viewer] 警告: 当前用户不在 video 组中")
            print("建议运行: sudo usermod -a -G video $USER")
            print("然后重新登录或重启系统")
            return False
    except KeyError:
        print("[realsense_viewer] video 组不存在，跳过权限检查")
        return True
    except Exception as e:
        print(f"[realsense_viewer] 权限检查失败: {e}")
        return True


def reset_usb_devices() -> None:
    """
    重置 USB 摄像头设备（用户级操作）。

    注意：某些操作可能需要管理员权限。
    """
    try:
        print("[realsense_viewer] 正在尝试重置 USB 摄像头设备...")
        ctx = rs.context()
        devices = ctx.query_devices()
        for device in devices:
            try:
                device.hardware_reset()
                print(f"[realsense_viewer] 已重置设备: {device.get_info(rs.camera_info.name)}")
                time.sleep(2)
            except Exception as e:
                print(f"[realsense_viewer] 重置设备失败: {e}")
        print("[realsense_viewer] 设备重置尝试完成")
    except Exception as e:
        print(f"[realsense_viewer] 重置 USB 设备失败: {e}")
        print("提示: 如果经常遇到设备占用问题，请考虑：")
        print("1. 将用户添加到 video 组: sudo usermod -a -G video $USER")
        print("2. 重启系统以应用组权限变更")
        print("3. 检查其他可能占用摄像头的程序")


def get_first_device(context: rs.context) -> Optional[rs.device]:
    """
    返回第一个 RealSense 设备，如果没有设备则返回 None。

    Args:
        context: RealSense 上下文。

    Returns:
        Optional[rs.device]: 第一个 RealSense 设备。
    """
    devices = context.query_devices()
    if len(devices) == 0:
        return None
    return devices[0]


def colourise_depth(depth_frame: rs.depth_frame) -> np.ndarray:
    """
    将深度帧（16-bit，单位毫米）转换为伪彩色 8-bit BGR 图像。

    Args:
        depth_frame: RealSense 深度帧。

    Returns:
        np.ndarray: 伪彩色深度图像。
    """
    depth_data = np.asanyarray(depth_frame.get_data())
    depth_image = cv2.convertScaleAbs(depth_data, alpha=0.03)
    depth_image_bgr = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
    return cv2.applyColorMap(depth_image_bgr, cv2.COLORMAP_JET)


def run_realsense_viewer(
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    output_dir: str = "data/images"
) -> None:
    """
    运行 RealSense 图像捕获和显示。

    Args:
        width (int): RGB 和深度流的宽度。
        height (int): RGB 和深度流的高度。
        fps (int): 帧率。
        output_dir (str): 保存图像的目录。
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    # 权限检查
    check_video_permissions()

    # 设备检查和重试机制
    max_retries = 3
    for attempt in range(max_retries):
        try:
            print(f"[realsense_viewer] 尝试启动 RealSense (第 {attempt + 1}/{max_retries} 次)...")
            
            if not check_device_availability():
                print("[realsense_viewer] 设备被占用，尝试重置...")
                reset_usb_devices()

            # 检查设备连接
            ctx = rs.context()
            device = get_first_device(ctx)
            if device is None:
                raise RuntimeError("未找到 RealSense 设备")

            print(f"[realsense_viewer] 找到设备: {device.get_info(rs.camera_info.name)}")
            print(f"[realsense_viewer] 序列号: {device.get_info(rs.camera_info.serial_number)}")
            print(f"[realsense_viewer] 固件版本: {device.get_info(rs.camera_info.firmware_version)}")

            # 初始化 RealSense
            pipeline = rs.pipeline(ctx)
            config = rs.config()
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

            # 应用深度后处理滤波器
            spatial_filter = rs.spatial_filter()
            temporal_filter = rs.temporal_filter()
            align_to = rs.stream.color
            align = rs.align(align_to)

            print("[realsense_viewer] 正在启动 RealSense 管道...")
            profile = pipeline.start(config)
            print("[realsense_viewer] RealSense 管道已启动。")

            # 获取相机内参
            colour_intr = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = colour_intr.get_intrinsics()
            print(f"[realsense_viewer] 相机内参: {intr.width}×{intr.height}, fx={intr.fx:.1f}, fy={intr.fy:.1f}")

            break  # 成功初始化，跳出重试循环

        except RuntimeError as e:
            if "Device or resource busy" in str(e) or "xioctl" in str(e):
                print(f"[realsense_viewer] 设备忙碌错误: {e}")
                if attempt < max_retries - 1:
                    print("[realsense_viewer] 等待并重试...")
                    time.sleep(2)
                    reset_usb_devices()
                else:
                    print("[realsense_viewer] 所有重试都失败了")
                    print("\n故障排除建议:")
                    print("1. 确保没有其他程序在使用摄像头")
                    print("2. 检查 USB 连接是否稳定")
                    print("3. 尝试重新插拔摄像头")
                    print("4. 重启系统以清理设备状态")
                    sys.exit(1)
            else:
                print(f"[realsense_viewer] RealSense 初始化失败: {e}")
                sys.exit(1)

    # 初始化图像存储
    latest_images = {
        "rgb": None,
        "depth_raw": None,
        "depth_colored": None
    }

    def get_images() -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        返回最新的 RGB、原始深度和伪彩色深度图像。

        Returns:
            Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
                - RGB 图像 (BGR 格式, uint8)
                - 原始深度图像 (uint16, 单位毫米)
                - 伪彩色深度图像 (BGR 格式, uint8)
        """
        return (
            latest_images["rgb"].copy() if latest_images["rgb"] is not None else None,
            latest_images["depth_raw"].copy() if latest_images["depth_raw"] is not None else None,
            latest_images["depth_colored"].copy() if latest_images["depth_colored"] is not None else None
        )

    def save_images(output_dir: str) -> None:
        """
        保存当前 RGB 和深度图像到指定目录，文件名包含时间戳。

        Args:
            output_dir (str): 保存图像的目录。
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        if latest_images["rgb"] is not None:
            rgb_path = os.path.join(output_dir, f"rgb_{timestamp}.png")
            cv2.imwrite(rgb_path, latest_images["rgb"])
            print(f"[realsense_viewer] RGB 图像已保存到: {rgb_path}")
        if latest_images["depth_colored"] is not None:
            depth_path = os.path.join(output_dir, f"depth_{timestamp}.png")
            cv2.imwrite(depth_path, latest_images["depth_colored"])
            print(f"[realsense_viewer] 深度图像已保存到: {depth_path}")
        if latest_images["rgb"] is None and latest_images["depth_colored"] is None:
            print("[realsense_viewer] 没有可保存的图像")

    # 主循环
    last = time.perf_counter()
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                depth_frame: rs.depth_frame = aligned_frames.get_depth_frame()
                color_frame: rs.video_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    time.sleep(0.01)
                    continue

                # 应用后处理滤波器
                depth_frame = spatial_filter.process(depth_frame)
                depth_frame = temporal_filter.process(depth_frame)

                # 转换图像
                color_image = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_colored = colourise_depth(depth_frame)

                # 更新最新图像
                latest_images["rgb"] = color_image
                latest_images["depth_raw"] = depth_raw
                latest_images["depth_colored"] = depth_colored

                # 合成图像
                combo = cv2.hconcat([color_image, depth_colored])

                # 添加 FPS 信息和控制提示
                fps = 1.0 / (time.perf_counter() - last)
                last = time.perf_counter()
                cv2.putText(
                    combo, f"RGB+Depth  {fps:5.1f} FPS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
                )
                cv2.putText(
                    combo, "按 S 保存图像, ESC/Q 退出", (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
                )

                # 显示
                cv2.imshow("RealSense RGB + Depth", combo)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("[realsense_viewer] 用户请求退出...")
                    break
                elif key == ord("s"):
                    save_images(output_dir)

            except Exception as e:
                print(f"[realsense_viewer] 帧处理错误: {e}")
                time.sleep(0.01)
                continue

    finally:
        print("[realsense_viewer] 正在停止 RealSense 管道...")
        pipeline.stop()
        cv2.destroyAllWindows()


def main() -> None:
    """
    主函数，解析命令行参数并运行 RealSense 查看器。
    """
    parser = argparse.ArgumentParser(
        description="RealSense 图像查看器",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--width", type=int, default=640, help="RGB 和深度流的宽度")
    parser.add_argument("--height", type=int, default=480, help="RGB 和深度流的高度")
    parser.add_argument("--fps", type=int, default=30, help="帧率")
    parser.add_argument("--output-dir", type=str, default="data/images", help="保存图像的目录")
    args = parser.parse_args()

    try:
        run_realsense_viewer(
            width=args.width,
            height=args.height,
            fps=args.fps,
            output_dir=args.output_dir
        )
    except KeyboardInterrupt:
        print("\n[realsense_viewer] 接收到中断信号，正在退出...")
    except Exception as e:
        print(f"[realsense_viewer] 程序异常退出: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()