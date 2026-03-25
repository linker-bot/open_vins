#!/usr/bin/env python3
"""
Read infra1 intrinsics from RealSense camera_info and auto-write to kalibr_imucam_chain.yaml
从 RealSense camera_info 读取 infra1 内参并自动写入 kalibr_imucam_chain.yaml

Usage / 用法：
  1. Start D455 camera / 启动 D455 相机: ros2 launch ov_msckf dexumi_d455.launch.py
  2. In another terminal / 另开终端，运行: python3 update_infra1_intrinsics.py
  (ROS environment must be sourced / 需已 source ROS 环境)
"""

import os
import subprocess
import sys
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "kalibr_imucam_chain.yaml")
TOPIC = "/camera/camera/infra1/camera_info"
TIMEOUT = 15


def get_camera_info():
    """Get camera_info from ros2 topic echo / 从 ros2 topic echo 获取 camera_info"""
    try:
        out = subprocess.run(
            ["ros2", "topic", "echo", TOPIC, "--once"],
            capture_output=True,
            text=True,
            timeout=TIMEOUT,
        )
    except FileNotFoundError:
        print("Error: ros2 not found, please source /opt/ros/jazzy/setup.bash first")
        print("错误: 未找到 ros2，请先 source /opt/ros/jazzy/setup.bash")
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print(f"Error: no message received from {TOPIC} within {TIMEOUT}s")
        print(f"错误: {TIMEOUT}s 内未收到 {TOPIC}")
        print("Please start camera first: ros2 launch ov_msckf dexumi_d455.launch.py")
        print("请先启动相机: ros2 launch ov_msckf dexumi_d455.launch.py")
        sys.exit(1)

    if out.returncode != 0 or not out.stdout.strip():
        print(f"Error: no camera_info received, make sure camera is running and publishing {TOPIC}")
        print(f"错误: 未收到 camera_info，请确保相机已启动并发布 {TOPIC}")
        sys.exit(1)

    if out.stderr and "WARNING" in out.stderr:
        print(out.stderr.strip())

    # ros2 topic echo may output multiple YAML documents, take the first one
    # ros2 topic echo 可能输出多段 YAML，取第一段
    raw = out.stdout.split("\n---\n")[0].strip()
    data = yaml.safe_load(raw)
    if not data:
        print("Error: failed to parse camera_info")
        print("错误: 无法解析 camera_info")
        sys.exit(1)

    K = data.get("k", data.get("K", []))
    D = data.get("d", data.get("D", []))
    if len(K) != 9:
        print(f"Error: unexpected K format, expected 9 elements, got {len(K)}")
        print(f"错误: K 格式异常，期望 9 个元素，得到 {len(K)}")
        sys.exit(1)

    fx, fy = K[0], K[4]
    cx, cy = K[2], K[5]
    distortion = D[:4] if len(D) >= 4 else D
    return fx, fy, cx, cy, distortion


def update_config(fx, fy, cx, cy, distortion):
    """Update kalibr_imucam_chain.yaml while preserving original format / 更新 kalibr_imucam_chain.yaml，保持原有格式"""
    import re

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    content = re.sub(
        r"intrinsics:\s*\[[^\]]+\]",
        f"intrinsics: [{fx}, {fy}, {cx}, {cy}]",
        content,
    )
    d_str = ", ".join(str(x) for x in distortion)
    content = re.sub(
        r"distortion_coeffs:\s*\[[^\]]*\]",
        f"distortion_coeffs: [{d_str}]",
        content,
    )

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    print(f"Waiting for {TOPIC} ...")
    print(f"等待 {TOPIC} ...")
    fx, fy, cx, cy, distortion = get_camera_info()
    print(f"  intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print(f"  distortion_coeffs: {distortion}")

    update_config(fx, fy, cx, cy, distortion)
    print(f"\nWritten to {CONFIG_PATH}")
    print(f"已写入 {CONFIG_PATH}")


if __name__ == "__main__":
    main()
