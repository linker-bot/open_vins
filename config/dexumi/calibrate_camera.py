#!/usr/bin/env python3
"""
cam1 (/dev/video2) camera intrinsics calibration script
cam1 (/dev/video2) 相机内参标定脚本

Uses OpenCV chessboard calibration: auto-detect and capture calibration frames,
then write results to kalibr_imucam_chain.yaml.
使用 OpenCV 棋盘格标定法，自动检测并采集标定图像，
标定完成后将结果写入 kalibr_imucam_chain.yaml。

Prerequisites / 准备工作:
  1) Print a 9x6 inner-corner (10x7 square) chessboard and mount on cardboard
     打印 9x6 内角点（10x7 方格）的棋盘格，贴到硬纸板上
  2) Ensure /dev/video2 is available and not in use by other apps
     确保 /dev/video2 可用且未被其他程序占用

Usage / 用法:
  python3 calibrate_camera.py
  python3 calibrate_camera.py --device /dev/video2 --rows 6 --cols 9 --square_size 25.0

Controls / 操作:
  - Place chessboard in view with varied poses / 将棋盘格放在相机前，覆盖画面不同区域和角度
  - Auto-detect and capture (green border = detected) / 程序自动检测并采集
  - After 20+ frames, press 'q' to run calibration / 采集 20 帧以上后按 'q' 开始标定
  - 'c' manual capture / 按 'c' 可随时手动触发采集
  - 'r' clear all captures / 按 'r' 可清除所有已采集的帧重新开始
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="cam1 camera intrinsics calibration / cam1 相机内参标定"
    )
    parser.add_argument("--device", type=str, default="/dev/video2",
                        help="V4L2 device path / V4L2 设备路径 (default /dev/video2)")
    parser.add_argument("--rows", type=int, default=9,
                        help="Chessboard inner corners (rows) / 棋盘格内角点行数 (default: 9)")
    parser.add_argument("--cols", type=int, default=6,
                        help="Chessboard inner corners (cols) / 棋盘格内角点列数 (default: 6)")
    parser.add_argument("--square_size", type=float, default=25.0,
                        help="Square size in mm / 每格边长 mm (default: 25.0)")
    parser.add_argument("--width", type=int, default=640,
                        help="Image width / 图像宽度 (default: 640)")
    parser.add_argument("--height", type=int, default=480,
                        help="Image height / 图像高度 (default: 480)")
    parser.add_argument("--min_frames", type=int, default=20,
                        help="Minimum captured frames / 最少采集帧数 (default: 20)")
    parser.add_argument("--auto_interval", type=float, default=1.5,
                        help="Min seconds between auto captures / 自动采集最小间隔秒数 (default: 1.5)")
    parser.add_argument("--output_yaml", type=str, default=None,
                        help="Output YAML path (default: kalibr_imucam_chain.yaml in script dir) / 输出 YAML 路径")
    return parser.parse_args()


def open_camera(device, width, height):
    """Open V4L2 camera and set capture properties / 打开 V4L2 相机并设置参数"""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[Error] Cannot open device / [错误] 无法打开设备: {device}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Info] Opened / [信息] 已打开 {device} | resolution / 分辨率: {actual_w}x{actual_h}")
    return cap


def corners_moved_enough(new_corners, last_corners, threshold=30.0):
    """Return True if chessboard moved enough vs last capture / 检查棋盘格是否相比上次移动了足够的距离"""
    if last_corners is None:
        return True
    diff = np.mean(np.abs(new_corners - last_corners))
    return diff > threshold


def draw_info(frame, num_captured, min_frames, detecting, just_captured):
    """Draw status overlay on frame / 在画面上绘制提示信息"""
    h, w = frame.shape[:2]

    # Status bar background / 状态栏背景
    cv2.rectangle(frame, (0, 0), (w, 60), (40, 40, 40), -1)

    # Capture count / 采集计数
    color = (0, 255, 0) if num_captured >= min_frames else (0, 200, 255)
    cv2.putText(frame, f"Captured: {num_captured}/{min_frames}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Status text / 状态提示
    if just_captured:
        cv2.putText(frame, "** CAPTURED **",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    elif detecting:
        cv2.putText(frame, "Chessboard detected - hold steady...",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    else:
        cv2.putText(frame, "Move chessboard into view",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 1)

    # Key hints / 操作提示
    hint = "q: calibrate & quit | c: manual capture | r: reset"
    cv2.putText(frame, hint, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    # Flash border on successful capture / 采集成功闪烁边框
    if just_captured:
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 255, 0), 4)


def run_calibration(obj_points_list, img_points_list, image_size):
    """Run OpenCV camera calibration / 执行 OpenCV 相机标定"""
    print("\n" + "=" * 60)
    print(f"Starting calibration / 开始标定... ({len(obj_points_list)} frames / 帧)")
    print("=" * 60)

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points_list, img_points_list, image_size, None, None
    )

    # Reprojection error / 计算重投影误差
    total_error = 0
    for i in range(len(obj_points_list)):
        projected, _ = cv2.projectPoints(
            obj_points_list[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(img_points_list[i], projected, cv2.NORM_L2)
        total_error += error * error / len(projected)
    mean_error = np.sqrt(total_error / len(obj_points_list))

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    k1, k2, p1, p2 = dist_coeffs[0, 0], dist_coeffs[0, 1], dist_coeffs[0, 2], dist_coeffs[0, 3]

    print(f"\nCalibration result / 标定结果:")
    print(f"  fx = {fx:.4f}")
    print(f"  fy = {fy:.4f}")
    print(f"  cx = {cx:.4f}")
    print(f"  cy = {cy:.4f}")
    print(f"  Distortion (k1,k2,p1,p2) / 畸变系数 = [{k1:.6f}, {k2:.6f}, {p1:.6f}, {p2:.6f}]")
    print(f"  Mean reprojection error (px) / 平均重投影误差 = {mean_error:.4f}")
    print()

    if mean_error > 1.0:
        print("[Warn] High reprojection error (>1.0 px); capture more varied poses /")
        print("[警告] 重投影误差偏大 (>1.0 px)，建议重新采集更多不同角度的图像")
    elif mean_error < 0.5:
        print("[Good] Low error, calibration quality is excellent / [优秀] 重投影误差很小，标定质量良好")
    else:
        print("[OK] Acceptable calibration quality / [良好] 标定质量可接受")

    return {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "dist": [k1, k2, p1, p2],
        "mean_error": mean_error,
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
    }


def update_yaml(yaml_path, calib_result, image_size):
    """Update intrinsics and distortion in kalibr_imucam_chain.yaml / 更新 kalibr_imucam_chain.yaml 中的内参和畸变"""
    fx = calib_result["fx"]
    fy = calib_result["fy"]
    cx = calib_result["cx"]
    cy = calib_result["cy"]
    dist = calib_result["dist"]
    w, h = image_size

    # Read original file / 读取原文件
    with open(yaml_path, "r") as f:
        content = f.read()

    # Replace intrinsics line / 替换 intrinsics 行
    import re
    content = re.sub(
        r'(intrinsics:\s*)\[.*?\]',
        f'intrinsics: [{fx:.4f}, {fy:.4f}, {cx:.4f}, {cy:.4f}]',
        content
    )

    # Replace distortion_coeffs line / 替换 distortion_coeffs 行
    content = re.sub(
        r'(distortion_coeffs:\s*)\[.*?\]',
        f'distortion_coeffs: [{dist[0]:.6f}, {dist[1]:.6f}, {dist[2]:.6f}, {dist[3]:.6f}]',
        content
    )

    # Replace resolution line / 替换 resolution 行
    content = re.sub(
        r'(resolution:\s*)\[.*?\]',
        f'resolution: [{w}, {h}]',
        content
    )

    with open(yaml_path, "w") as f:
        f.write(content)

    print(f"\n[Done] Updated / [完成] 已更新: {yaml_path}")
    print(f"  intrinsics: [{fx:.4f}, {fy:.4f}, {cx:.4f}, {cy:.4f}]")
    print(f"  distortion_coeffs: [{dist[0]:.6f}, {dist[1]:.6f}, {dist[2]:.6f}, {dist[3]:.6f}]")


def main():
    args = parse_args()

    # Chessboard pattern / 棋盘格参数
    pattern_size = (args.cols, args.rows)
    square_size = args.square_size  # mm

    # World points on z=0 plane / 准备棋盘格世界坐标 (z=0 平面)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * square_size

    # Calibration buffers / 存储标定数据
    obj_points_list = []  # 3D world / 3D 世界坐标
    img_points_list = []  # 2D image / 2D 图像坐标
    last_capture_time = 0
    last_corners = None
    just_captured_until = 0

    # Open camera / 打开相机
    cap = open_camera(args.device, args.width, args.height)
    image_size = (args.width, args.height)

    print()
    print("=" * 60)
    print("  cam1 intrinsics calibration / cam1 相机内参标定")
    print("=" * 60)
    print(f"  Chessboard / 棋盘格: {args.cols}x{args.rows} inner corners / 内角点, square / 格边长 {square_size}mm")
    print(f"  Min frames / 最少采集: {args.min_frames}")
    print(f"  Auto-capture interval / 自动采集间隔: {args.auto_interval}s")
    print()
    print("  Instructions / 操作说明:")
    print("    - Place board with varied poses / 将棋盘格放在相机前，覆盖画面不同区域和角度")
    print("    - Auto-detect and capture / 程序自动检测并采集")
    print("    - Press 'q' when enough frames / 采集足够帧数后按 'q' 开始标定")
    print("    - 'c' manual | 'r' reset | 'q' calibrate & quit / 'c' 手动采集 | 'r' 重置 | 'q' 标定并退出")
    print("=" * 60)
    print()

    # Sub-pixel refinement criteria / 亚像素精化参数
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[Error] Frame read failed / [错误] 读取帧失败")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()
        now = time.time()

        # Detect chessboard / 检测棋盘格
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

        detecting = False
        manual_capture = False

        if found:
            detecting = True
            # Sub-pixel refinement / 亚像素精化
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # Draw detection / 绘制检测结果
            cv2.drawChessboardCorners(display, pattern_size, corners_refined, found)

            # Auto-capture rules / 自动采集判断
            time_ok = (now - last_capture_time) > args.auto_interval
            moved_ok = corners_moved_enough(corners_refined, last_corners)

            if time_ok and moved_ok:
                obj_points_list.append(objp.copy())
                img_points_list.append(corners_refined)
                last_capture_time = now
                last_corners = corners_refined.copy()
                just_captured_until = now + 0.5
                num = len(obj_points_list)
                print(f"  [Auto capture / 自动采集] frame / 第 {num}")

        # Draw UI / 绘制 UI
        just_captured = now < just_captured_until
        draw_info(display, len(obj_points_list), args.min_frames, detecting, just_captured)

        cv2.imshow("Camera Calibration - cam1", display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('c') and found:
            # Manual capture / 手动采集
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points_list.append(objp.copy())
            img_points_list.append(corners_refined)
            last_capture_time = now
            last_corners = corners_refined.copy()
            just_captured_until = now + 0.5
            num = len(obj_points_list)
            print(f"  [Manual capture / 手动采集] frame / 第 {num}")

        elif key == ord('r'):
            # Reset / 重置
            obj_points_list.clear()
            img_points_list.clear()
            last_corners = None
            print("  [Reset / 重置] All captures cleared / 已清除所有采集数据")

        elif key == ord('q'):
            # Quit and calibrate / 退出并标定
            break

    cap.release()
    cv2.destroyAllWindows()

    # Check enough frames / 检查是否有足够的采集帧
    if len(obj_points_list) < 5:
        print(f"\n[Error] Only {len(obj_points_list)} frames; need at least 5 / [错误] 只采集了 {len(obj_points_list)} 帧，至少需要 5 帧才能标定")
        sys.exit(1)

    if len(obj_points_list) < args.min_frames:
        print(f"\n[Warn] Only {len(obj_points_list)} frames (recommend >= {args.min_frames}) / [警告] 只采集了 {len(obj_points_list)} 帧 (建议 >= {args.min_frames})")
        resp = input("Continue calibration? (y/n) / 是否继续标定? (y/n): ").strip().lower()
        if resp != 'y':
            print("Cancelled / 已取消")
            sys.exit(0)

    # Run calibration / 执行标定
    calib_result = run_calibration(obj_points_list, img_points_list, image_size)

    # Resolve output YAML path / 确定输出 YAML 路径
    if args.output_yaml:
        yaml_path = args.output_yaml
    else:
        script_dir = Path(__file__).parent.resolve()
        yaml_path = str(script_dir / "kalibr_imucam_chain.yaml")

    if os.path.exists(yaml_path):
        print(f"\nWrite calibration to {yaml_path}? / 是否将标定结果写入 {yaml_path}?")
        resp = input("(y/n): ").strip().lower()
        if resp == 'y':
            update_yaml(yaml_path, calib_result, image_size)
        else:
            print("Skipped write; update config manually / 已跳过写入，你可以手动更新配置文件")
    else:
        print(f"\n[Warn] YAML not found / [警告] YAML 文件不存在: {yaml_path}")
        print("Copy these into your config manually / 请手动将以下参数填入你的配置文件:")
        print(f"  intrinsics: [{calib_result['fx']:.4f}, {calib_result['fy']:.4f}, "
              f"{calib_result['cx']:.4f}, {calib_result['cy']:.4f}]")
        print(f"  distortion_coeffs: {calib_result['dist']}")

    print("\nCalibration done / 标定完成!")


if __name__ == "__main__":
    main()
