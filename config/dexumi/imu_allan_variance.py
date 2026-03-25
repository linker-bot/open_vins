#!/usr/bin/env python3
"""
IMU Noise Parameter Allan Variance Analysis Tool
IMU 噪声参数 Allan 方差分析工具

Features / 功能:
  1. record  - Subscribe to ROS2 IMU topic, record static data to CSV file
               订阅 ROS2 IMU 话题，录制静态数据到 CSV 文件
  2. analyze - Perform Allan variance analysis on recorded CSV data, extract noise parameters
               对录制的 CSV 数据进行 Allan 方差分析，提取噪声参数

Usage / 用法:
  # Step 1: Record IMU static data (default 2 hours)
  # 第一步: 录制 IMU 静态数据 (默认 2 小时)
  python3 imu_allan_variance.py record --duration 7200 --output imu_static_data.csv

  # Step 2: Analyze recorded data
  # 第二步: 分析录制的数据
  python3 imu_allan_variance.py analyze --input imu_static_data.csv

Dependencies / 依赖:
  - record mode requires / record 模式需要: rclpy, sensor_msgs (ROS2 environment / ROS2 环境)
  - analyze mode requires / analyze 模式需要: numpy, matplotlib (no ROS2 needed / 无需 ROS2)
"""

import argparse
import csv
import os
import signal
import sys
import time
from pathlib import Path


# ============================================================================
#  Recording module (requires ROS2) / 录制模块 (需要 ROS2)
# ============================================================================

class IMURecorder:
    """
    Subscribe to /imu/data topic and record IMU data to CSV file.
    订阅 /imu/data 话题，将 IMU 数据录制到 CSV 文件。
    """

    def __init__(self, topic, output_file, duration, accel_unit):
        """
        @brief Initialize IMU recorder / 初始化 IMU 录制器
        @param topic        IMU topic name / IMU 话题名称
        @param output_file  Output CSV file path / 输出 CSV 文件路径
        @param duration     Recording duration in seconds / 录制时长 (秒)
        @param accel_unit   Acceleration unit ("g" or "m/s2") / 加速度单位 ("g" 或 "m/s2")
        """
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Imu

        self.rclpy = rclpy
        self.duration = duration
        self.accel_unit = accel_unit
        self.output_file = output_file
        self.sample_count = 0
        self.start_time = None
        self.running = True

        # Open CSV file / 打开 CSV 文件
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nsec',
            'gyro_x', 'gyro_y', 'gyro_z',
            'accel_x', 'accel_y', 'accel_z',
            'accel_unit'
        ])

        # Initialize ROS2 / 初始化 ROS2
        rclpy.init()
        self.node = rclpy.create_node('imu_recorder')

        # QoS settings - compatible with both RELIABLE and BEST_EFFORT / QoS 设置 - 同时兼容 RELIABLE 和 BEST_EFFORT
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )

        self.subscription = self.node.create_subscription(
            Imu, topic, self.imu_callback, qos
        )

        self.node.get_logger().info(f'Starting IMU recording... / 开始录制 IMU 数据...')
        self.node.get_logger().info(f'  Topic / 话题:     {topic}')
        self.node.get_logger().info(f'  Output / 输出文件: {output_file}')
        self.node.get_logger().info(f'  Planned duration / 计划时长: {duration} s / 秒 ({duration/3600:.1f} h / 小时)')
        self.node.get_logger().info(f'  Accel unit / 加速度单位: {accel_unit}')
        self.node.get_logger().info(f'  Press Ctrl+C to stop early / 按 Ctrl+C 可提前停止录制')

    def imu_callback(self, msg):
        """
        @brief IMU message callback, write data to CSV / IMU 消息回调，将数据写入 CSV
        @param msg  sensor_msgs/Imu message / sensor_msgs/Imu 消息
        """
        if not self.running:
            return

        if self.start_time is None:
            self.start_time = time.time()

        # Write data / 写入数据
        self.csv_writer.writerow([
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            self.accel_unit
        ])

        self.sample_count += 1

        # Print progress every 1000 samples / 每 1000 个样本打印一次进度
        if self.sample_count % 1000 == 0:
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            remaining = self.duration - elapsed
            self.node.get_logger().info(
                f'Recorded / 已录制: {self.sample_count} samples / 样本 | '
                f'Rate / 采样率: {rate:.1f} Hz | '
                f'Elapsed / 已用时: {elapsed:.0f}s | '
                f'Remaining / 剩余: {max(0, remaining):.0f}s'
            )

        # Check if target duration is reached / 检查是否达到目标时长
        if self.start_time and (time.time() - self.start_time) >= self.duration:
            self.running = False
            self.node.get_logger().info(f'Recording complete! Total {self.sample_count} samples / 录制完成! 共 {self.sample_count} 个样本')

    def run(self):
        """@brief Main loop / 主循环"""
        try:
            while self.running and self.rclpy.ok():
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.node.get_logger().info(f'User interrupted, saving... / 用户中断，正在保存...')
        finally:
            self.csv_file.close()
            elapsed = time.time() - self.start_time if self.start_time else 0
            print(f'\nRecording finished / 录制结束:')
            print(f'  Total samples / 总样本数:   {self.sample_count}')
            print(f'  Duration / 录制时长:   {elapsed:.1f} s / 秒 ({elapsed/3600:.2f} h / 小时)')
            if elapsed > 0:
                print(f'  Avg sample rate / 平均采样率: {self.sample_count / elapsed:.1f} Hz')
            print(f'  Saved to / 文件保存至: {self.output_file}')
            print(f'  File size / 文件大小:   {os.path.getsize(self.output_file) / 1024 / 1024:.1f} MB')

            self.node.destroy_node()
            try:
                self.rclpy.shutdown()
            except Exception:
                pass  # Ctrl+C may have already triggered shutdown / Ctrl+C 可能已触发 shutdown


# ============================================================================
#  Analysis module (no ROS2 dependency) / 分析模块 (不依赖 ROS2)
# ============================================================================

def load_imu_data(csv_path):
    """
    @brief Load IMU data from CSV file / 从 CSV 文件加载 IMU 数据
    @param csv_path  CSV file path / CSV 文件路径
    @return (timestamps, gyro_data, accel_data, accel_unit, sample_rate)
            timestamps: timestamp array in seconds / 秒为单位的时间戳数组
            gyro_data:  Nx3 gyroscope data (rad/s) / Nx3 陀螺仪数据 (rad/s)
            accel_data: Nx3 accelerometer data (m/s²) / Nx3 加速度计数据 (m/s²)
            accel_unit: original acceleration unit / 原始加速度单位
            sample_rate: sample rate (Hz) / 采样率 (Hz)
    """
    import numpy as np

    timestamps = []
    gyro = []
    accel = []
    accel_unit = 'g'  # default / 默认

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # skip header / 跳过表头

        for row in reader:
            t = float(row[0]) + float(row[1]) * 1e-9
            timestamps.append(t)
            gyro.append([float(row[2]), float(row[3]), float(row[4])])
            accel.append([float(row[5]), float(row[6]), float(row[7])])
            if len(row) > 8:
                accel_unit = row[8]

    timestamps = np.array(timestamps)
    gyro_data = np.array(gyro)
    accel_data = np.array(accel)

    # Zero-base timestamps / 将时间戳归零
    timestamps -= timestamps[0]

    # Convert from g to m/s² if needed / 如果加速度单位是 g，转换为 m/s²
    if accel_unit.strip().lower() == 'g':
        print(f'[Info] Detected acceleration unit as g, converting to m/s² (×9.80665)')
        print(f'[信息] 检测到加速度单位为 g，正在转换为 m/s² (×9.80665)')
        accel_data *= 9.80665

    # Calculate actual sample rate / 计算实际采样率
    dt_array = np.diff(timestamps)
    sample_rate = 1.0 / np.median(dt_array)

    return timestamps, gyro_data, accel_data, accel_unit, sample_rate


def compute_allan_variance(data, dt, num_clusters=200):
    """
    @brief Compute Overlapping Allan Variance / 计算重叠 Allan 方差
    @param data         1D array, sensor rate data (e.g. rad/s or m/s²) / 1D 数组，传感器速率数据 (如 rad/s 或 m/s²)
    @param dt           Sampling interval in seconds / 采样间隔 (秒)
    @param num_clusters Number of clusters (log-spaced) / 聚类数量 (对数间隔)
    @return (taus, adevs)  Averaging time array and Allan deviation array / 平均时间数组和 Allan 偏差数组
    """
    import numpy as np

    N = len(data)

    # Cumulative integration / 累积积分: θ[n] = Σ data[i] * dt
    theta = np.cumsum(data) * dt

    # Prepend zero / 在前面补零: θ[0] = 0
    theta = np.insert(theta, 0, 0.0)

    # Cluster sizes: log-spaced from 1 to N/2 / 聚类大小: 对数间隔从 1 到 N/2
    max_m = N // 2
    if max_m < 2:
        raise ValueError(f"Too few samples ({N}), cannot compute Allan variance / 数据量太少 ({N} 样本)，无法计算 Allan 方差")

    m_values = np.unique(
        np.logspace(0, np.log10(max_m), num=num_clusters).astype(int)
    )
    m_values = m_values[m_values >= 1]
    m_values = m_values[m_values <= max_m]

    taus = []
    adevs = []

    for m in m_values:
        tau = m * dt
        # Overlapping Allan variance formula / 重叠 Allan 方差公式:
        # σ²(τ) = 1/(2τ²(N-2m+1)) * Σ_{k=0}^{N-2m} (θ[k+2m] - 2θ[k+m] + θ[k])²
        n_theta = len(theta)  # N+1
        if 2 * m >= n_theta:
            break

        # Vectorized computation / 向量化计算
        d = theta[2 * m:] - 2 * theta[m:n_theta - m] + theta[:n_theta - 2 * m]
        avar = np.mean(d ** 2) / (2.0 * tau ** 2)
        adev = np.sqrt(avar)

        taus.append(tau)
        adevs.append(adev)

    return np.array(taus), np.array(adevs)


def extract_noise_params(taus, adevs, sensor_name=""):
    """
    @brief Extract noise parameters from Allan deviation curve / 从 Allan 偏差曲线提取噪声参数
    @param taus        Averaging time array / 平均时间数组
    @param adevs       Allan deviation array / Allan 偏差数组
    @param sensor_name Sensor name (for printing) / 传感器名称 (用于打印)
    @return dict containing noise_density, bias_instability, random_walk / 包含 noise_density, bias_instability, random_walk
    """
    import numpy as np

    log_taus = np.log10(taus)
    log_adevs = np.log10(adevs)

    # --- 1. White noise density N (slope = -1/2) / 白噪声密度 N (slope = -1/2) ---
    # Fit slope = -1/2 line in short τ region / 在短 τ 区域拟合 slope = -1/2 的直线
    # N = ADEV(τ) * sqrt(τ) in white noise dominant region / N = ADEV(τ) * sqrt(τ) 在白噪声主导区
    # Select τ < 1s region for fitting / 选择 τ < 1s 的区域进行拟合
    short_mask = taus < 1.0
    if np.sum(short_mask) < 3:
        # If too few points with τ<1s, use first 20% / 如果 τ<1s 的点太少，用前 20% 的点
        n_short = max(3, len(taus) // 5)
        short_mask = np.zeros(len(taus), dtype=bool)
        short_mask[:n_short] = True

    # Constrained fit with slope=-1/2 / 用 slope=-1/2 约束拟合: log(ADEV) = -0.5*log(τ) + log(N)
    # => log(N) = log(ADEV) + 0.5*log(τ)
    log_N_estimates = log_adevs[short_mask] + 0.5 * log_taus[short_mask]
    log_N = np.median(log_N_estimates)
    N = 10 ** log_N

    # --- 2. Bias instability B (ADEV minimum) / 零偏不稳定性 B (ADEV 最小值) ---
    min_idx = np.argmin(adevs)
    B_raw = adevs[min_idx]
    tau_B = taus[min_idx]
    # B = min(ADEV) / sqrt(2*ln2/π) ≈ min(ADEV) / 0.6648
    B = B_raw / 0.6648

    # --- 3. Random walk K (slope = +1/2) / 随机游走 K (slope = +1/2) ---
    # Fit slope = +1/2 line in long τ region / 在长 τ 区域拟合 slope = +1/2 的直线
    # σ(τ) = K * sqrt(τ/3) => K = ADEV * sqrt(3/τ)
    long_mask = taus > tau_B
    if np.sum(long_mask) < 3:
        n_long = max(3, len(taus) // 5)
        long_mask = np.zeros(len(taus), dtype=bool)
        long_mask[-n_long:] = True

    # Check if long τ region has rising trend (slope > 0) / 检查长 τ 区域是否确实有上升趋势 (slope > 0)
    if np.sum(long_mask) >= 3:
        local_slopes = np.diff(log_adevs[long_mask]) / np.diff(log_taus[long_mask])
        positive_slope_mask = np.where(long_mask)[0]

        # Only use regions with slope > 0 / 只用斜率 > 0 的区域
        rising_mask = long_mask.copy()
        if len(local_slopes) > 0 and np.any(local_slopes > 0):
            # Constrained fit with slope = +1/2 / 用 slope = +1/2 约束拟合: log(ADEV) = 0.5*log(τ) + log(K/sqrt(3))
            # => log(K) = log(ADEV) - 0.5*log(τ) + 0.5*log(3)
            log_K_estimates = (log_adevs[rising_mask] - 0.5 * log_taus[rising_mask]
                               + 0.5 * np.log10(3))
            log_K = np.median(log_K_estimates)
            K = 10 ** log_K
        else:
            # No clear random walk region, estimate from longest τ / 没有明确的随机游走区域，使用最长 τ 处的估计
            K = adevs[-1] * np.sqrt(3.0 / taus[-1])
    else:
        K = adevs[-1] * np.sqrt(3.0 / taus[-1])

    # Generate fit lines for plotting / 生成拟合线用于绘图
    fit_lines = {
        'white_noise': {
            'taus': taus,
            'adevs': N / np.sqrt(taus),
            'label': f'White Noise N={N:.2e}'
        },
        'bias_instability': {
            'tau': tau_B,
            'adev': B_raw,
            'label': f'Bias Instability B={B:.2e} (t={tau_B:.1f}s)'
        },
        'random_walk': {
            'taus': taus,
            'adevs': K * np.sqrt(taus / 3.0),
            'label': f'Random Walk K={K:.2e}'
        }
    }

    return {
        'noise_density': N,
        'bias_instability': B,
        'random_walk': K,
        'fit_lines': fit_lines
    }


def plot_allan_variance(results, output_dir):
    """
    @brief Plot Allan deviation chart / 绘制 Allan 偏差图
    @param results    Analysis results dict / 分析结果字典
    @param output_dir Output directory / 输出目录
    """
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')  # Non-GUI backend / 无 GUI 后端
    import matplotlib.pyplot as plt

    plt.rcParams['font.size'] = 11
    plt.rcParams['figure.figsize'] = (14, 10)

    fig, axes = plt.subplots(2, 1, figsize=(14, 12))

    # --- Gyroscope / 陀螺仪 ---
    ax = axes[0]
    colors = ['#e74c3c', '#2ecc71', '#3498db']
    axis_names = ['X', 'Y', 'Z']

    for i in range(3):
        key = f'gyro_{axis_names[i].lower()}'
        r = results[key]
        ax.loglog(r['taus'], r['adevs'], color=colors[i], alpha=0.7,
                  label=f'Gyro {axis_names[i]}', linewidth=1.5)

    # Plot fit lines for average / 绘制平均值的拟合线
    r_avg = results['gyro_avg']
    ax.loglog(r_avg['taus'], r_avg['adevs'], 'k-', linewidth=2.5,
              label='Gyro Average', alpha=0.9)

    fl = r_avg['params']['fit_lines']
    ax.loglog(fl['white_noise']['taus'], fl['white_noise']['adevs'],
              '--', color='orange', linewidth=1.5, label=fl['white_noise']['label'])
    ax.loglog(fl['random_walk']['taus'], fl['random_walk']['adevs'],
              '--', color='purple', linewidth=1.5, label=fl['random_walk']['label'])
    ax.axhline(y=fl['bias_instability']['adev'], color='gray',
               linestyle=':', linewidth=1, label=fl['bias_instability']['label'])

    ax.set_xlabel('Averaging Time τ (s)')
    ax.set_ylabel('Allan Deviation (rad/s)')
    ax.set_title('Gyroscope Allan Deviation')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, which='both', alpha=0.3)

    # --- Accelerometer / 加速度计 ---
    ax = axes[1]

    for i in range(3):
        key = f'accel_{axis_names[i].lower()}'
        r = results[key]
        ax.loglog(r['taus'], r['adevs'], color=colors[i], alpha=0.7,
                  label=f'Accel {axis_names[i]}', linewidth=1.5)

    r_avg = results['accel_avg']
    ax.loglog(r_avg['taus'], r_avg['adevs'], 'k-', linewidth=2.5,
              label='Accel Average', alpha=0.9)

    fl = r_avg['params']['fit_lines']
    ax.loglog(fl['white_noise']['taus'], fl['white_noise']['adevs'],
              '--', color='orange', linewidth=1.5, label=fl['white_noise']['label'])
    ax.loglog(fl['random_walk']['taus'], fl['random_walk']['adevs'],
              '--', color='purple', linewidth=1.5, label=fl['random_walk']['label'])
    ax.axhline(y=fl['bias_instability']['adev'], color='gray',
               linestyle=':', linewidth=1, label=fl['bias_instability']['label'])

    ax.set_xlabel('Averaging Time τ (s)')
    ax.set_ylabel('Allan Deviation (m/s²)')
    ax.set_title('Accelerometer Allan Deviation')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, which='both', alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(output_dir, 'allan_variance_plot.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'\n[Info] Allan variance plot saved to / [信息] Allan 方差图已保存至: {plot_path}')

    return plot_path


def analyze_imu_data(csv_path, output_dir, update_yaml=False, yaml_path=None):
    """
    @brief Perform Allan variance analysis / 执行 Allan 方差分析
    @param csv_path    Input CSV file path / 输入 CSV 文件路径
    @param output_dir  Output directory / 输出目录
    @param update_yaml Whether to auto-update YAML config file / 是否自动更新 YAML 配置文件
    @param yaml_path   YAML config file path / YAML 配置文件路径
    """
    import numpy as np

    os.makedirs(output_dir, exist_ok=True)

    print('=' * 70)
    print('  IMU Allan Variance Noise Analysis / IMU Allan 方差噪声分析')
    print('=' * 70)

    # Load data / 加载数据
    print(f'\n[Step 1/4 / 步骤 1/4] Loading data / 加载数据: {csv_path}')
    timestamps, gyro_data, accel_data, accel_unit, sample_rate = load_imu_data(csv_path)

    N = len(timestamps)
    duration = timestamps[-1] - timestamps[0]
    dt = 1.0 / sample_rate

    print(f'  Samples / 样本数量:   {N}')
    print(f'  Duration / 录制时长:   {duration:.1f} s / 秒 ({duration/3600:.2f} h / 小时)')
    print(f'  Sample rate / 采样率:     {sample_rate:.1f} Hz')
    print(f'  Sample interval / 采样间隔:   {dt*1000:.3f} ms')

    # Data quality check / 数据质量检查
    dt_array = np.diff(timestamps)
    dt_std = np.std(dt_array)
    dt_max_gap = np.max(dt_array)
    print(f'  dt std / 时间间隔标准差: {dt_std*1000:.3f} ms')
    print(f'  max dt gap / 最大时间间隔:   {dt_max_gap*1000:.3f} ms')

    if dt_max_gap > 10 * dt:
        gap_count = np.sum(dt_array > 5 * dt)
        print(f'  Warning: {gap_count} large gaps (>{5*dt*1000:.1f}ms) may affect accuracy / '
              f'⚠ 警告: 发现 {gap_count} 个大间隔 (>{5*dt*1000:.1f}ms)，可能影响分析精度')

    if duration < 3600:
        print(f'  Warning: duration < 1h, random-walk estimate may be unreliable / '
              f'⚠ 警告: 录制时长不足 1 小时，随机游走参数可能不准确')
        print(f'         Recommend >= 2h recording / 建议至少录制 2 小时以获得可靠结果')

    # Print data statistics / 打印数据统计
    print(f'\n  Gyro mean (rad/s) / 陀螺仪均值 (rad/s):')
    for i, ax in enumerate(['X', 'Y', 'Z']):
        print(f'    {ax}: {np.mean(gyro_data[:, i]):.6f} ± {np.std(gyro_data[:, i]):.6f}')
    print(f'  Accel mean (m/s²) / 加速度计均值 (m/s²):')
    for i, ax in enumerate(['X', 'Y', 'Z']):
        print(f'    {ax}: {np.mean(accel_data[:, i]):.4f} ± {np.std(accel_data[:, i]):.4f}')

    # Compute Allan variance / 计算 Allan 方差
    print(f'\n[Step 2/4 / 步骤 2/4] Computing Allan variance / 计算 Allan 方差...')
    results = {}

    axis_names = ['x', 'y', 'z']

    # Per gyro axis / 陀螺仪各轴
    for i in range(3):
        print(f'  Gyro {axis_names[i].upper()} axis / 计算 Gyro {axis_names[i].upper()} 轴...')
        taus, adevs = compute_allan_variance(gyro_data[:, i], dt)
        params = extract_noise_params(taus, adevs, f'Gyro {axis_names[i].upper()}')
        results[f'gyro_{axis_names[i]}'] = {
            'taus': taus, 'adevs': adevs, 'params': params
        }

    # Per accelerometer axis / 加速度计各轴
    for i in range(3):
        print(f'  Accel {axis_names[i].upper()} axis / 计算 Accel {axis_names[i].upper()} 轴...')
        taus, adevs = compute_allan_variance(accel_data[:, i], dt)
        params = extract_noise_params(taus, adevs, f'Accel {axis_names[i].upper()}')
        results[f'accel_{axis_names[i]}'] = {
            'taus': taus, 'adevs': adevs, 'params': params
        }

    # Average Allan deviation across axes / 计算各轴平均 Allan 偏差
    print(f'  Averaging three axes / 计算三轴平均值...')

    # Gyro average / 陀螺仪平均
    gyro_adevs_list = [results[f'gyro_{ax}']['adevs'] for ax in axis_names]
    min_len = min(len(a) for a in gyro_adevs_list)
    gyro_avg_adevs = np.mean([a[:min_len] for a in gyro_adevs_list], axis=0)
    gyro_avg_taus = results['gyro_x']['taus'][:min_len]
    gyro_avg_params = extract_noise_params(gyro_avg_taus, gyro_avg_adevs, 'Gyro Avg')
    results['gyro_avg'] = {
        'taus': gyro_avg_taus, 'adevs': gyro_avg_adevs, 'params': gyro_avg_params
    }

    # Accelerometer average / 加速度计平均
    accel_adevs_list = [results[f'accel_{ax}']['adevs'] for ax in axis_names]
    min_len = min(len(a) for a in accel_adevs_list)
    accel_avg_adevs = np.mean([a[:min_len] for a in accel_adevs_list], axis=0)
    accel_avg_taus = results['accel_x']['taus'][:min_len]
    accel_avg_params = extract_noise_params(accel_avg_taus, accel_avg_adevs, 'Accel Avg')
    results['accel_avg'] = {
        'taus': accel_avg_taus, 'adevs': accel_avg_adevs, 'params': accel_avg_params
    }

    # Plot / 绘图
    print(f'\n[Step 3/4 / 步骤 3/4] Generating Allan variance plot / 生成 Allan 方差图...')
    plot_path = plot_allan_variance(results, output_dir)

    # Output summary / 输出结果
    print(f'\n[Step 4/4 / 步骤 4/4] Analysis summary / 分析结果汇总')
    print('=' * 70)

    print(f'\n  ▶ Gyroscope noise (3-axis average) / 陀螺仪噪声参数 (三轴平均):')
    gp = gyro_avg_params
    print(f'    ┌─────────────────────────────────────────────────────┐')
    print(f'    │ White noise density (N) / 白噪声密度 (N):    {gp["noise_density"]:.6e}  rad/s/√Hz   │')
    print(f'    │ Bias instability (B) / 零偏不稳定性 (B):  {gp["bias_instability"]:.6e}  rad/s       │')
    print(f'    │ Random walk (K) / 随机游走 (K):      {gp["random_walk"]:.6e}  rad/s²/√Hz  │')
    print(f'    └─────────────────────────────────────────────────────┘')

    print(f'\n  ▶ Accelerometer noise (3-axis average) / 加速度计噪声参数 (三轴平均):')
    ap = accel_avg_params
    print(f'    ┌─────────────────────────────────────────────────────┐')
    print(f'    │ White noise density (N) / 白噪声密度 (N):    {ap["noise_density"]:.6e}  m/s²/√Hz    │')
    print(f'    │ Bias instability (B) / 零偏不稳定性 (B):  {ap["bias_instability"]:.6e}  m/s²        │')
    print(f'    │ Random walk (K) / 随机游走 (K):      {ap["random_walk"]:.6e}  m/s³/√Hz    │')
    print(f'    └─────────────────────────────────────────────────────┘')

    print(f'\n  ▶ Per-axis detail / 各轴详细数据:')
    print(f'    {"Sensor / 传感器":<12} {"N / 白噪声 N":<16} {"B / 零偏不稳定":<16} {"K / 随机游走":<16}')
    print(f'    {"─"*60}')
    for i, ax in enumerate(['X', 'Y', 'Z']):
        gp_i = results[f'gyro_{ax.lower()}']['params']
        print(f'    Gyro {ax:<5} '
              f'{gp_i["noise_density"]:<16.6e} '
              f'{gp_i["bias_instability"]:<16.6e} '
              f'{gp_i["random_walk"]:<16.6e}')
    for i, ax in enumerate(['X', 'Y', 'Z']):
        ap_i = results[f'accel_{ax.lower()}']['params']
        print(f'    Accel {ax:<4} '
              f'{ap_i["noise_density"]:<16.6e} '
              f'{ap_i["bias_instability"]:<16.6e} '
              f'{ap_i["random_walk"]:<16.6e}')

    # OpenVINS config hints / OpenVINS 配置建议
    gyro_N = gyro_avg_params['noise_density']
    gyro_K = gyro_avg_params['random_walk']
    accel_N = accel_avg_params['noise_density']
    accel_K = accel_avg_params['random_walk']

    print(f'\n  ▶ OpenVINS kalibr_imu_chain.yaml recommended / 推荐参数:')
    print(f'    (×10 safety factor, for online VIO / 已乘以安全系数 ×10，适合 VIO 在线使用)')
    print(f'    ┌──────────────────────────────────────────────────────────────┐')
    print(f'    │ gyroscope_noise_density:     {gyro_N * 10:.6e}  # rad/s/√Hz   │')
    print(f'    │ gyroscope_random_walk:       {gyro_K * 10:.6e}  # rad/s²/√Hz  │')
    print(f'    │ accelerometer_noise_density: {accel_N * 10:.6e}  # m/s²/√Hz   │')
    print(f'    │ accelerometer_random_walk:   {accel_K * 10:.6e}  # m/s³/√Hz   │')
    print(f'    └──────────────────────────────────────────────────────────────┘')
    print(f'    Note: ×10 is OpenVINS practice for robustness in motion / 注: 安全系数 ×10 是 OpenVINS 推荐做法，')
    print(f'          helps robustness in real motion / 有助于系统在实际运动中保持鲁棒性。')

    # Save results to text file / 保存结果到文本文件
    result_path = os.path.join(output_dir, 'imu_noise_params.txt')
    with open(result_path, 'w') as f:
        f.write('IMU Allan Variance Analysis Results\n')
        f.write('=' * 60 + '\n\n')
        f.write(f'Data file:    {csv_path}\n')
        f.write(f'Samples:      {N}\n')
        f.write(f'Duration:     {duration:.1f} s ({duration/3600:.2f} h)\n')
        f.write(f'Sample rate:  {sample_rate:.1f} Hz\n\n')

        f.write('Raw Allan Variance Parameters:\n')
        f.write('-' * 60 + '\n')
        f.write(f'gyroscope_noise_density:     {gyro_N:.6e}  # rad/s/sqrt(Hz)\n')
        f.write(f'gyroscope_random_walk:       {gyro_K:.6e}  # rad/s^2/sqrt(Hz)\n')
        f.write(f'accelerometer_noise_density: {accel_N:.6e}  # m/s^2/sqrt(Hz)\n')
        f.write(f'accelerometer_random_walk:   {accel_K:.6e}  # m/s^3/sqrt(Hz)\n\n')

        f.write('OpenVINS Recommended (x10 safety factor):\n')
        f.write('-' * 60 + '\n')
        f.write(f'gyroscope_noise_density:     {gyro_N * 10:.6e}\n')
        f.write(f'gyroscope_random_walk:       {gyro_K * 10:.6e}\n')
        f.write(f'accelerometer_noise_density: {accel_N * 10:.6e}\n')
        f.write(f'accelerometer_random_walk:   {accel_K * 10:.6e}\n')

    print(f'\n[Info] Analysis saved to / [信息] 分析结果已保存至: {result_path}')

    # Optional: update YAML / 可选: 更新 YAML 文件
    if update_yaml and yaml_path:
        update_imu_yaml(yaml_path, gyro_N * 10, gyro_K * 10, accel_N * 10, accel_K * 10)

    print(f'\n{"=" * 70}')
    print(f'  Done / 分析完成!')
    print(f'  Output dir / 输出目录: {output_dir}')
    print(f'{"=" * 70}')

    return results


def update_imu_yaml(yaml_path, gyro_nd, gyro_rw, accel_nd, accel_rw):
    """
    @brief Update noise parameters in kalibr_imu_chain.yaml / 更新 kalibr_imu_chain.yaml 中的噪声参数
    @param yaml_path  YAML file path / YAML 文件路径
    @param gyro_nd    Gyro white noise density / 陀螺仪白噪声密度
    @param gyro_rw    Gyro random walk / 陀螺仪随机游走
    @param accel_nd   Accel white noise density / 加速度计白噪声密度
    @param accel_rw   Accel random walk / 加速度计随机游走
    """
    if not os.path.exists(yaml_path):
        print(f'[Error] YAML not found / [错误] YAML 文件不存在: {yaml_path}')
        return

    with open(yaml_path, 'r') as f:
        content = f.read()

    import re

    # Replace each parameter / 替换各个参数
    replacements = {
        'accelerometer_noise_density': f'{accel_nd:.6e}',
        'accelerometer_random_walk': f'{accel_rw:.6e}',
        'gyroscope_noise_density': f'{gyro_nd:.6e}',
        'gyroscope_random_walk': f'{gyro_rw:.6e}',
    }

    for key, value in replacements.items():
        pattern = rf'({key}\s*:\s*)[^\s#]+'
        content = re.sub(pattern, rf'\g<1>{value}', content)

    # Backup original file / 备份原文件
    backup_path = yaml_path + '.bak'
    if not os.path.exists(backup_path):
        import shutil
        shutil.copy2(yaml_path, backup_path)
        print(f'[Info] Backup saved / [信息] 已备份原文件至: {backup_path}')

    with open(yaml_path, 'w') as f:
        f.write(content)

    print(f'[Info] Updated YAML / [信息] 已更新 YAML 文件: {yaml_path}')
    print(f'  gyroscope_noise_density:     {gyro_nd:.6e}')
    print(f'  gyroscope_random_walk:       {gyro_rw:.6e}')
    print(f'  accelerometer_noise_density: {accel_nd:.6e}')
    print(f'  accelerometer_random_walk:   {accel_rw:.6e}')


# ============================================================================
#  Main / 主函数
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='IMU noise Allan variance tool / IMU 噪声参数 Allan 方差分析工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples / 示例:
  # Record 2h IMU static / 录制 2 小时 IMU 静态数据
  python3 imu_allan_variance.py record --duration 7200

  # 30min quick test / 录制 30 分钟快速测试
  python3 imu_allan_variance.py record --duration 1800

  # Analyze recorded data / 分析录制的数据
  python3 imu_allan_variance.py analyze --input imu_static_data.csv

  # Analyze and update YAML / 分析并自动更新 YAML 配置
  python3 imu_allan_variance.py analyze --input imu_static_data.csv --update-yaml
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='Subcommand / 子命令')

    # --- record subcommand / record 子命令 ---
    record_parser = subparsers.add_parser('record', help='Record IMU static data / 录制 IMU 静态数据')
    record_parser.add_argument(
        '--topic', type=str, default='/imu/data',
        help='IMU topic (default /imu/data; D455: /camera/camera/imu) / IMU 话题名称'
    )
    record_parser.add_argument(
        '--d455', action='store_true',
        help='D455 preset (topic /camera/camera/imu, unit m/s2) / 快速设置 D455 参数'
    )
    record_parser.add_argument(
        '--duration', type=int, default=7200,
        help='Duration in seconds (default 7200 = 2h) / 录制时长，单位秒'
    )
    record_parser.add_argument(
        '--output', type=str, default='imu_static_data.csv',
        help='Output CSV path / 输出 CSV 文件路径'
    )
    record_parser.add_argument(
        '--accel-unit', type=str, default='g', choices=['g', 'm/s2'],
        help='Accel unit from driver (default g; D455 m/s2) / IMU 驱动发布的加速度单位'
    )

    # --- analyze subcommand / analyze 子命令 ---
    analyze_parser = subparsers.add_parser('analyze', help='Analyze IMU noise / 分析 IMU 噪声参数')
    analyze_parser.add_argument(
        '--input', type=str, default='imu_static_data.csv',
        help='Input CSV path / 输入 CSV 文件路径'
    )
    analyze_parser.add_argument(
        '--output-dir', type=str, default='imu_noise_results',
        help='Output directory / 输出目录'
    )
    analyze_parser.add_argument(
        '--update-yaml', action='store_true',
        help='Auto-update kalibr_imu_chain.yaml / 自动更新 kalibr_imu_chain.yaml 文件'
    )
    analyze_parser.add_argument(
        '--yaml-path', type=str,
        default=None,
        help='kalibr_imu_chain.yaml path (default: same dir) / YAML 路径'
    )

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == 'record':
        topic = args.topic
        accel_unit = args.accel_unit
        if args.d455:
            topic = '/camera/camera/imu'
            accel_unit = 'm/s2'
            print(f'[Info] D455 mode / [信息] 已设置为 D455 模式: topic={topic}, unit={accel_unit}')
        recorder = IMURecorder(topic, args.output, args.duration, accel_unit)
        recorder.run()

    elif args.command == 'analyze':
        if not os.path.exists(args.input):
            print(f'[Error] CSV not found / [错误] CSV 文件不存在: {args.input}')
            sys.exit(1)

        yaml_path = args.yaml_path
        if yaml_path is None and args.update_yaml:
            # Default: kalibr_imu_chain.yaml in script dir / 默认使用同目录下的 kalibr_imu_chain.yaml
            script_dir = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(script_dir, 'kalibr_imu_chain.yaml')

        analyze_imu_data(
            csv_path=args.input,
            output_dir=args.output_dir,
            update_yaml=args.update_yaml,
            yaml_path=yaml_path
        )


if __name__ == '__main__':
    main()
