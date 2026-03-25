"""
One-click launch: D455 + OpenVINS
D455 + OpenVINS 一键启动

RealSense D455 + built-in IMU visual-inertial odometry (monocular or stereo IR)
RealSense D455 相机 + 内置 IMU 视觉惯性里程计（单目或双目红外）

Usage / 用法:
  ros2 launch ov_msckf dexumi_d455.launch.py
  ros2 launch ov_msckf dexumi_d455.launch.py use_stereo:=true
  ros2 launch ov_msckf dexumi_d455.launch.py rviz_enable:=true
  # Disable IR dot projector, manual exposure 10000 / 禁用红外散斑，并设置手动曝光为 10000
  ros2 launch ov_msckf dexumi_d455.launch.py emitter_enabled:=0 enable_auto_exposure:=false infra_exposure:=10000
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ov_config_dir = get_package_share_directory("ov_msckf")
    rviz_config = os.path.join(ov_config_dir, "launch", "display_ros2.rviz")

    realsense_share = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(realsense_share, "launch", "rs_launch.py")

    return LaunchDescription([

        DeclareLaunchArgument(
            "rviz_enable", default_value="false",
            description="Enable RViz2 visualization / 启用 RViz2 可视化"
        ),
        DeclareLaunchArgument(
            "use_stereo", default_value="false",
            description="Stereo IR (infra1+infra2) for scale / 双目红外，尺度可观测性更好"
        ),
        DeclareLaunchArgument(
            "emitter_enabled", default_value="0",
            description="D455 IR projector: 0=Off, 1=On / D455 红外投射器开关"
        ),
        DeclareLaunchArgument(
            "enable_auto_exposure", default_value="false",
            description="Auto exposure for IR / 红外自动曝光"
        ),
        DeclareLaunchArgument(
            "infra_exposure", default_value="8500",
            description="Manual IR exposure if auto off / 关闭自动曝光时的手动曝光值"
        ),

        # RealSense D455: IR infra1(+infra2), IMU, sync / RealSense D455：红外 infra1(+infra2)、IMU、同步
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "enable_color": "false",
                "enable_depth": "false",
                "enable_infra1": "true",
                "enable_infra2": LaunchConfiguration("use_stereo"),
                "depth_module.infra_profile": "640,480,30",
                "depth_module.emitter_enabled": LaunchConfiguration("emitter_enabled"),
                "depth_module.enable_auto_exposure": LaunchConfiguration("enable_auto_exposure"),
                "depth_module.exposure": LaunchConfiguration("infra_exposure"),
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "2",
                "enable_sync": "true",
                "log_level": "info",
            }.items(),
        ),

        # OpenVINS VIO (stereo) / OpenVINS VIO（双目）
        Node(
            package="ov_msckf",
            executable="run_subscribe_msckf",
            name="run_subscribe_msckf",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("use_stereo")),
            parameters=[
                {"config_path": os.path.join(ov_config_dir, "config", "d455_realsense_stereo", "estimator_config.yaml")},
                {"verbosity": "INFO"},
                {"use_stereo": True},
                {"max_cameras": 2},
            ],
        ),
        # OpenVINS VIO (monocular) / OpenVINS VIO（单目）
        Node(
            package="ov_msckf",
            executable="run_subscribe_msckf",
            name="run_subscribe_msckf",
            output="screen",
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("use_stereo")),
            parameters=[
                {"config_path": os.path.join(ov_config_dir, "config", "d455_realsense", "estimator_config.yaml")},
                {"verbosity": "INFO"},
                {"use_stereo": False},
                {"max_cameras": 1},
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("rviz_enable")),
            arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        ),
    ])
