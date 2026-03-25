#!/usr/bin/env python3
"""
Generate a chessboard image for camera calibration (A4 print layout)
生成用于相机标定的棋盘格图片 (适配 A4 纸打印)

Default: 10x7 squares (9x6 inner corners), 25 mm per square,
with margins and caption, saved as high-resolution PNG.
默认生成 10x7 方格 (9x6 内角点)，每格 25mm，
带有边距和标注信息，保存为高分辨率 PNG。

Usage / 用法:
  python3 generate_chessboard.py
  Output / 生成的文件: chessboard_9x6_25mm.png
"""

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Pillow required: pip3 install Pillow / 需要安装 Pillow: pip3 install Pillow")
    exit(1)

# ============================================================
# Parameters / 参数配置
# ============================================================

COLS = 7           # Square columns along A4 width / 方格列数 — 沿 A4 宽度方向 (7x25=175mm < 210mm)
ROWS = 10          # Square rows along A4 height / 方格行数 — 沿 A4 高度方向 (10x25=250mm < 297mm)
                   # Inner corners 6x9; calibrate_camera uses --cols 6 --rows 9 / 内角点: 6x9，标定脚本中 --cols 6 --rows 9
SQUARE_MM = 25.0   # Square side length (mm) / 每格边长 (mm)
DPI = 300          # Print resolution / 打印分辨率

# A4 size (mm) / A4 纸尺寸 (mm)
A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0

# ============================================================
# Pixel dimensions / 计算像素尺寸
# ============================================================

MM_TO_PX = DPI / 25.4  # 1 mm = DPI/25.4 px / 1mm = DPI/25.4 像素

a4_w_px = int(A4_WIDTH_MM * MM_TO_PX)
a4_h_px = int(A4_HEIGHT_MM * MM_TO_PX)
sq_px = int(SQUARE_MM * MM_TO_PX)

board_w_px = COLS * sq_px
board_h_px = ROWS * sq_px

# Centering offset / 居中偏移
offset_x = (a4_w_px - board_w_px) // 2
offset_y = (a4_h_px - board_h_px) // 2

# ============================================================
# Draw / 绘制
# ============================================================

img = Image.new("RGB", (a4_w_px, a4_h_px), "white")
draw = ImageDraw.Draw(img)

# Draw checkerboard / 画棋盘格
for row in range(ROWS):
    for col in range(COLS):
        x0 = offset_x + col * sq_px
        y0 = offset_y + row * sq_px
        x1 = x0 + sq_px
        y1 = y0 + sq_px
        if (row + col) % 2 == 0:
            draw.rectangle([x0, y0, x1, y1], fill="black")

# Outer border / 画外边框
draw.rectangle(
    [offset_x, offset_y, offset_x + board_w_px, offset_y + board_h_px],
    outline="black", width=2
)

# Bottom caption / 底部标注
label = f"Chessboard {COLS - 1}x{ROWS - 1} inner corners | square = {SQUARE_MM:.0f}mm | print at 100% scale on A4"
font_size_px = int(3.0 * MM_TO_PX)  # ~3 mm text height / 约 3mm 高的字
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", font_size_px)
except OSError:
    font = ImageFont.load_default()

text_y = offset_y + board_h_px + int(5 * MM_TO_PX)
draw.text((offset_x, text_y), label, fill="black", font=font)

# ============================================================
# Save / 保存
# ============================================================

filename = f"chessboard_{COLS - 1}x{ROWS - 1}_{SQUARE_MM:.0f}mm.png"
img.save(filename, dpi=(DPI, DPI))

print(f"Generated / 已生成: {filename}")
print(f"  Paper / 纸张: A4 ({A4_WIDTH_MM}x{A4_HEIGHT_MM} mm)")
print(f"  Grid / 方格: {COLS}x{ROWS} ({COLS - 1}x{ROWS - 1} inner corners / 内角点)")
print(f"  Square size / 格边长: {SQUARE_MM}mm")
print(f"  Resolution / 分辨率: {a4_w_px}x{a4_h_px} px @ {DPI} DPI")
print()
print("Print tip: use 100% scale (not 'fit to page'); verify 25 mm square size / 打印提示: 请使用 100% 缩放（不要 '适合页面'），确保格子实际边长为 25mm")
