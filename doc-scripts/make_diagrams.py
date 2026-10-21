#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate Chinese-labeled diagrams for hand-eye calibration docs."""
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
ASSETS.mkdir(exist_ok=True)

W, H = 1200, 720
BG = (250, 251, 253)
INK = (30, 35, 45)
ACCENT = (0, 102, 180)
ACCENT2 = (200, 90, 40)
GREEN = (40, 140, 90)
GRAY = (120, 128, 140)
LIGHT = (230, 236, 245)
BOARD = (40, 40, 40)


def font(size: int, bold: bool = False):
    candidates = [
        r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
    ]
    for p in candidates:
        if Path(p).exists():
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                continue
    return ImageFont.load_default()


def rounded_rect(draw, xy, radius, fill, outline=None, width=2):
    draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)


def arrow(draw, p1, p2, color=INK, width=3):
    draw.line([p1, p2], fill=color, width=width)
    x1, y1 = p1
    x2, y2 = p2
    import math

    ang = math.atan2(y2 - y1, x2 - x1)
    L = 14
    for da in (2.6, -2.6):
        ax = x2 - L * math.cos(ang + da)
        ay = y2 - L * math.sin(ang + da)
        draw.line([(x2, y2), (ax, ay)], fill=color, width=width)


def draw_robot_base(draw, cx, cy, label="机器人基座 Base"):
    rounded_rect(draw, (cx - 70, cy - 40, cx + 70, cy + 40), 12, (210, 215, 225), INK, 2)
    draw.text((cx, cy), label, fill=INK, font=font(18), anchor="mm")


def draw_arm(draw, points, color=ACCENT):
    for i in range(len(points) - 1):
        draw.line([points[i], points[i + 1]], fill=color, width=10)
        draw.ellipse(
            (points[i][0] - 8, points[i][1] - 8, points[i][0] + 8, points[i][1] + 8),
            fill=color,
        )
    ex, ey = points[-1]
    draw.ellipse((ex - 12, ey - 12, ex + 12, ey + 12), fill=ACCENT2, outline=INK, width=2)


def draw_camera(draw, cx, cy, label="相机", color=GREEN):
    rounded_rect(draw, (cx - 45, cy - 28, cx + 45, cy + 28), 8, color, INK, 2)
    draw.ellipse((cx - 14, cy - 14, cx + 14, cy + 14), fill=(240, 250, 245), outline=INK, width=2)
    draw.text((cx, cy + 42), label, fill=INK, font=font(18), anchor="mm")


def draw_board(draw, cx, cy, w=90, h=70, label="标定板"):
    # checkerboard look
    cols, rows = 6, 4
    cw, rh = w // cols, h // rows
    x0, y0 = cx - w // 2, cy - h // 2
    for r in range(rows):
        for c in range(cols):
            fill = (20, 20, 20) if (r + c) % 2 == 0 else (245, 245, 245)
            draw.rectangle((x0 + c * cw, y0 + r * rh, x0 + (c + 1) * cw, y0 + (r + 1) * rh), fill=fill)
    draw.rectangle((x0, y0, x0 + cols * cw, y0 + rows * rh), outline=INK, width=2)
    draw.text((cx, cy + h // 2 + 22), label, fill=INK, font=font(18), anchor="mm")


def make_eye_in_hand():
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 36), "Eye-in-Hand（眼在手上）", fill=INK, font=font(34, True), anchor="mm")
    d.text(
        (W // 2, 78),
        "腕部相机随末端运动 · 标定板固定在场景中 · 求解相机相对末端的外参 X",
        fill=GRAY,
        font=font(20),
        anchor="mm",
    )

    # table / workspace
    rounded_rect(d, (80, 160, 1120, 640), 20, LIGHT, (200, 208, 220), 2)

    draw_robot_base(d, 220, 520)
    arm = [(220, 480), (300, 360), (420, 280), (560, 250)]
    draw_arm(d, arm)
    d.text((360, 300), "机械臂", fill=ACCENT, font=font(18))

    # camera on end
    draw_camera(d, 620, 230, "RealSense 腕部相机\n(Eye-in-Hand)", GREEN)
    # FOV cone
    d.polygon([(575, 258), (820, 420), (900, 360), (665, 250)], fill=(180, 220, 200, ), outline=GREEN)
    # re-draw camera on top of cone tip area
    draw_camera(d, 620, 230, "RealSense 腕部相机", GREEN)

    draw_board(d, 880, 480, label="固定标定板 Target")

    # transform labels
    arrow(d, (560, 250), (575, 230), ACCENT2)
    d.text((500, 200), "X = T_cam^ee（待求）", fill=ACCENT2, font=font(20, True))

    arrow(d, (220, 470), (400, 280), ACCENT)
    d.text((250, 380), "A：末端相对基座运动", fill=ACCENT, font=font(18))

    arrow(d, (700, 280), (860, 430), GREEN)
    d.text((720, 340), "B：标定板相对相机运动", fill=GREEN, font=font(18))

    d.text(
        (W // 2, 680),
        "闭环：Base → EE → Camera → Target → Base   ⇒   A X = X B",
        fill=INK,
        font=font(22, True),
        anchor="mm",
    )
    img.save(ASSETS / "eye_in_hand.png")
    print("wrote eye_in_hand.png")


def make_eye_to_hand():
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 36), "Eye-to-Hand（眼在手外）", fill=INK, font=font(34, True), anchor="mm")
    d.text(
        (W // 2, 78),
        "场景相机固定安装 · 标定板装在末端 · 求解相机相对基座的外参",
        fill=GRAY,
        font=font(20),
        anchor="mm",
    )

    rounded_rect(d, (80, 160, 1120, 640), 20, LIGHT, (200, 208, 220), 2)

    # fixed scene camera (top-right)
    draw_camera(d, 980, 230, "RealSense 场景相机\n(Eye-to-Hand)", ACCENT)
    d.polygon([(935, 258), (620, 420), (700, 500), (1025, 258)], fill=(200, 220, 240), outline=ACCENT)
    draw_camera(d, 980, 230, "RealSense 场景相机", ACCENT)

    draw_robot_base(d, 240, 520)
    arm = [(240, 480), (340, 380), (480, 340), (620, 360)]
    draw_arm(d, arm)
    d.text((360, 330), "机械臂", fill=ACCENT, font=font(18))

    # board on end-effector
    draw_board(d, 700, 390, w=80, h=60, label="末端标定板")

    arrow(d, (980, 270), (760, 380), ACCENT)
    d.text((820, 300), "相机观测标定板", fill=ACCENT, font=font(18))

    arrow(d, (240, 470), (500, 360), ACCENT2)
    d.text((300, 400), "末端位姿（已知）", fill=ACCENT2, font=font(18))

    d.text((500, 220), "目标：T_cam^base（场景相机相对基座）", fill=ACCENT2, font=font(22, True))

    d.text(
        (W // 2, 680),
        "固定相机看“手上的板” · 同样化归为 AX = XB（定义略有不同）",
        fill=INK,
        font=font(22, True),
        anchor="mm",
    )
    img.save(ASSETS / "eye_to_hand.png")
    print("wrote eye_to_hand.png")


def make_compare():
    img = Image.new("RGB", (W, 560), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 36), "两种配置对比一览", fill=INK, font=font(32, True), anchor="mm")

    # left card
    rounded_rect(d, (60, 90, 570, 500), 16, (236, 245, 240), GREEN, 3)
    d.text((315, 130), "Eye-in-Hand", fill=GREEN, font=font(28, True), anchor="mm")
    d.text((315, 170), "腕部相机 / 手上眼", fill=INK, font=font(22), anchor="mm")
    lines_l = [
        "· 相机刚性安装在法兰/工具端",
        "· 标定板固定在工作台上",
        "· 求解：相机 → 末端 外参",
        "· 适合：近距抓取、随动观察",
        "· RealSense 示例：腕部 D405/D435",
    ]
    y = 220
    for t in lines_l:
        d.text((90, y), t, fill=INK, font=font(20))
        y += 42

    # right card
    rounded_rect(d, (630, 90, 1140, 500), 16, (236, 242, 250), ACCENT, 3)
    d.text((885, 130), "Eye-to-Hand", fill=ACCENT, font=font(28, True), anchor="mm")
    d.text((885, 170), "场景相机 / 手外眼", fill=INK, font=font(22), anchor="mm")
    lines_r = [
        "· 相机固定在支架/龙门/天花板",
        "· 标定板装在末端（或夹爪上）",
        "· 求解：相机 → 基座 外参",
        "· 适合：全局定位、多机器人协同",
        "· RealSense 示例：场景 D435/D455",
    ]
    y = 220
    for t in lines_r:
        d.text((660, y), t, fill=INK, font=font(20))
        y += 42

    img.save(ASSETS / "compare_configs.png")
    print("wrote compare_configs.png")


def make_frames_chain():
    img = Image.new("RGB", (W, 520), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 36), "坐标变换链与齐次矩阵", fill=INK, font=font(32, True), anchor="mm")

    boxes = [
        (140, 180, "Base\n基座"),
        (380, 180, "EE / Gripper\n末端"),
        (620, 180, "Camera\n相机"),
        (860, 180, "Target\n标定板"),
    ]
    for x, y, lab in boxes:
        rounded_rect(d, (x - 80, y - 50, x + 80, y + 50), 12, LIGHT, ACCENT, 2)
        d.text((x, y), lab, fill=INK, font=font(20), anchor="mm")

    for i in range(3):
        x1 = boxes[i][0] + 80
        x2 = boxes[i + 1][0] - 80
        arrow(d, (x1 + 8, 180), (x2 - 8, 180), ACCENT, 4)

    d.text((260, 120), "T_ee^base", fill=ACCENT, font=font(18), anchor="mm")
    d.text((500, 120), "X = T_cam^ee", fill=ACCENT2, font=font(18, True), anchor="mm")
    d.text((740, 120), "T_tgt^cam", fill=GREEN, font=font(18), anchor="mm")

    d.text(
        (W // 2, 300),
        "任意点从相机坐标系到基座：  p_base = T_ee^base · X · p_cam",
        fill=INK,
        font=font(22),
        anchor="mm",
    )

    # matrix box
    rounded_rect(d, (200, 350, 1000, 470), 12, (245, 247, 250), GRAY, 2)
    d.text(
        (600, 390),
        "齐次变换  T = [ R(3×3) | t(3×1) ;  0 0 0 | 1 ]",
        fill=INK,
        font=font(22),
        anchor="mm",
    )
    d.text(
        (600, 435),
        "R ∈ SO(3) 旋转 · t ∈ R³ 平移 · 用于统一旋转与平移",
        fill=GRAY,
        font=font(18),
        anchor="mm",
    )
    img.save(ASSETS / "transform_chain.png")
    print("wrote transform_chain.png")


def make_realsense_setup():
    img = Image.new("RGB", (W, 680), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 36), "RealSense 双相机标定场景示意", fill=INK, font=font(32, True), anchor="mm")
    d.text(
        (W // 2, 78),
        "腕部相机（Eye-in-Hand）+ 场景相机（Eye-to-Hand）可同时服务抓取与全局感知",
        fill=GRAY,
        font=font(18),
        anchor="mm",
    )

    rounded_rect(d, (70, 120, 1130, 620), 18, LIGHT, (200, 208, 220), 2)

    # scene camera stand
    d.rectangle((980, 200, 1000, 520), fill=(160, 170, 185))
    d.rectangle((940, 520, 1040, 545), fill=(120, 130, 145))
    draw_camera(d, 990, 170, "场景相机 D435/D455", ACCENT)

    draw_robot_base(d, 280, 520)
    arm = [(280, 480), (380, 360), (520, 300), (660, 280)]
    draw_arm(d, arm)
    draw_camera(d, 720, 250, "腕部相机 D405", GREEN)

    # table object
    rounded_rect(d, (780, 480, 900, 540), 8, (220, 180, 120), INK, 2)
    d.text((840, 560), "工件 / 抓取目标", fill=INK, font=font(18), anchor="mm")

    draw_board(d, 500, 500, w=70, h=50, label="标定板（可移动）")

    # legend
    rounded_rect(d, (100, 150, 420, 280), 10, (255, 255, 255), GRAY, 1)
    d.text((110, 165), "作用分工", fill=INK, font=font(20, True))
    d.text((110, 205), "· 场景相机：粗定位、避障、监控", fill=ACCENT, font=font(17))
    d.text((110, 240), "· 腕部相机：精定位、位姿估计", fill=GREEN, font=font(17))

    img.save(ASSETS / "realsense_dual_setup.png")
    print("wrote realsense_dual_setup.png")


def make_axxb():
    img = Image.new("RGB", (W, 500), BG)
    d = ImageDraw.Draw(img)
    d.text((W // 2, 40), "经典方程 AX = XB 的几何含义", fill=INK, font=font(30, True), anchor="mm")

    # two poses
    rounded_rect(d, (80, 110, 560, 420), 14, (240, 248, 244), GREEN, 2)
    d.text((320, 145), "位姿 i → 位姿 j（相对运动）", fill=GREEN, font=font(22, True), anchor="mm")
    d.text((120, 200), "机器人侧：A = (T_ee^base_i)⁻¹ · T_ee^base_j", fill=INK, font=font(18))
    d.text((120, 245), "（末端从 i 到 j 的相对变换）", fill=GRAY, font=font(16))
    d.text((120, 300), "相机侧：B = T_tgt^cam_i · (T_tgt^cam_j)⁻¹", fill=INK, font=font(18))
    d.text((120, 345), "（标定板在相机中从 i 到 j 的相对变换）", fill=GRAY, font=font(16))

    rounded_rect(d, (640, 110, 1120, 420), 14, (245, 242, 250), ACCENT2, 2)
    d.text((880, 145), "未知量 X（刚性外参）", fill=ACCENT2, font=font(22, True), anchor="mm")
    d.text((680, 210), "Eye-in-Hand：X = T_cam^ee", fill=INK, font=font(20))
    d.text((680, 260), "Eye-to-Hand：常取 X = T_tgt^ee", fill=INK, font=font(20))
    d.text((680, 310), "再合成得到 T_cam^base", fill=INK, font=font(20))
    d.text((880, 375), "多组 (A_k, B_k) 联立求解", fill=ACCENT, font=font(20, True), anchor="mm")

    img.save(ASSETS / "axxb_meaning.png")
    print("wrote axxb_meaning.png")


if __name__ == "__main__":
    make_eye_in_hand()
    make_eye_to_hand()
    make_compare()
    make_frames_chain()
    make_realsense_setup()
    make_axxb()
