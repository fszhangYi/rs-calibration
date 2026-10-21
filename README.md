# rs-calibration

场景相机 Eye-to-Hand 外参标定上位机（Windows / PyQt，依赖 `sensors` SDK）。

## 文档

- [需求报告](docs/需求报告-场景相机eye-to-hand标定上位机.md)
- [概念设计](docs/概念设计-场景相机eye-to-hand标定上位机.md)
- [详细设计](docs/详细设计-场景相机eye-to-hand标定上位机.md)

## 依赖

- Python 3.10+
- `sensors`（软链或 editable 安装）
- PyQt6（或 PyQt5）、OpenCV、NumPy、SciPy、pyrealsense2
- Elite EC SDK（`elite` / elirobots，经 sensors）

## 快速开始

```bash
cd /root/autodl-tmp/rs-calibration
# 软链 sensors（示例）
mkdir -p vendor && ln -sfn /path/to/sensors/src/sensors vendor/sensors
export PYTHONPATH="src:vendor:${PYTHONPATH}"

pip install -r requirements.txt
# 实机另需: pip install pyrealsense2  以及 elite/elirobots（经 sensors）

# 桌面 PyQt
python -m rs_calib

# Web 前端（默认 0.0.0.0:6006，AutoDL 可映射该端口）
python -m rs_calib web
# 或: python -m rs_calib.web.server

# Windows 桌面包（Linux 主机经 Wine 交叉打包，Scheme A 适配）
bash scripts/build-desktop.sh --target windows
# 产物：release/rs-calib-desktop-windows-x64-<ts>/ 与同名 .zip
```

## 功能概览

- RealSense **单/多相机**扫描开流 / 内参回填 / 预览  
- EC616 IP 连接、Arm/点动（关节+笛卡尔）、**矩阵走位**自动网格采点、录制 session  

- **多相机录制**：`session/cameras/<serial>/video.mp4` + 共享 `joints_poses.jsonl`  
- **实时 HandEye 仅单相机**；多相机请离线按相机分别求解  
- 离线智能滤波选点；双轴验证  

详见 `docs/`。

## 许可证

内部项目；与 sensors 部署策略一致。
