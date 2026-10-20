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
mkdir -p vendor && ln -sfn /root/autodl-tmp/sensors/src/sensors vendor/sensors
export PYTHONPATH="src:vendor:${PYTHONPATH}"

pip install -r requirements.txt
python -m rs_calib
```

## 许可证

内部项目；与 sensors 部署策略一致。
