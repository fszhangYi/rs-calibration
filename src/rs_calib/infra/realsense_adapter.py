from __future__ import annotations

import time
from typing import Optional

import numpy as np

from rs_calib.domain.types import DeviceInfo, Intrinsics
from rs_calib.errors import AppError


class RealSenseAdapter:
    """Single-camera RealSense wrapper (pyrealsense2)."""

    def __init__(self) -> None:
        self._pipeline = None
        self._profile = None
        self._serial: Optional[str] = None
        self._intrinsics: Optional[Intrinsics] = None
        self._width = 1280
        self._height = 720
        self._fps = 15

    @staticmethod
    def list_devices() -> list[DeviceInfo]:
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise AppError("CAM_SDK_MISSING", "未安装 pyrealsense2", str(e)) from e
        ctx = rs.context()
        out: list[DeviceInfo] = []
        for dev in ctx.query_devices():
            serial = str(dev.get_info(rs.camera_info.serial_number))
            name = str(dev.get_info(rs.camera_info.name)) if dev.supports(rs.camera_info.name) else ""
            fw = (
                str(dev.get_info(rs.camera_info.firmware_version))
                if dev.supports(rs.camera_info.firmware_version)
                else ""
            )
            out.append(DeviceInfo(serial=serial, name=name, firmware=fw))
        return out

    @property
    def is_open(self) -> bool:
        return self._pipeline is not None

    @property
    def serial(self) -> Optional[str]:
        return self._serial

    def open(self, serial: str, width: int = 1280, height: int = 720, fps: int = 15) -> Intrinsics:
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise AppError("CAM_SDK_MISSING", "未安装 pyrealsense2", str(e)) from e

        self.close()
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = pipeline.start(cfg)
        except Exception as e:  # noqa: BLE001
            raise AppError("CAM_OPEN_FAIL", f"打开相机失败: {serial}", str(e)) from e

        self._pipeline = pipeline
        self._profile = profile
        self._serial = serial
        self._width, self._height, self._fps = width, height, fps
        self._intrinsics = self._read_intrinsics(rs, profile, width, height, fps)
        return self._intrinsics

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
        self._pipeline = None
        self._profile = None
        self._serial = None

    def get_intrinsics(self) -> Intrinsics:
        if self._intrinsics is None:
            raise AppError("CAM_OPEN_FAIL", "相机未打开，无内参")
        return self._intrinsics

    def grab(self, timeout_ms: int = 1000) -> Optional[tuple[float, np.ndarray]]:
        if self._pipeline is None:
            return None
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms)
        except Exception:  # noqa: BLE001
            return None
        color = frames.get_color_frame()
        if not color:
            return None
        img = np.asanyarray(color.get_data()).copy()
        ts = time.time()
        return ts, img

    @staticmethod
    def _read_intrinsics(rs, profile, width: int, height: int, fps: int) -> Intrinsics:
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = stream.get_intrinsics()
        dist = list(intr.coeffs) if getattr(intr, "coeffs", None) is not None else [0.0] * 5
        while len(dist) < 5:
            dist.append(0.0)
        return Intrinsics(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            dist=[float(x) for x in dist[:5]],
            width=int(width),
            height=int(height),
            fps=int(fps),
        )
