from __future__ import annotations

from typing import Optional

from rs_calib.domain.types import DeviceInfo, Intrinsics
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra.realsense_adapter import RealSenseAdapter


class CameraService:
    def __init__(self) -> None:
        self.adapter = RealSenseAdapter()
        self.selected_serial: Optional[str] = None
        self.device_name: str = ""
        self.ui_intrinsics: Optional[Intrinsics] = None

    def scan(self) -> Result[list[DeviceInfo]]:
        try:
            return Ok(self.adapter.list_devices())
        except AppError as e:
            return Err(e)

    def open(self, serial: str, width: int, height: int, fps: int) -> Result[Intrinsics]:
        try:
            intr = self.adapter.open(serial, width, height, fps)
            self.selected_serial = serial
            self.ui_intrinsics = Intrinsics(
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.cx,
                cy=intr.cy,
                dist=list(intr.dist),
                width=intr.width,
                height=intr.height,
                fps=intr.fps,
            )
            return Ok(self.ui_intrinsics)
        except AppError as e:
            return Err(e)

    def close(self) -> None:
        self.adapter.close()

    def reread_intrinsics(self) -> Result[Intrinsics]:
        try:
            intr = self.adapter.get_intrinsics()
            self.ui_intrinsics = Intrinsics(
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.cx,
                cy=intr.cy,
                dist=list(intr.dist),
                width=intr.width,
                height=intr.height,
                fps=intr.fps,
            )
            return Ok(self.ui_intrinsics)
        except AppError as e:
            return Err(e)

    def current_intrinsics(self) -> Optional[Intrinsics]:
        return self.ui_intrinsics
