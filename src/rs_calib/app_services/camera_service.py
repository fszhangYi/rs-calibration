from __future__ import annotations

from typing import Optional

import numpy as np

from rs_calib.domain.types import DeviceInfo, Intrinsics
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra.realsense_adapter import RealSenseAdapter


class CameraService:
    """Manage one or more RealSense color streams."""

    def __init__(self) -> None:
        self._adapters: dict[str, RealSenseAdapter] = {}
        self.device_names: dict[str, str] = {}
        self.ui_intrinsics: dict[str, Intrinsics] = {}
        self.preview_serial: Optional[str] = None

    # ---- compatibility aliases (primary / preview camera) ----
    @property
    def adapter(self) -> RealSenseAdapter:
        """Primary adapter for legacy single-cam call sites."""
        serial = self.preview_serial
        if serial and serial in self._adapters:
            return self._adapters[serial]
        if self._adapters:
            return next(iter(self._adapters.values()))
        # empty placeholder so worker can poll is_open safely
        if not hasattr(self, "_empty"):
            self._empty = RealSenseAdapter()
        return self._empty

    @property
    def selected_serial(self) -> Optional[str]:
        return self.preview_serial

    @selected_serial.setter
    def selected_serial(self, value: Optional[str]) -> None:
        self.preview_serial = value

    @property
    def device_name(self) -> str:
        if self.preview_serial:
            return self.device_names.get(self.preview_serial, "")
        return ""

    @device_name.setter
    def device_name(self, value: str) -> None:
        if self.preview_serial:
            self.device_names[self.preview_serial] = value or ""

    def scan(self) -> Result[list[DeviceInfo]]:
        try:
            return Ok(RealSenseAdapter.list_devices())
        except AppError as e:
            return Err(e)

    @property
    def open_serials(self) -> list[str]:
        return [s for s, a in self._adapters.items() if a.is_open]

    @property
    def n_open(self) -> int:
        return len(self.open_serials)

    @property
    def is_multi(self) -> bool:
        return self.n_open > 1

    def open(
        self,
        serial: str,
        width: int,
        height: int,
        fps: int,
        *,
        name: str = "",
        replace_all: bool = True,
    ) -> Result[Intrinsics]:
        """Open one camera. If replace_all, close others first (single-cam UX)."""
        if replace_all:
            self.close()
        return self._open_one(serial, width, height, fps, name=name)

    def open_many(
        self,
        serials: list[str],
        width: int,
        height: int,
        fps: int,
        *,
        names: Optional[dict[str, str]] = None,
    ) -> Result[dict[str, Intrinsics]]:
        """Open multiple cameras; closes any previous streams first."""
        serials = [s.strip() for s in serials if s and str(s).strip()]
        if not serials:
            return Err(AppError("CAM_OPEN_FAIL", "未指定相机序列号"))
        # preserve order, unique
        seen: set[str] = set()
        ordered: list[str] = []
        for s in serials:
            if s not in seen:
                seen.add(s)
                ordered.append(s)
        self.close()
        opened: dict[str, Intrinsics] = {}
        try:
            for s in ordered:
                nm = (names or {}).get(s, "")
                res = self._open_one(s, width, height, fps, name=nm)
                if not isinstance(res, Ok):
                    self.close()
                    return Err(res.error)
                opened[s] = res.value
            return Ok(opened)
        except Exception as e:  # noqa: BLE001
            self.close()
            return Err(AppError("CAM_OPEN_FAIL", "多相机开流失败", str(e)))

    def _open_one(
        self,
        serial: str,
        width: int,
        height: int,
        fps: int,
        *,
        name: str = "",
    ) -> Result[Intrinsics]:
        if serial in self._adapters and self._adapters[serial].is_open:
            # already open — refresh intrinsics
            try:
                intr = self._adapters[serial].get_intrinsics()
                self.ui_intrinsics[serial] = Intrinsics(
                    fx=intr.fx,
                    fy=intr.fy,
                    cx=intr.cx,
                    cy=intr.cy,
                    dist=list(intr.dist),
                    width=intr.width,
                    height=intr.height,
                    fps=intr.fps,
                )
                if self.preview_serial is None:
                    self.preview_serial = serial
                return Ok(self.ui_intrinsics[serial])
            except AppError as e:
                return Err(e)
        adapter = RealSenseAdapter()
        try:
            intr = adapter.open(serial, width, height, fps)
        except AppError as e:
            return Err(e)
        self._adapters[serial] = adapter
        self.device_names[serial] = name or ""
        self.ui_intrinsics[serial] = Intrinsics(
            fx=intr.fx,
            fy=intr.fy,
            cx=intr.cx,
            cy=intr.cy,
            dist=list(intr.dist),
            width=intr.width,
            height=intr.height,
            fps=intr.fps,
        )
        if self.preview_serial is None:
            self.preview_serial = serial
        return Ok(self.ui_intrinsics[serial])

    def set_preview_serial(self, serial: str) -> Result[Intrinsics]:
        if serial not in self._adapters or not self._adapters[serial].is_open:
            return Err(AppError("CAM_OPEN_FAIL", f"相机未打开: {serial}"))
        self.preview_serial = serial
        return Ok(self.ui_intrinsics[serial])

    def close(self, serial: Optional[str] = None) -> None:
        if serial is None:
            for a in list(self._adapters.values()):
                a.close()
            self._adapters.clear()
            self.ui_intrinsics.clear()
            self.device_names.clear()
            self.preview_serial = None
            return
        a = self._adapters.pop(serial, None)
        if a is not None:
            a.close()
        self.ui_intrinsics.pop(serial, None)
        self.device_names.pop(serial, None)
        if self.preview_serial == serial:
            self.preview_serial = next(iter(self._adapters), None)

    def reread_intrinsics(self, serial: Optional[str] = None) -> Result[Intrinsics]:
        serial = serial or self.preview_serial
        if not serial or serial not in self._adapters:
            return Err(AppError("CAM_OPEN_FAIL", "相机未打开，无内参"))
        try:
            intr = self._adapters[serial].get_intrinsics()
            self.ui_intrinsics[serial] = Intrinsics(
                fx=intr.fx,
                fy=intr.fy,
                cx=intr.cx,
                cy=intr.cy,
                dist=list(intr.dist),
                width=intr.width,
                height=intr.height,
                fps=intr.fps,
            )
            return Ok(self.ui_intrinsics[serial])
        except AppError as e:
            return Err(e)

    def current_intrinsics(self, serial: Optional[str] = None) -> Optional[Intrinsics]:
        serial = serial or self.preview_serial
        if serial is None:
            return None
        return self.ui_intrinsics.get(serial)

    def grab_all(self, timeout_ms: int = 1000) -> dict[str, tuple[float, np.ndarray]]:
        """Grab one frame from each open camera. Missing streams are omitted."""
        out: dict[str, tuple[float, np.ndarray]] = {}
        for serial, adapter in list(self._adapters.items()):
            if not adapter.is_open:
                continue
            grabbed = adapter.grab(timeout_ms)
            if grabbed is None:
                continue
            out[serial] = grabbed
        return out
