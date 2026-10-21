from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


class SessionVideoWriter:
    """Prefer mp4; fall back to PNG sequence.

    ``media_dir`` is the directory that will hold ``video.mp4`` or ``frames/``.
    ``rel_path_prefix`` is stored in meta (e.g. ``cameras/<serial>``); empty means session root.
    """

    def __init__(
        self,
        media_dir: Path,
        width: int,
        height: int,
        fps: int,
        fourcc: str = "mp4v",
        *,
        rel_path_prefix: str = "",
    ):
        self.media_dir = Path(media_dir)
        self.media_dir.mkdir(parents=True, exist_ok=True)
        self.width = width
        self.height = height
        self.fps = max(1, int(fps))
        self.fourcc = fourcc
        self.rel_path_prefix = rel_path_prefix.strip().strip("/\\")
        self._writer: Optional[cv2.VideoWriter] = None
        self._use_frames = False
        self._frames_dir: Optional[Path] = None
        self.frame_count = 0
        self._open()

    def _open(self) -> None:
        path = self.media_dir / "video.mp4"
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc[:4].ljust(4, " "))
        writer = cv2.VideoWriter(str(path), fourcc, float(self.fps), (self.width, self.height))
        if writer is not None and writer.isOpened():
            self._writer = writer
            self._use_frames = False
            return
        if writer is not None:
            writer.release()
        self._frames_dir = self.media_dir / "frames"
        self._frames_dir.mkdir(parents=True, exist_ok=True)
        self._use_frames = True
        self._writer = None

    @property
    def info(self) -> dict:
        if self.rel_path_prefix:
            base = self.rel_path_prefix.replace("\\", "/")
            if self._use_frames:
                return {"path": f"{base}/frames", "fourcc": None, "fallback_frames": True}
            return {"path": f"{base}/video.mp4", "fourcc": self.fourcc, "fallback_frames": False}
        if self._use_frames:
            return {"path": "frames", "fourcc": None, "fallback_frames": True}
        return {"path": "video.mp4", "fourcc": self.fourcc, "fallback_frames": False}

    def write(self, image_bgr: np.ndarray) -> None:
        img = image_bgr
        if img.shape[1] != self.width or img.shape[0] != self.height:
            img = cv2.resize(img, (self.width, self.height))
        if self._use_frames:
            assert self._frames_dir is not None
            name = f"{self.frame_count:06d}.png"
            cv2.imwrite(str(self._frames_dir / name), img)
        else:
            assert self._writer is not None
            self._writer.write(img)
        self.frame_count += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self._use_frames and self._frames_dir is not None:
            idx = {"count": self.frame_count, "pattern": "%06d.png"}
            with (self.media_dir / "frames_index.json").open("w", encoding="utf-8") as f:
                json.dump(idx, f, indent=2)


class SessionVideoReader:
    def __init__(
        self,
        session_dir: Path,
        meta: Optional[dict] = None,
        *,
        camera_serial: Optional[str] = None,
        media_dir: Optional[Path] = None,
        video_info: Optional[dict] = None,
    ):
        self.session_dir = Path(session_dir)
        self.meta = meta or {}
        self._cap: Optional[cv2.VideoCapture] = None
        self._frames_dir: Optional[Path] = None
        self._count = int(self.meta.get("frame_count") or 0)
        self._idx = 0

        if media_dir is None and camera_serial is not None:
            from rs_calib.infra import session_store

            cams = session_store.list_cameras_from_meta(self.meta)
            cam = next((c for c in cams if c.get("serial") == camera_serial), None)
            if cam is None:
                raise FileNotFoundError(f"camera {camera_serial} not in meta")
            media_dir = session_store.resolve_camera_media_dir(self.session_dir, cam)
            video_info = video_info or cam.get("video") or {}
        elif media_dir is None:
            # legacy: session root
            media_dir = self.session_dir
            video_info = video_info or (self.meta.get("video") or {})

        self.media_dir = Path(media_dir)
        video = video_info or {}

        if video.get("fallback_frames") or (self.media_dir / "frames").is_dir() and not (
            self.media_dir / "video.mp4"
        ).is_file():
            self._frames_dir = self.media_dir / "frames"
            idx_path = self.media_dir / "frames_index.json"
            if idx_path.is_file():
                with idx_path.open("r", encoding="utf-8") as f:
                    self._count = int(json.load(f).get("count", self._count))
        else:
            mp4 = self.media_dir / "video.mp4"
            if mp4.is_file():
                self._cap = cv2.VideoCapture(str(mp4))
                if self._count <= 0:
                    self._count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            elif (self.media_dir / "frames").is_dir():
                self._frames_dir = self.media_dir / "frames"

    @property
    def frame_count(self) -> int:
        return self._count

    def read_index(self, index: int) -> Optional[np.ndarray]:
        if self._frames_dir is not None:
            path = self._frames_dir / f"{index:06d}.png"
            if not path.is_file():
                return None
            return cv2.imread(str(path))
        if self._cap is None:
            return None
        self._cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
