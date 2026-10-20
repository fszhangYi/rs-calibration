from __future__ import annotations

import numpy as np

from rs_calib.qt_compat import QImage, QLabel, QPixmap, Qt


class PreviewLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self.setAlignment(Qt.AlignCenter if hasattr(Qt, "AlignCenter") else Qt.AlignmentFlag.AlignCenter)
        self.setText("无预览")
        self.setStyleSheet("background:#222;color:#aaa;")

    def show_bgr(self, image: np.ndarray) -> None:
        if image is None or image.size == 0:
            return
        h, w = image.shape[:2]
        if image.ndim == 2:
            qimg = QImage(image.data, w, h, w, QImage.Format_Grayscale8)
        else:
            rgb = np.ascontiguousarray(image[:, :, ::-1])
            bytes_per_line = 3 * w
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            # keep reference
            self._rgb = rgb
        pix = QPixmap.fromImage(qimg.copy())
        scaled = pix.scaled(
            self.size(),
            Qt.KeepAspectRatio if hasattr(Qt, "KeepAspectRatio") else Qt.AspectRatioMode.KeepAspectRatio,
            Qt.SmoothTransformation
            if hasattr(Qt, "SmoothTransformation")
            else Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(scaled)
