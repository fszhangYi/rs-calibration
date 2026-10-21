from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Optional


@dataclass
class AppSection:
    language: str = "zh_CN"
    session_root: str = "./sessions"


@dataclass
class CameraSection:
    width: int = 1280
    height: int = 720
    fps: int = 15


@dataclass
class ArmSection:
    robot_ip: str = "10.111.34.200"
    robot_tcp_port: int = 54321
    num_joints: int = 6
    max_delta_deg: float = 2.0
    poll_hz: float = 50.0
    jog_joint_step_deg: float = 1.0
    jog_cart_step_m: float = 0.005
    jog_cart_step_deg: float = 2.0
    cart_frame: str = "flange"


@dataclass
class ArucoSection:
    dict: str = "DICT_5X5_50"
    marker_id: int = 0
    marker_length_m: float = 0.15
    T_flange_marker: Optional[list[list[float]]] = None


@dataclass
class HandeyeSection:
    # eye_to_hand (场景相机) | eye_in_hand (腕部相机)
    setup: str = "eye_to_hand"
    method: str = "TSAI"
    compare_methods: bool = False


@dataclass
class ConvergenceSection:
    min_samples: int = 12
    recompute_every_n: int = 1
    max_delta_t_mm: float = 1.5
    max_delta_R_deg: float = 0.5
    stable_count: int = 5
    min_translation_span_m: float = 0.15
    min_rotation_span_deg: float = 25.0
    dup_joint_eps_deg: float = 0.3
    dup_pose_eps_mm: float = 3.0
    dup_pose_eps_deg: float = 1.0
    outlier_jump_mm: float = 50.0
    outlier_jump_deg: float = 30.0


@dataclass
class RecordSection:
    video_fourcc: str = "mp4v"
    jpeg_quality: int = 90


@dataclass
class VerifySection:
    axis_length_m: float = 0.05
    show_tcp_axes: bool = True
    show_marker_error: bool = True


@dataclass
class MatrixPathSection:
    nx: int = 3
    ny: int = 3
    nz: int = 1
    step_x_m: float = 0.05
    step_y_m: float = 0.05
    step_z_m: float = 0.03
    n_rx: int = 1
    n_ry: int = 3
    n_rz: int = 1
    step_rx_deg: float = 10.0
    step_ry_deg: float = 12.0
    step_rz_deg: float = 10.0
    snake: bool = True
    dwell_s: float = 0.8
    frame: str = "flange"

    def to_dict(self) -> dict:
        from dataclasses import asdict

        return asdict(self)


@dataclass
class AppConfig:
    app: AppSection = field(default_factory=AppSection)
    camera: CameraSection = field(default_factory=CameraSection)
    arm: ArmSection = field(default_factory=ArmSection)
    tcp_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.18])
    aruco: ArucoSection = field(default_factory=ArucoSection)
    handeye: HandeyeSection = field(default_factory=HandeyeSection)
    convergence: ConvergenceSection = field(default_factory=ConvergenceSection)
    record: RecordSection = field(default_factory=RecordSection)
    verify: VerifySection = field(default_factory=VerifySection)
    matrix_path: MatrixPathSection = field(default_factory=MatrixPathSection)

    def reset_convergence_defaults(self) -> None:
        self.convergence = ConvergenceSection()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def default_app_config() -> AppConfig:
    return AppConfig()


def _merge_dataclass(dc_obj: Any, data: dict[str, Any]) -> Any:
    if not is_dataclass(dc_obj) or not isinstance(data, dict):
        return dc_obj
    kwargs = {}
    for f in fields(dc_obj):
        if f.name not in data:
            kwargs[f.name] = getattr(dc_obj, f.name)
            continue
        cur = getattr(dc_obj, f.name)
        val = data[f.name]
        if is_dataclass(cur) and isinstance(val, dict):
            kwargs[f.name] = _merge_dataclass(cur, val)
        else:
            kwargs[f.name] = val
    return type(dc_obj)(**kwargs)


def app_config_from_dict(data: dict[str, Any]) -> AppConfig:
    return _merge_dataclass(AppConfig(), data)


def resolve_config_path(path: Optional[str | Path] = None) -> Path:
    import os

    if path is not None:
        return Path(path)
    env = (os.environ.get("RS_CALIB_CONFIG") or "").strip()
    if env:
        return Path(env)
    here = Path(__file__).resolve().parents[3]
    return here / "configs" / "default.yaml"
