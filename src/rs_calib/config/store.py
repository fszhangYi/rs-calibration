from __future__ import annotations

from pathlib import Path
from typing import Optional, Union

import yaml

from rs_calib.config.schema import AppConfig, app_config_from_dict, resolve_config_path


def load_config(path: Optional[Union[str, Path]] = None) -> AppConfig:
    p = resolve_config_path(path)
    if not p.is_file():
        return AppConfig()
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return app_config_from_dict(data)


def save_config(cfg: AppConfig, path: Optional[Union[str, Path]] = None) -> Path:
    p = resolve_config_path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg.to_dict(), f, allow_unicode=True, sort_keys=False)
    return p
