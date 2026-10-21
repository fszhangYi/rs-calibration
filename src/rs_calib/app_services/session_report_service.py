from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

import numpy as np

from rs_calib.errors import AppError, Err, Ok, Result


def _se3_delta_mm_deg(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    dt = float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0)
    R = A[:3, :3].T @ B[:3, :3]
    cval = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    dR = float(np.degrees(np.arccos(cval)))
    return dt, dR


def _load_json(path: Path) -> Optional[dict[str, Any]]:
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _iter_jsonl(path: Path):
    if not path.is_file():
        return
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            yield json.loads(line)


def build_session_report(session_dir: str | Path) -> Result[dict[str, Any]]:
    """Analyze session files and build a concise Chinese report for the UI log.

    Sources:
    - ``meta.json`` — setup / camera / board / frame_count / timestamps
    - ``joints_poses.jsonl`` — motion span (xyz / rpy / joints), duration
    - ``live_solves.jsonl`` — each solve basis+result, converge/reject history
    - ``extrinsics.json`` — exported result vs locked converge
    """
    sess = Path(session_dir)
    if not sess.is_dir():
        return Err(AppError("REPORT", f"session 不存在: {sess}"))

    meta = _load_json(sess / "meta.json") or {}
    ext = _load_json(sess / "extrinsics.json")
    lines: list[str] = []
    lines.append(f"======== Session 结论 · {sess.name} ========")

    # ---- meta ----
    he = meta.get("handeye") or {}
    ar = meta.get("aruco") or {}
    cam = (meta.get("camera") or {})
    if not cam and meta.get("cameras"):
        cam = meta["cameras"][0]
    lines.append("[概况]")
    lines.append(
        f"  模式={he.get('setup', '?')}  method={he.get('method', '?')}  "
        f"板={ar.get('board_type', '?')}  相机={cam.get('serial', '?')}"
    )
    lines.append(
        f"  录制开始={meta.get('created_at', '?')}  "
        f"frame_count={meta.get('frame_count', '?')}  incomplete={meta.get('incomplete')}"
    )
    lines.append(
        f"  材料: meta.json"
        + (" + joints_poses.jsonl" if (sess / "joints_poses.jsonl").is_file() else "")
        + (" + live_solves.jsonl" if (sess / "live_solves.jsonl").is_file() else "")
        + (" + extrinsics.json" if ext else "")
        + (" + video.mp4" if (sess / "video.mp4").is_file() else "")
    )

    # ---- joints / motion ----
    n_jp = 0
    first_ts = last_ts = None
    joints: list[list[float]] = []
    poses: list[list[float]] = []
    for d in _iter_jsonl(sess / "joints_poses.jsonl") or []:
        n_jp += 1
        ts = d.get("ts_cam")
        if ts is None:
            ts = d.get("ts")
        if first_ts is None:
            first_ts = ts
        last_ts = ts
        if d.get("joints_rad"):
            joints.append(d["joints_rad"])
        if d.get("pose_xyzrpy"):
            poses.append(d["pose_xyzrpy"])

    lines.append("[运动覆盖 · joints_poses.jsonl]")
    if n_jp == 0:
        lines.append("  （无 joints_poses 数据）")
    else:
        dur = None
        if first_ts is not None and last_ts is not None:
            try:
                dur = float(last_ts) - float(first_ts)
            except Exception:  # noqa: BLE001
                dur = None
        lines.append(
            f"  姿态样本={n_jp}"
            + (f"  时长≈{dur:.1f}s" if dur is not None else "")
        )
        if poses:
            P = np.asarray(poses, dtype=float)
            xyz_mm = (P[:, :3].max(0) - P[:, :3].min(0)) * 1000.0
            rpy_deg = (P[:, 3:].max(0) - P[:, 3:].min(0)) * 180.0 / np.pi
            lines.append(
                f"  平移跨度 XYZ≈{xyz_mm[0]:.0f}/{xyz_mm[1]:.0f}/{xyz_mm[2]:.0f} mm"
            )
            lines.append(
                f"  姿态跨度 RPY≈{rpy_deg[0]:.1f}/{rpy_deg[1]:.1f}/{rpy_deg[2]:.1f} °"
            )
        if joints:
            J = np.asarray(joints, dtype=float)
            span = (J.max(0) - J.min(0)) * 180.0 / np.pi
            lines.append("  关节跨度(°)≈" + "/".join(f"{x:.1f}" for x in span.tolist()))

    # ---- live solves ----
    solves = list(_iter_jsonl(sess / "live_solves.jsonl") or [])
    lines.append("[实时求解 · live_solves.jsonl]")
    if not solves:
        lines.append("  （无 live_solves 历史 — 旧 session 或未开实时求解）")
    else:
        n_rej = sum(1 for s in solves if (s.get("extra") or {}).get("rejected_sample"))
        n_acc = len(solves) - n_rej
        first_c = next(
            (
                s
                for s in solves
                if (s.get("convergence") or {}).get("converged")
                and not (s.get("extra") or {}).get("rejected_sample")
            ),
            None,
        )
        last_c = None
        for s in reversed(solves):
            if (s.get("convergence") or {}).get("converged") and not (
                s.get("extra") or {}
            ).get("rejected_sample"):
                last_c = s
                break
        lines.append(
            f"  求解次数={len(solves)}  接受={n_acc}  拒样审计={n_rej}"
        )
        lines.append(
            f"  首次求解 #{solves[0]['solve_idx']} @ {solves[0].get('ts', '?')}"
        )
        lines.append(
            f"  末次求解 #{solves[-1]['solve_idx']} @ {solves[-1].get('ts', '?')}"
        )
        if first_c is None:
            lines.append("  结论: 全程未出现稳定收敛（accepted converge）")
        else:
            c0 = first_c["convergence"]
            r0 = first_c["result"]
            lines.append(
                f"  首次收敛 #{first_c['solve_idx']} @ {first_c.get('ts', '?')}  "
                f"样本={first_c['basis']['n_samples']}  "
                f"Δt={c0.get('delta_t_mm', 0):.3f}mm ΔR={c0.get('delta_R_deg', 0):.3f}° "
                f"streak={c0.get('stable_streak')}"
            )
            T0 = r0.get("T_flange_cam") or r0.get("T_base_cam")
            key = "T_flange_cam" if r0.get("T_flange_cam") is not None else "T_base_cam"
            if T0 is not None:
                xyz = np.asarray(T0, float)[:3, 3]
                lines.append(
                    f"    {key} xyz≈({xyz[0]*1000:.2f}, {xyz[1]*1000:.2f}, {xyz[2]*1000:.2f}) mm"
                )
            if last_c is not None and last_c is not first_c:
                c1 = last_c["convergence"]
                lines.append(
                    f"  末次接受收敛 #{last_c['solve_idx']} @ {last_c.get('ts', '?')}  "
                    f"样本={last_c['basis']['n_samples']}  "
                    f"Δt={c1.get('delta_t_mm', 0):.3f}mm ΔR={c1.get('delta_R_deg', 0):.3f}°"
                )
                T1 = last_c["result"].get("T_flange_cam") or last_c["result"].get("T_base_cam")
                if T0 is not None and T1 is not None:
                    dt, dR = _se3_delta_mm_deg(T0, T1)
                    lines.append(f"  首末收敛差: Δt={dt:.3f}mm ΔR={dR:.3f}°")
            after = [s for s in solves if s["solve_idx"] >= first_c["solve_idx"]]
            n_rej_after = sum(
                1 for s in after if (s.get("extra") or {}).get("rejected_sample")
            )
            lines.append(
                f"  收敛后拒样={n_rej_after}/{len(after)} "
                f"(锁解生效，多余样本被丢弃)"
            )

    # ---- extrinsics ----
    lines.append("[导出外参 · extrinsics.json]")
    if ext is None:
        lines.append("  （尚无 extrinsics.json）")
        verdict = "未导出外参"
        usable = False
    else:
        conv = ext.get("convergence") or {}
        lines.append(
            f"  created_at={ext.get('created_at', '?')}  source={ext.get('source', '?')}  "
            f"solve_idx={ext.get('solve_idx', '?')}"
        )
        lines.append(
            f"  setup={ext.get('setup')}  method={ext.get('method')}  "
            f"n_samples={ext.get('n_samples')}  converged={conv.get('converged')}"
        )
        if conv:
            lines.append(
                f"  状态 Δt={conv.get('delta_t_mm')} mm  ΔR={conv.get('delta_R_deg')}°  "
                f"streak={conv.get('stable_streak')}  "
                f"拒样累计={ext.get('rejected_after_converge', conv.get('warning'))}"
            )
        T = ext.get("T_flange_cam") or ext.get("T_base_cam")
        key = "T_flange_cam" if ext.get("T_flange_cam") is not None else "T_base_cam"
        if T is not None:
            xyz = np.asarray(T, float)[:3, 3]
            lines.append(
                f"  {key} xyz≈({xyz[0]*1000:.2f}, {xyz[1]*1000:.2f}, {xyz[2]*1000:.2f}) mm"
            )
            xyzrpy = ext.get("xyzrpy_flange_cam") or ext.get("xyzrpy_base_cam")
            if xyzrpy:
                lines.append(
                    "  xyzrpy=" + ", ".join(f"{float(x):.5f}" for x in xyzrpy[:6])
                )

        # align with live history
        usable = bool(conv.get("converged"))
        verdict = "外参可用（已标记收敛）" if usable else "外参已写出但未标记收敛，请慎用"
        if solves and T is not None:
            last_c = None
            for s in reversed(solves):
                if (s.get("convergence") or {}).get("converged") and not (
                    s.get("extra") or {}
                ).get("rejected_sample"):
                    last_c = s
                    break
            if last_c is not None:
                Tl = last_c["result"].get("T_flange_cam") or last_c["result"].get(
                    "T_base_cam"
                )
                if Tl is not None:
                    dt, dR = _se3_delta_mm_deg(Tl, T)
                    lines.append(
                        f"  与末次接受收敛解对比: Δt={dt:.3f}mm ΔR={dR:.3f}°"
                        + (" → 一致" if dt < 0.05 and dR < 0.05 else " → 有偏差")
                    )
                    if dt < 0.05 and dR < 0.05:
                        usable = True
                        verdict = "外参可用：与锁定收敛解一致"

    lines.append("[结论]")
    lines.append(f"  {verdict}")
    lines.append("======== 报告结束 ========")

    report = "\n".join(lines)
    return Ok(
        {
            "session": str(sess),
            "report": report,
            "usable": usable,
            "has_live_solves": bool(solves),
            "has_extrinsics": ext is not None,
            "converged": bool((ext or {}).get("convergence", {}).get("converged"))
            if ext
            else bool(
                any(
                    (s.get("convergence") or {}).get("converged")
                    and not (s.get("extra") or {}).get("rejected_sample")
                    for s in solves
                )
            ),
        }
    )
