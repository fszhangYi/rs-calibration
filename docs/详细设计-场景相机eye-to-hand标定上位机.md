# 场景相机 Eye-to-Hand 标定上位机 — 详细设计

| 项 | 内容 |
|---|---|
| 文档版本 | **v1.0** |
| 日期 | 2026-10-20 |
| 依据 | 需求报告 **v1.1**、概念设计 **v1.0** |
| 目标平台 | Windows x64；PyQt；Python 3.10+ |
| 状态 | 设计冻结稿（实现前可再改一版接口，改后升 v1.1） |

---

## 1. 文档说明

本文规定实现级结构：包划分、类与接口、状态机、线程/信号、文件 schema、算法步骤、UI 控件映射与错误处理。  
不写具体业务源码；伪代码仅阐明契约。

**命名约定：**

- 齐次变换 `T_a_b` 表示 \(T_{a\leftarrow b}\)（把 b 系坐标变到 a 系），与需求导出 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\) 一致。  
- 代码标识符：`T_base_cam`、`T_base_flange`、`T_cam_marker`。

---

## 2. 工程结构

```
rs-calibration/
├── docs/
│   ├── 需求报告-….md
│   ├── 概念设计-….md
│   └── 详细设计-….md          # 本文
├── configs/
│   └── default.yaml
├── vendor/
│   └── sensors -> <sensors/src/sensors 或安装路径>
├── src/
│   └── rs_calib/
│       ├── __init__.py
│       ├── __main__.py          # python -m rs_calib
│       ├── app.py               # QApplication 入口
│       ├── version.py
│       ├── config/
│       │   ├── schema.py        # dataclass + 默认值
│       │   └── store.py         # load/save yaml
│       ├── domain/
│       │   ├── types.py         # Pose, Intrinsics, Sample…
│       │   ├── math3d.py        # inv, xyzrpy, delta_R/t
│       │   ├── aruco.py
│       │   ├── handeye.py
│       │   ├── filter_select.py
│       │   ├── convergence.py
│       │   └── verify_geom.py
│       ├── infra/
│       │   ├── realsense_adapter.py
│       │   ├── elite_arm_adapter.py
│       │   ├── kinematics_bridge.py
│       │   ├── session_store.py
│       │   └── video_io.py
│       ├── app_services/        # Application 用例
│       │   ├── camera_service.py
│       │   ├── arm_service.py
│       │   ├── record_service.py
│       │   ├── live_calib_service.py
│       │   ├── offline_calib_service.py
│       │   └── verify_service.py
│       ├── workers/             # QThread / 可运行对象
│       │   ├── camera_worker.py
│       │   ├── arm_poll_worker.py
│       │   ├── calib_worker.py
│       │   └── file_job_worker.py
│       └── ui/
│           ├── main_window.py
│           ├── widgets_camera.py
│           ├── widgets_arm.py
│           ├── widgets_record.py
│           ├── widgets_live.py
│           ├── dialog_offline.py
│           ├── dialog_verify.py
│           ├── dialog_params.py
│           └── preview_label.py
├── tests/
│   ├── test_math3d.py
│   ├── test_handeye_synthetic.py
│   ├── test_filter_select.py
│   ├── test_convergence.py
│   └── test_session_store.py
└── pyproject.toml / requirements.txt
```

**PYTHONPATH：** `src` + `vendor` 父路径，使 `import sensors` 与 `import rs_calib` 同时可用。

---

## 3. 配置模型

### 3.1 `configs/default.yaml` 字段

```yaml
app:
  language: zh_CN
  session_root: "./sessions"

camera:
  width: 1280
  height: 720
  fps: 15
  # serial 空表示未选

arm:
  robot_ip: "10.111.34.200"
  robot_tcp_port: 54321
  num_joints: 6
  max_delta_deg: 2.0
  poll_hz: 50
  jog_joint_step_deg: 1.0
  jog_cart_step_m: 0.005
  jog_cart_step_deg: 2.0
  cart_frame: "flange"   # flange | tcp（显示与增量参考）

tcp_xyz: [0.0, 0.0, 0.18]

aruco:
  dict: DICT_5X5_50
  marker_id: 0
  marker_length_m: 0.15
  # 可选，仅验证 B
  T_flange_marker: null    # null → I4

handeye:
  method: TSAI             # TSAI|PARK|HORAUD|ANDREFF|DANIILIDIS
  compare_methods: false

convergence:               # 与需求 §7 一致
  min_samples: 12
  recompute_every_n: 1
  max_delta_t_mm: 1.5
  max_delta_R_deg: 0.5
  stable_count: 5
  min_translation_span_m: 0.15
  min_rotation_span_deg: 25
  dup_joint_eps_deg: 0.3
  dup_pose_eps_mm: 3.0
  dup_pose_eps_deg: 1.0
  outlier_jump_mm: 50.0
  outlier_jump_deg: 30.0

record:
  video_fourcc: "mp4v"     # 失败则 frames/
  jpeg_quality: 90

verify:
  axis_length_m: 0.05
  show_tcp_axes: true
  show_marker_error: true
```

### 3.2 配置 API

```text
ConfigStore.load(path) -> AppConfig
ConfigStore.save(path, AppConfig)
AppConfig.reset_convergence_defaults()
```

`AppConfig` 为 dataclass 树；UI「恢复推荐值」只重置 `convergence` 段。

---

## 4. Domain 类型与数学

### 4.1 核心类型（`domain/types.py`）

| 类型 | 字段 |
|---|---|
| `Intrinsics` | `fx,fy,cx,cy`；`dist: (5,)`；`width,height,fps` |
| `FlangeSample` | `ts_cam, ts_arm, frame_idx, joints_rad[6], T_base_flange(4,4), pose_xyzrpy[6]` |
| `CalibSample` | `frame_idx, T_base_flange, T_cam_marker` |
| `ExtrinsicsResult` | `T_base_cam, T_cam_base, method, n_samples, used_indices, metrics` |
| `ConvergenceStatus` | `n_valid, delta_t_mm, delta_R_deg, stable_streak, converged: bool, warning: str\|None` |
| `FilterReport` | `n_raw, n_after_detect, n_after_dup, n_after_outlier, n_selected, indices` |
| `VerifyFrameMetrics` | `frame_idx, marker_err_t_mm, marker_err_R_deg, has_detect` |

### 4.2 `math3d` 必备函数

| 函数 | 行为 |
|---|---|
| `inv_T(T)` | 刚体逆 |
| `T_from_xyzrpy(xyzrpy)` / `xyzrpy_from_T(T)` | XYZ 外旋，与 `sensors.kinematics` / scipy `xyz` 一致 |
| `T_flange_tcp(tcp_xyz)` | 平移齐次阵 |
| `rotation_angle_deg(R_a, R_b)` | 相对旋转角 |
| `se3_delta(T_a, T_b)` | → `(t_mm, R_deg)` |
| `project_axes(K, dist, T_cam_obj, axis_len)` | → 图像折线端点 |

---

## 5. Domain 算法详细设计

### 5.1 ArUco（`domain/aruco.py`）

```text
detect_marker(image_bgr, intrinsics, cfg) -> Optional[T_cam_marker]
  1. gray = BGR2GRAY
  2. detectMarkers(dict=DICT_5X5_50)
  3. 若 ids 不含 marker_id → None
  4. estimatePoseSingleMarkers(corners[id], length, K, dist)
  5. Rodrigues(rvec) → R; 组装 T_cam_marker
```

可选返回画好轴的 debug 图，供预览叠加。

### 5.2 HandEye（`domain/handeye.py`）

**输入：** `List[CalibSample]`，`method`。

**OpenCV 约定处理（实现必须写清并单测）：**

按 OpenCV 文档 *eye-to-hand*：

- `R_gripper2base, t_gripper2base`：由 **法兰** `T_base_flange` 取 \(R,t\) 即 gripper→base 的旋转平移（若实测符号相反，以合成测试为准，在此函数内统一换算，**导出层始终输出 `T_base_cam`**）。  
- `R_target2cam, t_target2cam`：来自 `T_cam_marker`。

```text
solve_handeye(samples, method) -> ExtrinsicsResult
  断言 len(samples) >= 3（UI 层另用 min_samples=12 做收敛）
  组装 R_g2b, t_g2b, R_t2c, t_t2c
  R_c2b, t_c2b = cv2.calibrateHandEye(..., method=map[method])
  # 文档：eye-to-hand 时返回 cam2base → 即 T_base_cam 的 R,t
  T_base_cam = Rt_to_T(R_c2b, t_c2b)
  T_cam_base = inv_T(T_base_cam)
  return ExtrinsicsResult(...)
```

`method` 映射：

| 名 | OpenCV 常量 |
|---|---|
| TSAI | `CALIB_HAND_EYE_TSAI` |
| PARK | `CALIB_HAND_EYE_PARK` |
| HORAUD | `CALIB_HAND_EYE_HORAUD` |
| ANDREFF | `CALIB_HAND_EYE_ANDREFF` |
| DANIILIDIS | `CALIB_HAND_EYE_DANIILIDIS` |

**禁止**把 `tcp_xyz`、`T_flange_marker` 传入本函数。

### 5.3 滤波与选点（`domain/filter_select.py`）

流水线：

```text
raw frames (img + FlangeSample)
  → detect → list[(idx, CalibSample)]          # 丢无板
  → drop_duplicates(joints/pose eps)           # 近重复
  → drop_outliers(jump mm/deg vs prev kept)    # 异常跳变
  → ensure_span_or_warn(translation/rotation)  # 不足则 warning，不硬失败
  → select_diverse(k or coverage)              # 贪心：最大化与已选集合的最小 SE3 距离
  → return samples_selected, FilterReport
```

`select_diverse` 默认：若过滤后 ≤ 40 则全用；否则选 `max(min_samples*2, 24)` 个最分散点。

### 5.4 收敛（`domain/convergence.py`）

```text
class ConvergenceTracker:
  update(new_result: ExtrinsicsResult) -> ConvergenceStatus
    if n < min_samples: converged=False
    else:
      δt, δR = se3_delta(prev.T, new.T)
      if δt<=max_delta_t_mm and δR<=max_delta_R_deg: streak++
      else: streak=0
      converged = streak >= stable_count
```

实时：每新增 `recompute_every_n` 个**通过实时去重**的样本后调用 `solve` + `update`。  
离线：对最终解可只报 `FilterReport`；若做「增量重放」可选跑同一 Tracker。

### 5.5 验证几何（`domain/verify_geom.py`）

```text
# 主误差（默认）
T_cam_marker_pred = T_cam_base @ T_base_flange @ T_flange_marker
# T_flange_marker 默认 I
err = se3_delta(T_cam_marker_pred, T_cam_marker_det)

# 辅：TCP 轴
T_cam_tcp_A = T_cam_base @ T_base_flange @ T_flange_tcp(tcp_xyz)
T_cam_tcp_B = T_cam_marker_det @ inv(T_flange_marker) @ T_flange_tcp(tcp_xyz)
```

绘制：`cv2.line`；A 用 BGR `(0,0,255)/(0,255,0)/(255,0,0)`；B 用 `(255,255,0)` 三轴同色或青品红黄。

---

## 6. Infrastructure 适配器

### 6.1 `RealSenseAdapter`

封装 `sensors.drivers.camera.realsense` 能力；若 Sensor 类绑 YAML 过重，允许同文件调用 `pyrealsense2` **仅通过 adapter 内集中调用**，但枚举/内参字段命名与 sensors 的 `camera_infos` 对齐。

| 方法 | 说明 |
|---|---|
| `list_devices() -> list[DeviceInfo]` | serial, name, firmware |
| `open(serial, w,h,fps)` | 启动 pipeline |
| `close()` | |
| `get_intrinsics() -> Intrinsics` | color |
| `grab(timeout_ms) -> (ts, bgr_ndarray) \| None` | |
| `is_open -> bool` | |

**约束：** 同时只 open 一台（本期单相机）。

### 6.2 `EliteArmAdapter`

包装 `EliteArmWriteSensor`：

| 方法 | 对应 sensors |
|---|---|
| `connect(ip, port=54321)` | 构造 + `open()`；失败抛/返回 `Result` |
| `disconnect()` | `close()` |
| `arm()` / `disarm(stop=True)` / `stop()` | `write({arm})` 等 |
| `read_state() -> ArmState` | `read()` → joints + armed |
| `jog_joint(idx, delta_rad)` | `write({jog_joint, delta_rad})` |
| `move_joints_rad(q)` | `write({joints_rad})` |

`ArmState`: `ok, joints_rad, armed, endpoint, error, ts`。

### 6.3 `KinematicsBridge`

| 方法 | 实现 |
|---|---|
| `fk_flange(joints_rad) -> T(4,4)` | `sensors.kinematics.fk_flange_from_joints_rad` |
| `ik_flange(T, seed_rad) -> (ok, joints_rad)` | `sensors.kinematics.ik_flange`（注意角度单位：与 sensors API 对齐，内部统一 rad↔deg） |
| `fk_tcp(joints_rad, tcp_xyz) -> T` | `fk_flange @ T_flange_tcp` |
| `cart_jog(joints, tcp_xyz, frame, delta_xyzrpy) -> (ok, q_new, err)` | 当前法兰或 TCP 位姿左乘/右乘增量 → IK → 返回目标关节 |

**笛卡尔点动约定（冻结）：**

- `frame=flange`：在法兰系下右乘微小 `Δ`（工具系点动）或按 UI 选择「基座系平移」——**默认：基座系平移 + 当前姿态下的局部旋转（XYZ 增量加在欧拉上）**，实现简单、示教直观。  
- `frame=tcp`：先变到 TCP 位姿再同样规则，IK 目标仍是法兰（先右乘 `inv(T_flange_tcp)`）。  
- 每次点动只发 **一拍** `joints_rad`；若 `|Δq|_∞` 超过 `max_delta`，Application 拆成多拍短步进（循环调用 adapter），每拍间隔 ~20–40 ms，UI 可中断。

### 6.4 `SessionStore` / `VideoIO`

见 §8。

---

## 7. Application 服务与状态机

### 7.1 相机服务状态

```
Idle ──scan──► Listed ──select+open──► Streaming ──close──► Listed
                  │                        │
                  └──────── error ◄────────┘
```

### 7.2 臂服务状态

```
Disconnected ──connect──► Connected ──arm──► Armed ──disarm/stop──► Connected
       ▲                     │                  │
       └──────disconnect─────┴──────────────────┘
```

- 仅 `Armed` 接受 jog / move。  
- `Connected` 仍可读 joints（monitor）。

### 7.3 录制状态

```
Idle ──start──► Recording ──stop──► Finalizing ──► Idle(session_path)
                   │
                   └── error → 尽量 flush 已写数据，标 meta.incomplete=true
```

### 7.4 `RecordService` 对齐逻辑

```text
on_camera_frame(ts_cam, image):
  arm = last_arm_snapshot   # 来自 poll，允许空则跳过帧或写 joints=null（推荐：跳过不落盘）
  T = fk_flange(arm.joints_rad)
  sample = FlangeSample(...)
  if recording:
    video.write(image)
    jsonl.append(sample)
  preview.emit(image)
  if live_calib_enabled:
    live_calib_service.offer_frame(image, sample)
```

### 7.5 `LiveCalibService`

```text
offer_frame(image, sample):
  T_cm = detect(...)
  if None: return
  if is_duplicate(sample, last_kept): return
  buffer.append(CalibSample(...))
  if len(buffer) % recompute_every_n == 0 and len(buffer) >= 3:
    queue_to_calib_worker(buffer.copy())
```

Worker 回传 `ExtrinsicsResult` → `ConvergenceTracker.update` → 信号 `calib_updated`。

### 7.6 `OfflineCalibService`

输入 path → 后台：

1. 读 meta / 缺省用当前 AppConfig  
2. 逐帧读视频 + jsonl（按 `frame_idx`）  
3. `filter_select` → `solve_handeye`  
4. 可选多 method 对比：若平移差 > 5 mm 或角差 > 2° → warning  
5. 写 `extrinsics.json` + 返回 `FilterReport`  

### 7.7 `VerifyService`

加载三件套 → 播放时钟（UI QTimer 或 worker）→ 每帧 `verify_geom` → 叠加图 → metrics 列表可导出 CSV。

---

## 8. 文件 Schema

### 8.1 Session 目录

```
sessions/session_YYYYMMDD_HHMMSS/
  meta.json
  video.mp4                 # 或 frames/%06d.png + frames_index.json
  joints_poses.jsonl
  extrinsics.json           # 可选
  offline_report.json       # 可选
```

### 8.2 `meta.json`

```json
{
  "schema_version": 1,
  "created_at": "2026-10-20T20:00:00+08:00",
  "software_version": "0.1.0",
  "incomplete": false,
  "camera": {
    "serial": "……",
    "name": "Intel RealSense D435",
    "width": 1280,
    "height": 720,
    "fps": 15,
    "intrinsics": {
      "fx": 0, "fy": 0, "cx": 0, "cy": 0,
      "dist": [0, 0, 0, 0, 0]
    }
  },
  "arm": {
    "robot_ip": "10.111.34.200",
    "model": "EC616"
  },
  "tcp_xyz": [0.0, 0.0, 0.18],
  "aruco": {
    "dict": "DICT_5X5_50",
    "marker_id": 0,
    "marker_length_m": 0.15
  },
  "T_flange_marker": null,
  "video": {
    "path": "video.mp4",
    "fourcc": "mp4v",
    "fallback_frames": false
  },
  "frame_count": 0
}
```

停止录制时回写 `frame_count`。

### 8.3 `joints_poses.jsonl`

每行一个 JSON 对象，字段强制：

```json
{
  "ts_cam": 1730000000.123,
  "ts_arm": 1730000000.120,
  "frame_idx": 0,
  "joints_rad": [0, 0, 0, 0, 0, 0],
  "T_base_flange": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "pose_xyzrpy": [0, 0, 0, 0, 0, 0]
}
```

`pose_xyzrpy` 必须与 `T_base_flange` 一致（由 FK 生成，禁止手填分叉）。

### 8.4 `extrinsics.json`

```json
{
  "schema_version": 1,
  "T_base_cam": [[...]],
  "T_cam_base": [[...]],
  "xyzrpy_base_cam": [x,y,z,rx,ry,rz],
  "method": "TSAI",
  "n_samples": 24,
  "used_frame_indices": [1, 5, 9],
  "convergence": {
    "delta_t_mm": 0.4,
    "delta_R_deg": 0.12,
    "converged": true
  },
  "source_session": "session_…",
  "created_at": "…"
}
```

### 8.5 视频降级

若 `VideoWriter.open` 失败：

- 创建 `frames/`，写 `%06d.png`  
- `meta.video.fallback_frames=true`，`path="frames"`  
- `frames_index.json`: `{"count": N, "pattern": "%06d.png"}`

---

## 9. 线程、信号与队列

### 9.1 Worker 一览

| Worker | 父 | 周期/触发 | 出信号（例） |
|---|---|---|---|
| `CameraWorker` | QThread | grab 循环 | `frame_ready(np.ndarray, float ts)` |
| `ArmPollWorker` | QThread | `1/poll_hz` | `arm_state(ArmState)` |
| `CalibWorker` | QThread | 队列任务 | `result_ready(ExtrinsicsResult)` `failed(str)` |
| `FileJobWorker` | QThread | 离线/验证批处理 | `progress(int,int)` `finished(object)` |

### 9.2 队列容量

| 队列 | 容量 | 满时策略 |
|---|---|---|
| preview 到 UI | 1 | 丢弃旧帧 |
| live → CalibWorker | 2 | 丢弃旧任务，保留最新 buffer 快照 |
| 录制写盘 | 32 | 阻塞短等；持续满则告警并停录 |

### 9.3 线程安全

- Adapter 的 `EliteArmWriteSensor` 自带锁；ArmPoll 与 Jog **都走同一 adapter 实例**。  
- `last_arm_snapshot` 用 `threading.Lock` 或原子替换不可变 dataclass。  
- NumPy 图像跨线程：**copy** 后再 emit（或文档约定接收方立即 copy）。

### 9.4 Qt 类型注册

对自定义结果用 `pyqtSignal(object)` 传 dataclass/dict，避免元类型注册麻烦。

---

## 10. UI 详细设计

### 10.1 主窗口分区

| 区域 | 控件 |
|---|---|
| 相机 | `QListWidget` 设备；刷新；开流/关流；内参 `QDoubleSpinBox`×4 + dist；「重新读取」 |
| 预览 | `PreviewLabel`（QLabel 缩放显示）；可选叠加 ArUco 轴 |
| 臂 | IP `QLineEdit`；连接/断开；状态灯；Arm / Disarm / Stop |
| 点动 | 关节 J1–J6 ± 按钮；步进角；笛卡尔 ±X±Y±Z±Rx±Ry±Rz；步进 m/°；`frame` 下拉 flange/tcp |
| 录制 | 开始/停止；路径标签；实时计算 `QCheckBox` |
| 实时标定 | 样本数、收敛灯、Δt/ΔR、`T_base_cam` 摘要、导出按钮 |
| 菜单 | 参数、离线计算、验证、打开目录、退出 |

### 10.2 对话框

**参数对话框：** 收敛全字段、ArUco、tcp_xyz、点动默认步进、HandEye method；「恢复收敛推荐值」。

**离线对话框：** 选 video、jsonl、meta（可选）；运行；进度条；结果文本 + 保存路径。

**验证对话框：** 选 video、jsonl、extrinsics；内参来源（meta/当前）；播放条；主误差数字；图例。

### 10.3 交互约束

| 条件 | UI 行为 |
|---|---|
| 未 Streaming | 禁用录制 |
| 未 Connected | 禁用 Arm/点动 |
| 未 Armed | 点动按钮禁用（或点按提示） |
| Recording | 允许点动；禁止改相机 serial / 关流（或先停录） |
| 收敛 | 状态标签变绿；允许「写入 session/extrinsics.json」 |

### 10.4 连接反馈文案（示例）

- 成功：`已连接 10.111.34.200:54321，关节已刷新`  
- 失败：`连接失败：端口不通 / elite SDK 缺失 / monitor 超时`（透传 adapter error）

---

## 11. 错误模型

```text
class AppError(Exception):
  code: str          # CAM_OPEN_FAIL | ARM_CONNECT_FAIL | ARM_NOT_ARMED |
                     # JOG_REJECTED | RECORD_IO | CALIB_FAIL | VERIFY_IO
  message: str       # 中文用户可读
  detail: str|None   # 技术细节打日志
```

所有 Service 方法返回 `Result[T] = Ok(T) | Err(AppError)`，或抛 `AppError` 由 UI 统一 `QMessageBox`。

日志：`logging` → 文件 `logs/rs_calib_YYYYMMDD.log` + 可选 UI 日志窗。

---

## 12. 测试设计

| 用例 | 断言 |
|---|---|
| 合成 HandEye | 随机 `T_base_cam`、多组法兰相对运动、虚拟板 → 恢复误差 t<1e-3 m、R<0.1° |
| 滤波去重 | 重复关节被剔；多样性选点覆盖跨度 ≥ 阈值 |
| 收敛 | 解抖动小于阈值时 streak 累加至 converged |
| Session 读写 | jsonl roundtrip；meta schema_version |
| math3d | xyzrpy ↔ T 往返 |
| IK 点动（可选，无硬件） | dry_run 下 cart_jog 返回有限关节 |

硬件集成测试（手工）：M1–M7 验收清单按需求 §10。

---

## 13. 实现顺序映射（对需求里程碑）

| 里程碑 | 详细设计交付切片 |
|---|---|
| M1 | 工程骨架、Config、RealSenseAdapter、CameraWorker、主窗预览/内参 |
| M2 | EliteArmAdapter、KinematicsBridge、ArmPoll、点动 UI、tcp_xyz |
| M3 | RecordService、SessionStore、VideoIO |
| M4 | aruco + handeye + 导出 + 合成单测 |
| M5 | LiveCalibService、CalibWorker、收敛面板 |
| M6 | filter_select、Offline 对话框 |
| M7 | verify_geom、Verify 对话框、验收 |

---

## 14. 开放实现抉择（已给默认，实现时可改但需改文档）

| 项 | 默认 |
|---|---|
| PyQt 版本 | **PyQt6**（若 Win 环境缺 wheel 则退 PyQt5，API 用薄兼容层） |
| 笛卡尔增量 | 基座系平移 + 欧拉角叠加 |
| 实时预览是否强制检测 | 是（低成本）；无板不入样本 |
| HandEye 符号 | 以合成单测钉死，不信任旧脚本 |

---

## 15. 修订记录

| 版本 | 说明 |
|---|---|
| v1.0 | 首版详细设计：包结构、配置、算法契约、适配器、状态机、schema、线程 UI、测试与里程碑切片 |
