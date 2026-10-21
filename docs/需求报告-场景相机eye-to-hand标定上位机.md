# 场景相机 Eye-to-Hand 外参标定上位机 — 需求报告

| 项 | 内容 |
|---|---|
| 文档版本 | **v1.1**（小项已闭环） |
| 日期 | 2026-10-20 |
| 项目目录 | `/root/autodl-tmp/rs-calibration` |
| 阶段目标 | **单路**场景相机（固定相机）**外参**标定（eye-to-hand） |
| 目标平台 | **Windows**，UI：**PyQt** |
| 算法参考 | `hww/hik_gello/eye_to_hand_calib.py`（**仅参考，不保证正确**；实现按标准手眼流程） |
| 唯一运行时依赖 | `sensors` SDK（本仓库通过**软链**引入，不复制驱动源码） |

---

## 0. 已确认产品决策（摘要）

| # | 决策 |
|---|---|
| 1 | 只扫描 RealSense；机械臂 **手输 IP → 手动连接**，连接成功/失败有明确反馈 |
| 2 | Windows + PyQt |
| 3 | 记录中的 **pose = 法兰盘中心**位姿；**tcp_xyz** 单独提供配置接口（用于验证与可视化，不混入 pose 字段语义） |
| 4 | 旧标定代码仅作思路参考；数据记录与求解走**标准 eye-to-hand** 约定并在本文档写死 |
| 5 | 数据格式**无需**兼容旧 pickle |
| 6 | 收敛条件**可配置**；本文给出推荐初值 |
| 7 | 验证色轴 A/B 定义见 §6（本报告给定） |
| 8 | **需要软件点动**（关节 + 笛卡尔都要）；录制是一段时间连续数据，与运动方式无关，由上位机控臂 |
| 9 | 只能依赖 `sensors`；可用软链 |
| 10 | 本期严格**单相机** |
| 11 | ArUco：`DICT_5X5_50` / ID `0` / 边长 `0.15 m` |
| 12 | 视频优先 `mp4`，失败降级 png 序列 |
| 13 | **`T_flange_marker` 不进入 HandEye 主求解**（见 §3.5）；仅影响验证色轴 B 的几何对齐 |

---

## 1. 背景与目标

在 **RealSense 场景相机 + Elite EC616** 上，提供 Windows PyQt 上位机，完成场景相机相对机器人基座的 **eye-to-hand 外参**标定，覆盖：

1. 扫描并选择 RealSense  
2. SDK 读取内参并回填  
3. 连接机械臂（IP）并软件点动；开始/停止录制（图像 + joints + 法兰 pose）  
4. 可选实时解算外参并显示收敛  
5. 离线智能滤波与重算  
6. 用双色坐标系验证外参  

**本期不做**：多相机并行、腕部 eye-in-hand、相机内参重标定、自动轨迹规划采点（点动由操作员完成）。

---

## 2. 依赖与工程约定

### 2.1 sensors 软链

- 本仓库仅依赖 `sensors` 包中的 SDK/驱动能力，例如：  
  - `sensors.drivers.camera.realsense.RealSenseSensor`（或同等枚举/开流/内参 API）  
  - `sensors.drivers.arm.elite_write.EliteArmWriteSensor`（连接、armed 门控、点动/写关节；读状态同源 monitor）  
  - `sensors.kinematics`（EC616 FK/IK，法兰系）  
- 推荐：在 `rs-calibration/vendor/sensors` → `/root/autodl-tmp/sensors/src/sensors`（或安装 editable）做软链 / `PYTHONPATH`，**不 fork 驱动逻辑**。  
- Windows 实机部署时同样只安装/引用 `sensors` 发行物，不内嵌第二份 Elite/RealSense 封装。

### 2.2 安全与控臂

- 控臂走 `arm_write` 契约：`open` 仅 monitor；显式 **Arm/Initialize** 后才允许 `TT_add_joint`；提供 **Disarm / Stop**。  
- 点动受 `sensors` 软限位与单步 `max_delta` 约束；UI 需显示 armed 状态。  
- 录制与点动可并行：录制线程只采样，不改变运动逻辑。

---

## 3. 坐标系与标准记录约定

### 3.1 坐标系

| 符号 | 含义 |
|---|---|
| `{base}` | 机器人基座 |
| `{flange}` | 法兰盘中心（机械接口） |
| `{tcp}` | 工具中心：`T_flange_tcp` 由配置 `tcp_xyz` 生成（默认仅平移，姿态与法兰一致） |
| `{cam}` | 场景相机光学/彩色坐标系（与 RealSense color 内参一致） |
| `{marker}` | 标定板（ArUco）坐标系 |

### 3.2 字段语义（冻结）

| 字段 | 语义 | 单位 |
|---|---|---|
| `joints` | 6 轴关节角 | 同时存 `joints_rad`（主）与可选 `joints_deg` |
| `pose` / `T_base_flange` | **法兰中心**相对基座的位姿 | 齐次 4×4；另可存 `xyzrpy`（m, rad，XYZ 外旋，与 sensors FK 一致） |
| `tcp_xyz` | 法兰→TCP 平移配置 `[x,y,z]` | m；**不写入每帧 pose**，写入 `meta` |
| `T_cam_marker` | 标定板在相机系下的位姿 | 检测得到，可不落盘到 joints 文件（实时/离线现算） |

**禁止**：把 TCP 位姿写进名为 `pose` 的字段，避免与「法兰 pose」混淆。

### 3.3 标准 eye-to-hand 求解约定

采用 OpenCV `cv2.calibrateHandEye` 的 **eye-to-hand** 标准用法（实现时以 OpenCV 文档为准，并在代码注释写明输入是法兰还是 gripper）：

**每帧有效样本：**

1. \(R_{\mathrm{flange2base}},\, t_{\mathrm{flange2base}}\) ← 由 `T_base_flange` 取旋转/平移（注意 OpenCV 对 eye-to-hand 要求的是 gripper→base 或 base→gripper 的约定，实现层统一转换，导出时只公布一种）  
2. \(R_{\mathrm{target2cam}},\, t_{\mathrm{target2cam}}\) ← ArUco 估计的 `T_cam_marker`

**求解输出（导出冻结）：**

- 主结果：\(T_{\mathrm{base}\leftarrow\mathrm{cam}}\)（4×4），即相机在基座系下的位姿  
- 同时导出其逆 \(T_{\mathrm{cam}\leftarrow\mathrm{base}}\)  
- 导出 JSON/YAML：矩阵 + 四元数/xyzrpy + 方法名 + 样本数 + 收敛指标  

默认方法：`CALIB_HAND_EYE_TSAI`；UI 可选 PARK / HORAUD / ANDREFF / DANIILIDIS 作对比（不阻塞主流程）。

> 旧 `hik_gello` 脚本中的求逆/变量命名**不作为正确性依据**；以本节与 OpenCV 文档 + 单元测试（合成数据）为准。

### 3.4 tcp_xyz 配置接口

- UI / 配置文件可编辑 `tcp_xyz: [x, y, z]`（米）  
- 推荐默认：`[0.0, 0.0, 0.18]`（与 sensors-arm/DCS 常见默认一致，可改）  
- 用途：验证叠加、误差统计、预览；**HandEye 主求解使用法兰样本**（标准），避免把未标定工具偏置吃进外参  

### 3.5 标定板参数（已确认）

| 参数 | 取值 | 说明 |
|---|---|---|
| 字典 | `DICT_5X5_50` | 已确认 |
| Marker ID | `0` | 已确认 |
| 边长 | `0.150` m | 已确认 |
| `T_flange_marker` | 可选配置，默认单位阵 | **见下方算法说明** |

#### `T_flange_marker` 与算法的关系（结论：主求解不受影响）

Eye-to-hand 的 `calibrateHandEye` 每帧只用：

- 机器人给出的 **法兰（或夹具）相对基座** 位姿  
- 相机观测到的 **标定板相对相机** 位姿  

板刚性地装在末端上时，法兰→板 的固定偏置会体现在观测数据里，但在标准 AX=XB / OpenCV HandEye 公式中会被吸收，**求解得到的仍是相机↔基座外参**，**不需要事先知道 `T_flange_marker`**，填错或当成单位阵也**不会改变**主外参结果（只要安装在标定过程中保持刚性、不变）。

它**会影响**的只有：

1. **验证色轴 B**（用「板检测 → 推 TCP」时，需要板相对法兰的安装关系，偏置不准则 A/B 对不齐，但外参文件本身仍可正确）  
2. 若有人误把「板系」当成「法兰系」做别的下游应用  

因此：主流程 **不强制** 测量 `T_flange_marker`；UI 可保留高级项（默认单位阵），供验证可视化微调。HandEye / 实时 / 离线导出外参链路 **不读取该偏置参与求解**。

---

## 4. 功能需求

### 4.1 相机：扫描 / 选择 / 内参

| ID | 需求 |
|---|---|
| F1.1 | 扫描本机 RealSense，列表显示：序列号、名称/型号、（可得时）固件 |
| F1.2 | 单选一台作为当前相机；支持刷新 |
| F1.3 | 选定后开流，读取 **当前分辨率/fps** 下的 color 内参并回填：\(f_x,f_y,c_x,c_y\)、畸变系数、宽高、fps |
| F1.4 | 内参可编辑 +「重新读取」；后续检测/标定/验证一律用 UI 当前值 |
| F1.5 | SDK 缺失、无设备、设备占用失败时 UI 明确提示 |

### 4.2 机械臂：IP 连接 + 反馈 + 点动

| ID | 需求 |
|---|---|
| F2.1 | 输入机器人 IP（默认端口 54321 / monitor 8056 按 sensors 默认），点击「连接」 |
| F2.2 | 连接反馈：成功（可显示当前 joints）、失败（超时/SDK 缺失/端口不通）、断开 |
| F2.3 | 状态区：连接状态、armed、急停/Disarm |
| F2.4 | **软件点动（都必须有）**：**关节点动**（± 步进可配）+ **笛卡尔点动**（`sensors.kinematics` IK；UI 可切换相对法兰/TCP 显示，下发仍为关节） |
| F2.5 | 点动前必须 Arm；提供 Stop/Disarm；遵守 sensors 单步角增量与软限位 |
| F2.6 | 实时显示：joints、法兰 pose（xyzrpy）、可选 TCP pose（由 tcp_xyz 推算，只读展示） |

### 4.3 录制（一段时间连续数据）

| ID | 需求 |
|---|---|
| F3.1 | 「开始录制 / 停止录制」；录制中预览画面 |
| F3.2 | 录制内容与机械臂如何运动无关：操作员可点动或外部动臂，上位机只按时序抽样 |
| F3.3 | 落盘目录示例见 §5；视频与 `joints_poses` 用时间戳/`frame_idx` 对齐 |
| F3.4 | `meta.json` 写入：相机序列号、内参、流配置、机器人 IP、tcp_xyz、ArUco 参数、可选 `T_flange_marker`（仅供验证）、软件版本等 |
| F3.5 | 录制帧率目标与相机流一致（推荐 15 或 30 FPS，可配） |

### 4.4 实时计算 + 收敛

| ID | 需求 |
|---|---|
| F4.1 | Checkbox「实时计算」；勾选后在录制（及可选预览）中，对有效 ArUco 帧积累样本并周期性 HandEye |
| F4.2 | 显示：有效样本数、当前 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\)、相对上次的 \(\Delta t\)、\(\Delta R\)、是否收敛 |
| F4.3 | 收敛参数全部可配置（§7）；达到收敛时高亮提示，可一键导出当前外参 |
| F4.4 | 解算不得阻塞取流（独立线程/队列） |

### 4.5 离线智能计算

| ID | 需求 |
|---|---|
| F5.1 | 选择：视频 + `joints_poses`（+ 可选 meta；无 meta 则用 UI 内参/ArUco/tcp 配置） |
| F5.2 | 滤波：丢检帧、位姿跳变异常、近重复位姿（关节或法兰 SE(3) 阈值可配）、姿态多样性不足的冗余帧 |
| F5.3 | 选点：在过滤后按位姿覆盖选子集，使外参尽快稳定；可对多种 HandEye 方法做一致性检查 |
| F5.4 | 输出外参、样本索引、过滤统计、收敛曲线；可保存结果文件 |

### 4.6 验证（双色坐标轴）

见 §6。

---

## 5. 数据格式（新设计，不兼容旧 pickle）

```
session_YYYYMMDD_HHMMSS/
  meta.json
  video.mp4                 # color，Windows 可用 mp4v/avc1；失败则降级为 png 序列
  joints_poses.jsonl        # 一行一帧采样
  extrinsics.json           # 可选，标定成功后写入
```

**`joints_poses.jsonl` 单行示例：**

```json
{
  "ts": 1730000000.123,
  "frame_idx": 42,
  "joints_rad": [0.1, -0.2, 0.3, 0.0, 1.2, 0.0],
  "T_base_flange": [[...4x4...]],
  "pose_xyzrpy": [0.4, 0.0, 0.3, 0.0, 1.57, 0.0]
}
```

说明：`pose_xyzrpy` 与 `T_base_flange` 均表示**法兰**；二者应一致（由 FK 生成）。

---

## 6. 验证：颜色 A / 颜色 B（本报告定义）

验证输入：视频 + `joints_poses` + 外参 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\)（或逆）+ 内参 + `tcp_xyz` + `T_flange_marker`。

对每一帧构造 **同一物理对象：TCP 坐标系** 在相机图像中的投影，用两色小坐标轴（轴长可配，默认 50 mm）：

### 6.1 颜色 A — 机械臂参考（机器人运动学链）

\[
T_{\mathrm{cam}\leftarrow\mathrm{tcp}}^{(A)}
= T_{\mathrm{cam}\leftarrow\mathrm{base}}
\cdot T_{\mathrm{base}\leftarrow\mathrm{flange}}
\cdot T_{\mathrm{flange}\leftarrow\mathrm{tcp}}(\texttt{tcp\_xyz})
\]

其中 \(T_{\mathrm{base}\leftarrow\mathrm{flange}}\) 来自该帧记录的法兰 pose。  
**语义**：不依赖本次视觉检测，只依赖记录关节/法兰 + 外参 + tcp 配置 → **参考真值链**。

**建议绘制颜色**：X 红 / Y 绿 / Z 蓝，线宽较粗或实线（经典 RGB）。

### 6.2 颜色 B — 标定效果（视觉检测链）

\[
T_{\mathrm{cam}\leftarrow\mathrm{tcp}}^{(B)}
= T_{\mathrm{cam}\leftarrow\mathrm{marker}}
\cdot T_{\mathrm{marker}\leftarrow\mathrm{flange}}
\cdot T_{\mathrm{flange}\leftarrow\mathrm{tcp}}(\texttt{tcp\_xyz})
\]

其中 \(T_{\mathrm{cam}\leftarrow\mathrm{marker}}\) 为该帧 ArUco 检测；\(T_{\mathrm{marker}\leftarrow\mathrm{flange}} = (T_{\mathrm{flange}\leftarrow\mathrm{marker}})^{-1}\)。  

**语义说明：**

- 该式**本身不含外参**；A 含外参、B 含板检测。二者对齐好，说明外参与同步大致正确。  
- 若 `T_flange_marker` 未知而用单位阵，则 B 画的是「把板系当成法兰系再加 tcp」的近似 TCP；**A/B 可能系统性错开一块，但不代表 HandEye 外参算错**。  
- 因此验证 UI 应注明：主外参评价也可看「用法兰+外参投影的预测板位姿」与「ArUco 检测板位姿」的误差（该对比**不依赖** `T_flange_marker`）——实现时把该指标作为**默认主误差**，A/B TCP 双轴为辅助观感。

**默认主误差（推荐实现，与板安装偏置无关）：**

\[
T_{\mathrm{cam}\leftarrow\mathrm{marker}}^{\mathrm{pred}}
= T_{\mathrm{cam}\leftarrow\mathrm{base}}
\cdot T_{\mathrm{base}\leftarrow\mathrm{flange}}
\cdot T_{\mathrm{flange}\leftarrow\mathrm{marker}}
\]

当 `T_flange_marker = I` 时，即用法兰当板系原点做预测，与检测板比较；若板相对法兰有未知固定偏置，相对运动仍一致，绝对轴可能有常值偏差，但不妨碍看外参是否随帧稳定。

**建议绘制颜色**：青 / 品红 / 黄（或整架青色），与 A 的 RGB 明显区分；图例写清 A/B。

### 6.3 播放与 UI

- 播放 / 暂停 / 拖动；无板帧只画 A 或跳过 B  
- 显示当前帧 \(\Delta t\)（mm）、\(\Delta R\)（deg）  
- 可导出误差曲线  

---

## 7. 收敛条件（可配置 + 推荐初值）

实时与离线共用同一套参数（离线可对「全量选点后的最终解」再跑一遍稳定判据）。

| 参数 | 推荐初值 | 含义 |
|---|---|---|
| `min_samples` | **12** | 最少有效样本才开始判收敛 |
| `recompute_every_n` | **1** | 每新增 N 个有效样本重算一次（也可改时间节流） |
| `max_delta_t_mm` | **1.5** | 相邻两次解的平移变化阈值 |
| `max_delta_R_deg` | **0.5** | 相邻两次解的旋转角变化阈值 |
| `stable_count` | **5** | 连续多少次重算均满足上述阈值 → **已收敛** |
| `min_translation_span_m` | **0.15** | 样本法兰平移覆盖的最小空间尺度（过小警告） |
| `min_rotation_span_deg` | **25** | 样本姿态变化覆盖（过小警告） |
| `dup_joint_eps_deg` | **0.3** | 关节近重复剔除阈值（离线/实时采样） |
| `dup_pose_eps_mm` / `dup_pose_eps_deg` | **3.0** / **1.0** | 法兰位姿近重复剔除 |
| `outlier_reproj_px` | **5.0** | （可选）板重投影/帧间跳变异常阈值 |

UI：参数面板可改；「恢复推荐值」按钮。

---

## 8. 非功能需求

| ID | 要求 |
|---|---|
| N1 | 目标机 Windows；开发可用 Linux，但交付以 Win + PyQt 为准 |
| N2 | 预览/录制不因 HandEye 卡顿；点动命令低延迟 |
| N3 | 关键结果可复现：meta + 版本号 + 随机种子（若有） |
| N4 | 中文 UI 为主（可后续双语） |
| N5 | 异常可恢复：相机掉线、臂断连、检测失败均有提示，不静默崩溃 |

---

## 9. 逻辑架构

```
┌──────────────────────── PyQt UI ────────────────────────┐
│ 相机列表/内参 │ 臂 IP 连接/Arm │ 点动 │ 录制 │ 实时/离线 │ 验证 │
└───────┬───────────────┬───────────────┬─────────────────┘
        │               │               │
   CameraSvc      ArmWriteSvc      Calib/Filter/Verify
   (sensors RS)   (sensors elite   (OpenCV HandEye +
                   write+kinematics)  本仓库算法)
```

---

## 10. 验收标准

1. 仅列出 RealSense；选中后内参回填正确。  
2. 输入 IP 连接 EC616：成功/失败有反馈；Arm 后可关节（及笛卡尔）点动与 Stop。  
3. 录制得到 `video` + `joints_poses.jsonl` + `meta.json`；`pose` 为法兰；`tcp_xyz` 仅在 meta/配置。  
4. 实时计算可显示样本数与收敛状态；参数可改且「推荐值」可用。  
5. 离线智能计算能剔除重复/异常并输出外参与统计。  
6. 验证：同帧叠加 **A（运动学+外参→TCP）** 与 **B（ArUco+安装+tcp→TCP）** 两色轴，并显示误差。  
7. 依赖路径仅通过 `sensors`（软链/安装），无第二套 Elite/RS 驱动。

---

## 11. 里程碑

| 阶段 | 内容 |
|---|---|
| M1 | 工程骨架 + sensors 软链 + PyQt 壳 + RS 扫描/内参/预览 |
| M2 | 臂 IP 连接反馈 + Arm/点动/Stop + 法兰 FK 显示 + tcp_xyz 配置 |
| M3 | 连续录制落盘（视频 + jsonl + meta） |
| M4 | ArUco + 标准 HandEye + 导出 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\) |
| M5 | 实时计算 + 可配置收敛 |
| M6 | 离线智能滤波选点 |
| M7 | 验证双轴 A/B + 验收 |

---

## 12. 小项闭环（原 §12）

| 项 | 结论 |
|---|---|
| ArUco | `DICT_5X5_50` / ID `0` / `0.15 m` |
| 点动 | 关节 + 笛卡尔都要 |
| 视频 | 优先 mp4，失败 → png 序列 |
| `T_flange_marker` | **不影响 HandEye 主外参**；不强制测量；验证以「预测板 vs 检测板」为主误差，TCP 双轴为辅 |

---

## 13. 修订记录

| 版本 | 说明 |
|---|---|
| v0.1 | 澄清稿 |
| v1.0 | 纳入 10 条产品决策；冻结法兰 pose / tcp_xyz / 导出外参 / A·B 验证定义 / 收敛推荐值 / Win·PyQt·sensors 软链 |
| v1.1 | 确认 ArUco/双点动/mp4；阐明 `T_flange_marker` 不进入主求解，并调整验证主误差定义 |
