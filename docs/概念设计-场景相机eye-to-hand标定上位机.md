# 场景相机 Eye-to-Hand 标定上位机 — 概念设计

| 项 | 内容 |
|---|---|
| 文档版本 | **v1.0** |
| 日期 | 2026-10-20 |
| 依据 | `需求报告-场景相机eye-to-hand标定上位机.md` **v1.1** |
| 后续文档 | `详细设计-场景相机eye-to-hand标定上位机.md` |
| 范围 | 单路场景相机外参标定；Windows + PyQt；依赖 `sensors` SDK |

---

## 1. 设计目标与原则

### 1.1 目标

将「扫相机 → 取内参 → 连臂点动 → 连续录制 → 实时/离线 HandEye → 可视化验证」做成可交付的 Windows 上位机，使操作员无需阅读脚本即可完成 eye-to-hand 外参标定。

### 1.2 设计原则

| 原则 | 含义 |
|---|---|
| **单一真相源** | 坐标系、字段语义、导出外参以需求报告为准；旧 `hik_gello` 脚本不作为正确性依据 |
| **薄封装 sensors** | 相机/臂/运动学一律经 `sensors`；本仓库只做编排、算法、UI、落盘 |
| **UI 与硬件解耦** | 服务层可单测、可 dry-run；PyQt 只订阅状态与发命令 |
| **取流不被算法阻塞** | 预览/录制在采集路径；HandEye 在计算路径；跨线程用队列/信号 |
| **安全默认可控** | 臂默认未 Arm；点动有步进与软限位；随时 Stop/Disarm |
| **可复现** | 每次 session 带完整 meta；外参导出自描述 |

### 1.3 非目标（概念层再次划界）

- 多相机、eye-in-hand、内参重标定、自动轨迹规划  
- 第二套 RealSense / Elite 驱动  
- 兼容旧 pickle 数据  

---

## 2. 系统上下文

```
                 ┌──────────────────────────────────────┐
                 │     rs-calibration 上位机 (PyQt)      │
                 │  UI · Session · Calib · Verify · Jog  │
                 └───────┬──────────────────┬───────────┘
                         │                  │
              ┌──────────▼──────────┐  ┌────▼─────────────────┐
              │ sensors RealSense   │  │ sensors Elite EC616  │
              │ 枚举 / 开流 / 内参   │  │ arm_write + monitor  │
              └──────────┬──────────┘  └────┬─────────────────┘
                         │                  │
                   USB RealSense      以太网 IP:54321 / 8056
                         │                  │
                   场景固定相机           EC616 控制器
                                            │
                                   sensors.kinematics
                                   (法兰 FK / IK)
```

**外部参与者：**

- 操作员：选相机、输 IP、Arm、点动、录制、看收敛、离线算、验证  
- 现场硬件：1 台 RealSense、1 台 EC616、末端固定 ArUco 板  

---

## 3. 核心概念模型

### 3.1 领域对象

| 概念 | 说明 |
|---|---|
| **CameraDevice** | 一台可枚举的 RealSense（serial、name、firmware） |
| **Intrinsics** | 当前流下的 color 内参 + 畸变 + 分辨率/fps（UI 可覆盖） |
| **ArmConnection** | 由 IP 建立的 EC 连接；状态：断开 / 已连接 / Armed |
| **FlangePose** | \(T_{\mathrm{base}\leftarrow\mathrm{flange}}\)，由 joints + FK 得到；**即记录中的 pose** |
| **TcpConfig** | `tcp_xyz`，仅配置/验证用，不写入 pose 字段 |
| **MarkerObservation** | 单帧 ArUco 检测得到的 \(T_{\mathrm{cam}\leftarrow\mathrm{marker}}\) |
| **CalibSample** | 一对有效样本：`(T_base_flange, T_cam_marker)`（+ frame 索引） |
| **Extrinsics** | 主结果 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\) 及其逆、方法、统计 |
| **Session** | 一次录制目录：`meta` + `video` + `joints_poses` + 可选 `extrinsics` |
| **ConvergenceState** | 样本数、Δt、ΔR、是否收敛、稳定计数 |

### 3.2 坐标系关系（概念）

```
{base} ──T_base_flange──► {flange} ──tcp_xyz──► {tcp}
  ▲                           │
  │                           └──T_flange_marker──► {marker}  (可选，仅验证 B)
  │
  └── T_base_cam ◄── HandEye 主结果 ── {cam} ──T_cam_marker──► {marker}
```

- **求解用：** `T_base_flange` + `T_cam_marker` → `T_base_cam`  
- **验证主误差：** 预测板 vs 检测板（可不依赖 `T_flange_marker`）  
- **验证辅观感：** TCP 轴 A（运动学+外参）vs B（检测+安装+tcp）

---

## 4. 逻辑架构（分层）

```
┌─────────────────────────────────────────────────────────┐
│  Presentation：PyQt MainWindow + 分区面板 / 对话框         │
│  （信号槽；不直接碰 pyrealsense2 / elite）                 │
├─────────────────────────────────────────────────────────┤
│  Application：用例编排                                    │
│  ScanCameras · ConnectArm · Jog · Record · LiveCalib     │
│  OfflineCalib · Verify · ExportExtrinsics                │
├─────────────────────────────────────────────────────────┤
│  Domain：纯算法 / 纯数据                                   │
│  ArucoDetect · HandEyeSolver · SampleFilter · Convergence│
│  VerifyGeometry · PoseMath                               │
├─────────────────────────────────────────────────────────┤
│  Infrastructure：对 sensors 的适配 + 落盘                  │
│  RealSenseAdapter · EliteArmAdapter · SessionStore       │
│  VideoWriter · ConfigStore                               │
└─────────────────────────────────────────────────────────┘
```

**依赖方向：** Presentation → Application → Domain；Infrastructure 被 Application 注入。Domain **零依赖** PyQt / sensors。

---

## 5. 运行时概念：线程与数据流

### 5.1 线程角色（概念）

| 线程 | 职责 | 约束 |
|---|---|---|
| **UI 线程** | 控件、状态栏、参数编辑 | 禁止重计算、禁止阻塞 IO |
| **Camera 线程** | 取帧、预览推送、录制写视频 | 固定目标 FPS；队列有界丢旧 |
| **Arm 轮询线程** | 周期 `read()` joints → FK → 状态广播 | 与写命令同适配器内锁（沿用 sensors） |
| **Jog 路径** | UI 点击 → Application → `arm_write` | 短调用；失败回传错误字符串 |
| **Calib 线程** | 收样本、HandEye、收敛判别 | 只吃队列；结果用信号回 UI |
| **Offline/Verify 工作线程** | 读文件、批处理 | 进度信号；可取消 |

### 5.2 主数据流：在线录制 + 可选实时标定

```
Camera thread          Arm poll              Calib thread
    │                     │                       │
    │ frame(ts,img)       │ joints(ts)            │
    ▼                     ▼                       │
  RecordMux ──对齐──► Sample(frame_idx, img, joints, T_flange)
    │                     │                       │
    ├──► VideoWriter      │                       │
    ├──► joints_poses.jsonl                       │
    └──► (若实时计算) Detect ArUco ──有效──► CalibSample ──► HandEye
                                                      │
                                                      ▼
                                               Convergence → UI
```

对齐策略（概念）：以**相机帧**为时钟；每帧取臂侧「最近一次 joints 快照」（插值非必须；记录 `arm_ts` 与 `cam_ts` 差值供质检）。

### 5.3 离线 / 验证数据流

```
Session 文件 ──► OfflinePipeline
                  ├─ 解码视频帧
                  ├─ 读 jsonl 法兰位姿
                  ├─ ArUco + Filter + Select
                  ├─ HandEye + 报告
                  └─ 可选写 extrinsics.json

Session + Extrinsics ──► VerifyPlayer
                  ├─ 按帧叠加轴 A/B
                  ├─ 主误差：预测板 vs 检测板
                  └─ 误差曲线 / 导出
```

---

## 6. 功能模块划分（概念）

| 模块 | 用户可见能力 | 关键内部能力 |
|---|---|---|
| **M-Cam** | 扫描列表、选择、开流预览、内参回填/编辑 | RealSense 枚举与 pipeline |
| **M-Arm** | IP 连接反馈、Arm/Disarm/Stop、关节+笛卡尔点动、状态显示 | EliteArmWrite + FK/IK |
| **M-Cfg** | tcp_xyz、ArUco、收敛参数、步进量 | 读写本地 yaml/json |
| **M-Rec** | 开始/停止录制、session 路径提示 | 视频+jsonl+meta 原子落盘 |
| **M-Live** | 实时计算开关、收敛仪表 | 样本积累 + HandEye + 收敛状态机 |
| **M-Off** | 选文件、跑智能计算、看统计与曲线 | 滤波选点 + 多方法对比 |
| **M-Ver** | 选视频/记录/外参、播放叠加 | 几何投影 + 主误差 |

模块间通过 Application 用例交互，避免面板直接互相调用硬件。

---

## 7. UI 信息架构（概念布局）

单主窗口，**左侧导航 / 顶部分段**均可；推荐 **一屏工作台 + 右侧状态**：

```
┌──────────────┬────────────────────────────┬────────────┐
│ 相机         │                            │ 臂状态     │
│ 列表/刷新    │      实时预览 / 回放区       │ 连接/Armed │
│ 内参表单     │                            │ joints     │
├──────────────┤                            │ 法兰 xyzrpy│
│ 臂 IP/连接   │                            │ TCP(只读)  │
│ Arm/Stop     │                            ├────────────┤
│ 关节点动     │                            │ 实时标定   │
│ 笛卡尔点动   │                            │ 样本/收敛  │
├──────────────┤                            │ Δt ΔR      │
│ 录制控制     │                            │ 导出外参   │
│ 实时计算☑    │                            ├────────────┤
│ 参数…        │                            │ 日志/告警  │
└──────────────┴────────────────────────────┴────────────┘
  菜单/工具：离线计算…  验证…  打开 session…  关于
```

- **离线计算 / 验证** 用独立对话框或堆叠页，避免挤占在线工作台。  
- 中文为主；危险操作（Arm、开始录制覆盖）二次确认可选。

---

## 8. 关键用例（概念级）

### UC1 准备相机
扫描 → 选择 → 开流 → 内参回填 → 预览可见 ArUco。

### UC2 准备机械臂
输入 IP → 连接（成败反馈）→ 配置 tcp_xyz → Arm → 关节点动与笛卡尔点动验证 → 需要时 Disarm/Stop。

### UC3 在线标定采集
开始录制 → 操作员点动覆盖位姿 → （可选）勾选实时计算观察收敛 → 停止录制 → 导出/保存外参。

### UC4 离线智能重算
选择 session 视频与 jsonl → 运行滤波选点 → 查看外参与统计 → 保存 `extrinsics.json`。

### UC5 验证
加载视频、jsonl、外参 → 播放 → 看主误差与双轴 → 判断是否验收通过。

---

## 9. 算法概念管线

### 9.1 检测

灰度 → ArUco `DICT_5X5_50` → ID=0 → `estimatePoseSingleMarkers`(边长 0.15 m, K, dist) → `T_cam_marker`。

### 9.2 HandEye（标准 eye-to-hand）

输入有效样本列表 → 按 OpenCV 文档组装 `R_gripper2base, t_*, R_target2cam, t_*` → `calibrateHandEye` → 归一化为导出的 \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\)。  
**不读** `T_flange_marker` / `tcp_xyz`。

### 9.3 智能滤波与选点（概念策略）

1. 剔除：无检测、位姿跳变异常、近重复（关节/法兰阈值）  
2. 多样性：在 SE(3)/关节空间做覆盖选点（贪心最远点或体素网格）  
3. 求解：主方法 TSAI；可选多种方法输出一致性告警  
4. 收敛：与在线同一套阈值参数  

### 9.4 验证几何（概念）

- **主误差：** \(\mathrm{pred}(T_{\mathrm{cam}\leftarrow\mathrm{marker}})\) vs 检测  
- **辅：** TCP 轴 A / B 叠加  

---

## 10. 配置与持久化概念

| 存储 | 内容 |
|---|---|
| 用户配置 `config.yaml` | 默认 IP、tcp_xyz、流分辨率/fps、点动步进、收敛参数、ArUco、上次 session 根目录 |
| Session 目录 | 需求报告 §5 结构 |
| 外参文件 | `extrinsics.json`（矩阵 + 元数据） |

---

## 11. 依赖与部署概念

```
rs-calibration/
  docs/                 # 本设计与需求
  src/rs_calib/         # 应用代码（实现阶段）
  vendor/sensors ->     # 软链至 sensors 包源码或 site-packages
  configs/default.yaml
  README.md
```

**运行时依赖：** Python 3.10+、PyQt5/6、OpenCV、NumPy、SciPy、`pyrealsense2`、`sensors`（含 `elite`/`elirobots`）、Windows x64。

---

## 12. 质量与风险（概念）

| 风险 | 缓解 |
|---|---|
| OpenCV eye-to-hand 输入约定易反 | Domain 单测：合成刚体外参，断言恢复误差 |
| 相机/臂时间不同步 | 记录双时间戳；验证看误差抖动 |
| HandEye 阻塞 UI | 独立线程 + 有界队列 |
| 点动超限 | 完全依赖 sensors guard；UI 显示 error |
| Win 视频编码失败 | mp4 失败自动降级 png 序列，meta 标明 |
| 板安装偏置未知 | 主误差不依赖 `T_flange_marker`；文档提示 |

---

## 13. 与详细设计的边界

本文回答 **系统长什么样、模块怎么分、数据怎么流、原则是什么**。  
下列内容放入详细设计：

- 包/类/接口签名、状态机转移表  
- 线程与 Qt 信号精确表、队列容量  
- 文件 JSON schema、配置项清单  
- UI 控件级说明、命令映射到 `arm_write`  
- HandEye 输入变换伪代码、滤波算法伪代码  
- 错误码与日志规范  

---

## 14. 修订记录

| 版本 | 说明 |
|---|---|
| v1.0 | 首版概念设计，对齐需求 v1.1 |
