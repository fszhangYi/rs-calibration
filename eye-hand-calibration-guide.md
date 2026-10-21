# Eye-in-Hand 与 Eye-to-Hand 手眼标定图文指南

面向机器人视觉与 Intel RealSense 腕部/场景相机部署的实用说明：先建立直观概念，再展开公式，最后给出物料清单与操作步骤。

---

## 1. 先建立概念：什么是“手眼标定”

机器人要“看见并动手”，必须把相机坐标系里的点，变换到机械臂基座（或工具）坐标系。手眼标定要做的，就是求出这一段**刚性外参**——相机相对末端，或相机相对基座的固定位姿。

- **手（Hand）**：机械臂末端法兰、夹爪或工具坐标系（End-Effector, EE）。
- **眼（Eye）**：相机坐标系（对 RealSense 通常取彩色光心或官方外参定义的相机系）。
- **标定板（Target / Board）**：棋盘格、ChArUco、AprilTag 等，用于从图像恢复板相对相机的位姿。

<p align="center">
  <img src="assets/transform_chain.png" alt="坐标变换链" width="90%"/>
</p>
<p align="center"><em>图：从基座到末端、相机、标定板的齐次变换链。</em></p>

直观口诀：

- **眼在手上（Eye-in-Hand）**：相机跟着手臂走，板子通常钉在桌上。
- **眼在手外（Eye-to-Hand）**：相机钉在场景里，板子通常夹在手上。

<p align="center">
  <img src="assets/choose_config.png" alt="配置选择流程图" width="85%"/>
</p>
<p align="center"><em>图：按相机安装位置选择 Eye-in-Hand 或 Eye-to-Hand。</em></p>

---

## 2. 两种配置对照

<p align="center">
  <img src="assets/compare_configs.png" alt="两种配置对比" width="90%"/>
</p>
<p align="center"><em>图：Eye-in-Hand 与 Eye-to-Hand 的安装与求解目标对比。</em></p>

| 项目 | Eye-in-Hand（腕部相机） | Eye-to-Hand（场景相机） |
| --- | --- | --- |
| 相机安装 | 刚性固定在法兰/腕部支架 | 刚性固定在支架、龙门或天花板 |
| 标定板位置 | 固定在工作台/夹具上 | 固定在末端或夹爪上 |
| 主要求解量 | 相机相对末端外参（见下式） | 先求手–板关系，再合成相机相对基座外参 |
| 典型用途 | 近距精定位、视觉伺服、抓取修正 | 全局粗定位、监控、多机协同 |
| RealSense 示例 | 腕部 D405 / D435 | 场景 D435 / D455 |

Eye-in-Hand 主要求解量：

<p align="center">
  <img src="assets/formulas/inline_X_eeTcam.png" alt="X = eeTcam" width="35%"/>
</p>

Eye-to-Hand 最终常得到：

<p align="center">
  <img src="assets/formulas/inline_baseTcam.png" alt="baseTcam" width="28%"/>
</p>

### 2.1 Eye-in-Hand：眼在手上

相机与末端刚性连接。机械臂运动时，相机相对末端不变，相对基座在变；桌面上的标定板相对基座不变。

<p align="center">
  <img src="assets/eye_in_hand.png" alt="Eye-in-Hand 示意" width="90%"/>
</p>
<p align="center"><em>图：腕部 RealSense 观察固定标定板，求解相机相对末端的外参 X。</em></p>

**闭环直觉**：从基座出发，经末端到相机再到标定板，应与“基座到标定板”的固定关系一致。相邻两次采样之间，末端相对运动 A 与标定板在相机中的相对运动 B，通过未知外参 X 耦合，得到经典方程：

<p align="center">
  <img src="assets/formulas/inline_axxb.png" alt="AX=XB" width="28%"/>
</p>

### 2.2 Eye-to-Hand：眼在手外

相机固定在场景中，标定板装在末端。手臂移动板子，固定相机从不同视角观察。

<p align="center">
  <img src="assets/eye_to_hand.png" alt="Eye-to-Hand 示意" width="90%"/>
</p>
<p align="center"><em>图：场景 RealSense 观察末端标定板，最终得到相机相对基座的外参。</em></p>

OpenCV 等库对 Eye-to-Hand 仍可化归为 AX=XB，但 A、B、X 的定义与 Eye-in-Hand 不同（见第 3 节）。得到手–板关系后，再与某次观测的板–相机位姿、末端–基座位姿合成，得到：

<p align="center">
  <img src="assets/formulas/inline_baseTcam.png" alt="baseTcam" width="28%"/>
</p>

---

## 3. 公式详解

### 3.1 齐次变换预备知识

三维刚体位姿用如下 4×4 齐次矩阵表示：

<p align="center">
  <img src="assets/formulas/eq_homogeneous_T.png" alt="齐次变换矩阵" width="75%"/>
</p>

记号

<p align="center">
  <img src="assets/formulas/inline_aTb.png" alt="aTb" width="18%"/>
</p>

表示把 b 系中的点变到 a 系：

<p align="center">
  <img src="assets/formulas/eq_point_transform.png" alt="点变换" width="45%"/>
</p>

链式法则：

<p align="center">
  <img src="assets/formulas/eq_chain_rule.png" alt="链式法则" width="45%"/>
</p>

逆变换：

<p align="center">
  <img src="assets/formulas/eq_inverse.png" alt="逆变换" width="80%"/>
</p>

机器人控制器通常直接给出末端相对基座位姿（正向运动学）：

<p align="center">
  <img src="assets/formulas/inline_baseTee.png" alt="baseTee" width="28%"/>
</p>

相机通过标定板检测给出标定板相对相机位姿：

<p align="center">
  <img src="assets/formulas/inline_camTtgt.png" alt="camTtgt" width="28%"/>
</p>

### 3.2 统一主方程：AX = XB

对相邻（或任意两帧）位姿 i 与 j，可构造相对运动，使未知刚性外参 X 满足：

<p align="center">
  <img src="assets/formulas/eq_axxb.png" alt="AX=XB" width="40%"/>
</p>

<p align="center">
  <img src="assets/axxb_meaning.png" alt="AX=XB 含义" width="90%"/>
</p>
<p align="center"><em>图：A 来自机器人侧相对运动，B 来自视觉侧相对运动，X 为待求刚性外参。</em></p>

几何含义：机器人侧走一段相对刚体变换 A，视觉侧走一段相对刚体变换 B；中间夹着的外参 X 在两种顺序下应一致，故 AX=XB。

单组 (A, B) 解不唯一。实践中采集不少于约 10 组位姿（建议 15–30），得到多组相对运动对，再最小二乘或闭式方法求 X。旋转轴尽量不全平行，平移与姿态变化要充分。

<p align="center">
  <img src="assets/formulas/inline_AkBk.png" alt="多组AkBk" width="35%"/>
</p>

### 3.3 Eye-in-Hand 的 A、B、X

令：

- 第 i 次采样时末端相对基座（机器人 API 已知）：

<p align="center">
  <img src="assets/formulas/inline_baseTee.png" alt="baseTee(i)" width="28%"/>
</p>

- 第 i 次采样时标定板相对相机（视觉已知）：

<p align="center">
  <img src="assets/formulas/inline_camTtgt.png" alt="camTtgt(i)" width="28%"/>
</p>

- 待求：相机相对末端（安装后近似不变）：

<p align="center">
  <img src="assets/formulas/inline_X_eeTcam.png" alt="X=eeTcam" width="35%"/>
</p>

相对运动常用定义（与 OpenCV `calibrateHandEye` 的 Eye-in-Hand 约定一致）：

<p align="center">
  <img src="assets/formulas/eq_eih_A.png" alt="Eye-in-Hand A" width="70%"/>
</p>

<p align="center">
  <img src="assets/formulas/eq_eih_B.png" alt="Eye-in-Hand B" width="70%"/>
</p>

则：

<p align="center">
  <img src="assets/formulas/eq_eih_AX.png" alt="Eye-in-Hand AX=XB" width="65%"/>
</p>

部署时，把相机点变到基座：

<p align="center">
  <img src="assets/formulas/eq_eih_deploy.png" alt="部署变换" width="90%"/>
</p>

若夹爪相对法兰还有固定偏置

<p align="center">
  <img src="assets/formulas/inline_eeTgripper.png" alt="eeTgripper" width="30%"/>
</p>

再左乘或右乘该偏置即可。

### 3.4 Eye-to-Hand 的 A、B、X

令标定板刚性固连在末端，手–板外参近似不变：

<p align="center">
  <img src="assets/formulas/inline_eeTtgt.png" alt="eeTtgt" width="28%"/>
</p>

OpenCV 对 Eye-to-Hand 的常见约定是：

<p align="center">
  <img src="assets/formulas/eq_eth_A.png" alt="Eye-to-Hand A" width="70%"/>
</p>

<p align="center">
  <img src="assets/formulas/eq_eth_B.png" alt="Eye-to-Hand B" width="70%"/>
</p>

<p align="center">
  <img src="assets/formulas/eq_eth_AX.png" alt="Eye-to-Hand AX=XB" width="65%"/>
</p>

部分文献先求手–板外参 X，再对某一帧合成：

<p align="center">
  <img src="assets/formulas/eq_eth_compose.png" alt="合成 baseTcam" width="85%"/>
</p>

两种写法目标一致：得到场景相机相对基座的外参。实现时以所用库文档的输入约定为准，不要混用 A / B 的求逆顺序。

### 3.5 旋转–平移分解（理解求解时有用）

把 AX=XB 写成旋转与平移两部分：

<p align="center">
  <img src="assets/formulas/eq_rot.png" alt="旋转方程" width="45%"/>
</p>

<p align="center">
  <img src="assets/formulas/eq_trans.png" alt="平移方程" width="55%"/>
</p>

经典可分离方法（Tsai–Lenz 等）：先由旋转方程求 Rx，再代入平移方程求 tx。同时求解或迭代法（对偶四元数、非线性最小二乘）可减轻旋转误差向平移的传递。OpenCV 提供多种方法，例如：

- `CALIB_HAND_EYE_TSAI`
- `CALIB_HAND_EYE_PARK`
- `CALIB_HAND_EYE_HORAUD`
- `CALIB_HAND_EYE_ANDREFF`
- `CALIB_HAND_EYE_DANIILIDIS`

实践建议：同一数据集试 2–3 种方法，用验证位姿的重投影/闭环误差择优。

### 3.6 误差与可辨识性要点

1. **内参先准**：手眼外参建立在相机内参与畸变模型之上；RealSense 可用厂内参或 `rs-sensor-control` / 自标定结果。
2. **时间同步**：记录末端位姿与图像时间戳应对齐，避免运动中曝光造成的位姿错配。
3. **激励充分**：至少绕两个近似正交轴转动；仅平移或仅绕平行轴转会导致 X 退化。
4. **标定板质量**：角点清晰、平面度好；ChArUco 在部分遮挡下更稳。
5. **验证**：留出未参与求解的位姿，比较标定板在基座系下由机器人链与视觉链算出的差异（毫米级/亚毫米级视系统精度而定）：

<p align="center">
  <img src="assets/formulas/inline_baseTtgt.png" alt="baseTtgt" width="28%"/>
</p>

---

## 4. 以 RealSense 为例：作用、物料与步骤

<p align="center">
  <img src="assets/realsense_dual_setup.png" alt="RealSense 双相机场景" width="90%"/>
</p>
<p align="center"><em>图：腕部相机（Eye-in-Hand）与场景相机（Eye-to-Hand）同时部署的示意。</em></p>

### 4.1 标定的作用（为什么必须做）

| 相机角色 | 配置 | 标定后能做什么 |
| --- | --- | --- |
| 腕部 RealSense | Eye-in-Hand | 把深度/点云从相机系变到法兰/基座，做精定位抓取、贴合、插装 |
| 场景 RealSense | Eye-to-Hand | 在基座系下报告物体粗位姿，供运动规划、碰撞监测、任务调度 |
| 双机协同 | 两者都标定 | 场景粗定位 → 腕部精对准；两相机点云可变换到同一基座系融合 |

没有可靠手眼外参时，常见现象是：“画面里对准了，手臂却偏了几厘米”——那是坐标系没对齐，不是单纯检测算法问题。

### 4.2 物料清单（BOM）

#### 通用物料

| 类别 | 建议规格 | 备注 |
| --- | --- | --- |
| 机械臂 | 6 轴协作臂或工业臂 | 能稳定读出法兰/工具位姿 |
| 标定板 | 棋盘格或 ChArUco（亚克力/铝板） | 打印后贴平；注明格宽（mm） |
| 支架与快换 | 相机支架、法兰转接板 | 标定后勿再松动 |
| 工控机 | USB3 主机 + 机器人网口/总线 | RealSense 需稳定 USB3 |
| 软件 | librealsense、OpenCV、机器人 SDK | Python/C++ 均可 |
| 辅材 | 螺丝胶、遮光罩、标定板压条 | 减少微动与反光 |

#### Eye-in-Hand（腕部相机）专项

| 物料 | 示例 | 说明 |
| --- | --- | --- |
| 腕部深度相机 | RealSense D405（近距）或 D435 | D405 适合短距精细作业 |
| 腕部安装件 | 法兰→相机支架 | 视线尽量不被夹爪遮挡 |
| 固定标定板 | 桌面吸附/螺钉固定 | 标定全程相对基座不动 |

#### Eye-to-Hand（场景相机）专项

| 物料 | 示例 | 说明 |
| --- | --- | --- |
| 场景深度相机 | RealSense D435 / D455 | 视场覆盖工作空间 |
| 落地/龙门支架 | 高度与倾角可调 | 标定后锁定 |
| 末端标定板 | 轻质板固定在法兰或夹爪 | 板法向尽量朝向场景相机 |

### 4.3 操作步骤总览

<p align="center">
  <img src="assets/calib_flow.png" alt="标定流程" width="90%"/>
</p>
<p align="center"><em>图：从准备到部署的手眼标定主流程。</em></p>

#### 步骤 A：共同准备

1. **安装刚性化**：拧紧相机支架；腕部线缆留足够活动余量，避免运动拉扯改变外参。
2. **相机内参**：用 RealSense Viewer 确认流稳定；记录串号、分辨率、深度单元；必要时做板内参标定。
3. **标定板参数**：实测方格边长（不要只用打印设定值）；OpenCV 中 `squareLength` 等单位与机器人一致（通常米或毫米统一）。
4. **安全**：示教限速；场景相机支架远离臂工作包络碰撞区。

#### 步骤 B：Eye-in-Hand（腕部 RealSense）采集与求解

1. 将标定板固定在工作台，腕部相机对准板面。
2. 示教约 15–30 个位姿：改变距离、俯仰、偏航；保证板在彩色/红外图中完整清晰。
3. 每个位姿记录：
   - 末端相对基座位姿（或关节角由 FK 算出）
   - 图像 → 解算标定板相对相机位姿
4. 用相邻或全组合构造 (Ak, Bk)，调用例如 OpenCV：

```python
# 示意：R_gripper2base, t_gripper2base, R_target2cam, t_target2cam 为列表
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI,
)
# 得到 X = T_cam^ee（注意 OpenCV 命名 gripper≈ee）
```

5. **验证**：新到位姿，比较“FK + X + 视觉”得到的板在基座系位置与几何真值/多次平均的偏差。
6. **固化**：保存 X 与采集元数据（分辨率、板尺寸、方法名、时间）；支架打胶或标记，防止漂移。

#### 步骤 C：Eye-to-Hand（场景 RealSense）采集与求解

1. 场景相机固定；标定板装在末端，确保多数姿态下板朝向相机。
2. 同样采集多组位姿与标定板相对相机位姿。
3. 按库文档传入 Eye-to-Hand 模式（OpenCV 对输入列表的解释与 Eye-in-Hand 不同，需对照官方说明构造 R、t）。
4. 得到相机相对基座外参后，将场景点云/检测框变换到基座系，用已知桌面高度或第二块验证板检查误差。
5. 固化支架；记录相机相对机台的粗测尺寸，便于日后漂移排查。

#### 步骤 D：双相机协同部署建议

1. 两套外参都变换到同一 `base` 系。
2. 任务流示例：场景相机粗定位工件 → 规划接近 → 腕部相机精细 6D 位姿 → 抓取。
3. 定期回归：支架碰撞、换夹爪、重装相机后必须重标定或至少做验证集抽检。

### 4.4 常见问题速查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 平移偏差大、旋转尚可 | 板尺寸不准、深度尺度、时间不同步 | 重测格宽；静止曝光；检查单位 |
| 换方法结果跳变大 | 位姿激励不足或噪声大 | 增加姿态多样性；滤波角点 |
| 运行一段时间变差 | 支架微动、线缆受力 | 复打紧；加定位销；重标定 |
| 边缘位姿误差大 | 畸变/FOV；板几乎平行光轴 | 限制工作区；改善观测角 |

---

## 5. 小结

- **Eye-in-Hand**：相机在腕部，板在场景；核心求相机相对末端外参，服务近距精细作业。

<p align="center">
  <img src="assets/formulas/inline_eeTcam.png" alt="eeTcam" width="28%"/>
</p>

- **Eye-to-Hand**：相机在场景，板在手上；核心求相机相对基座外参，服务全局感知与调度。

<p align="center">
  <img src="assets/formulas/inline_baseTcam.png" alt="baseTcam" width="28%"/>
</p>

- 数学上两者都常归结为 **AX = XB**，差别在 A / B / X 的具体定义。
- 对 RealSense：腕部与场景相机可并存；先内参与刚性安装，再多姿态采集，最后验证并固化外参。

延伸阅读：

- [Hand–eye calibration problem（Wikipedia）](https://en.wikipedia.org/wiki/Hand_eye_calibration_problem)
- [OpenCV calibrateHandEye](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)
- [Intel RealSense 文档](https://dev.intelrealsense.com/docs)

---

*文档配图与公式图由本仓库脚本生成；公式约定请与所用机器人 SDK / OpenCV 版本对照后实施。*
