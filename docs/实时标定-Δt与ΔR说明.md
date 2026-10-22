# 实时标定中的 Δt / ΔR 说明

本文说明上位机实时求解界面上 **Δt**、**ΔR** 的含义与计算方式，便于判读收敛状态。

---

## 1. 这两个量在比什么？

实时求解每积累一批样本后，会重新解一次手眼外参，得到当前外参矩阵 \(X_k\)。  
**Δt / ΔR 不是单帧检测误差**，而是：

> **相邻两次外参解** \(X_{k-1}\) 与 \(X_k\) 之间的位姿差。

用来回答：再多采一点数据后，外参是否还在明显跳动。

比较对象由手眼模式决定（`ExtrinsicsResult.primary_T()`）：

| 模式 | 比较的外参 \(X\) |
|------|------------------|
| 眼在手外 `eye_to_hand` | \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\) |
| 眼在手上 `eye_in_hand` | \(T_{\mathrm{flange}\leftarrow\mathrm{cam}}\) |

实现见：`src/rs_calib/domain/convergence.py`、`src/rs_calib/domain/types.py`。

---

## 2. 计算公式

统一函数：`se3_delta(T_a, T_b)`（`src/rs_calib/domain/math3d.py`）。

设两次外参为齐次矩阵：

\[
T=\begin{bmatrix}R&t\\0&1\end{bmatrix},\quad
t\in\mathbb{R}^3,\ R\in\mathrm{SO}(3)
\]

### 2.1 Δt（平移差，单位 mm）

\[
\Delta t = \|\,t_a - t_b\,\|_2 \times 1000
\]

即两解平移向量欧氏距离，再换成毫米。

### 2.2 ΔR（旋转差，单位 °）

先相对旋转：

\[
R_{\mathrm{rel}} = R_a^{\mathsf{T}} R_b
\]

再取相对转角（Rodrigues / 迹公式）：

\[
\cos\theta = \mathrm{clip}\!\left(\dfrac{\mathrm{tr}(R_{\mathrm{rel}})-1}{2},\ -1,\ 1\right)
\qquad
\Delta R = \arccos(\cos\theta)\ \text{（再转为度）}
\]

\(\Delta R \in [0^\circ,\ 180^\circ]\)。  
**接近 180° 表示两解姿态几乎反向**，通常不是物理上真翻了半圈，而是求解分支不稳或样本不足（见 §4）。

---

## 3. 如何用于「收敛」判定？

默认阈值（`configs/default.yaml` → `convergence`）：

| 参数 | 含义 | 默认 |
|------|------|------|
| `max_delta_t_mm` | Δt 上限 | 1.5 mm |
| `max_delta_R_deg` | ΔR 上限 | 0.5 ° |
| `stable_count` | 连续满足次数 | 5 |
| `min_samples` | 最少样本 | 12 |

逻辑简述：

1. 样本数不足时：显示 Δt=ΔR=0，未收敛。  
2. 每次新解与**上一解**比：若 \(\Delta t\le\) 阈值且 \(\Delta R\le\) 阈值，则 `stable_streak += 1`，否则清零。  
3. `streak ≥ stable_count` 且样本够 → **收敛**，锁定当前解。  
4. 收敛后：新解若相对**锁解**超阈值，则拒样（不纳入），避免把已稳外参冲坏。

因此界面上的 Δt/ΔR 多数时候是「相对上一解」；拒样提示里的差值是「相对锁解」。

---

## 4. 读数注意点

1. **样本很少时**（刚开录 / 臂几乎没动）  
   HandEye 条件差，Δt、ΔR 可以很大，属正常，应继续走位采点。

2. **ΔR 反复接近 180°**  
   属于异常信号。常见原因是求解器在姿态分支间跳动。  
   当前工程在 OpenCV 无 `calibrateHandEye`（如部分 OpenCV 5.0 轮子）时使用原生 Tsai/Park/Andreff，并对上一解做姿态对齐，以抑制伪 180° 跳变。方法名可能出现 `*_NATIVE_NO_CV_HANDEYE`。

3. **与验证页误差不是同一指标**  
   - 实时 Δt/ΔR：外参解 \(X_{k-1}\) vs \(X_k\)  
   - 验证页板误差：预测板位姿 vs 当帧 ArUco 检测  
   二者都小，才更有信心。

> **血泪教训：** 曾出现「Δt/ΔR 已收敛，ETH 验证仍差 ~45 mm」——根因是原生 HandEye 实现错误，不是阈值太松。详见  
> `docs/踩坑记录-原生HandEye三大实现错误与ETH大误差.md`。

---

## 5. 代码入口（便于对照）

| 环节 | 位置 |
|------|------|
| Δt/ΔR 数值定义 | `domain/math3d.py` → `se3_delta` / `rotation_angle_deg` |
| 收敛与拒样 | `domain/convergence.py` → `ConvergenceTracker` |
| 实时调用 | `app_services/live_calib_service.py` → `offer_frame` |
| 手眼主矩阵 | `domain/types.py` → `ExtrinsicsResult.primary_T` |

---

## 6. 一句话总结

**Δt / ΔR = 相邻两次手眼外参解在 SE(3) 上的平移距离（mm）与相对转角（°）；连续多次都小于阈值才判收敛。**
