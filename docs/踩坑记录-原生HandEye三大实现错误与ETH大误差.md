# 踩坑记录：原生 HandEye 三大实现错误 → ETH 绝对误差几十毫米

> 状态：**已修**（Park 外积、Tsai \(P\) 向量、全组合相对运动）  
> 发现会话：`session_20261022_193810`（眼在手外 ETH）  
> 关联坑：`docs/踩坑记录-OpenCV5无HandEye导致ΔR约180.md`（OpenCV 5 无官方 API → 被迫走原生）  
> 环境示例：Windows · `opencv-python` **5.0.0** · `calibrateHandEye` 缺失 · 方法名 `*_NATIVE_NO_CV_HANDEYE`

---

## 0. 先分清两套「误差」

| 指标 | 含义 | 小了代表什么 |
|------|------|----------------|
| 实时 **Δt / ΔR** | 相邻两次外参解 \(X_{k-1}\) 与 \(X_k\) 的差 | \(X\) **稳住了**，不等于绝对准 |
| 验证画面 **板误差** | 预测板位姿 vs 图像检测板位姿 | 外参 + 安装偏置在物理上对齐 |

本坑的典型假象：

- 实时已「收敛」（例如 Δt≈0.8 mm、ΔR≈0.03°）
- 验证里 RGB 预测轴与 CMY 检测轴差 **几十～一百多毫米 / 几十度**
- 第一反应会怪：`T_flange_marker`、板边长、法兰偏置量错了

**`T_flange_marker` 不参与 AX=XB 求解，只影响绝对画图。**  
ETH 这份数据上，即使用数据反推最优安装，旧外参绝对误差仍约 **45 mm**——说明 **\(X\) 本身就偏了**，不是只差一个常值偏置。

Δt/ΔR 定义见：`docs/实时标定-Δt与ΔR说明.md`。

---

## 1. 现象（有多坑）

以 `session_20261022_193810`（ETH · 单 ArUco 80 mm · 约 144 有效样本）复盘：

| 检查项 | 旧结果（踩坑时） | 修复后重解 |
|--------|------------------|------------|
| 方法名 | `TSAI_NATIVE_NO_CV_HANDEYE` | `TSAI_NATIVE_NO_CV_HANDEYE(PARK)` 等 |
| AX=XB 残差（表面） | 看起来不大 | 更小且与绝对误差一致 |
| 绝对验证（估最优 \(T_{fm}\)） | **~45 mm / ~8°** | **~3.6 mm / ~0.7°** |
| 相对运动预测（\(T_{fm}\) 消去） | **~60–80 mm** | **~数毫米** |
| 实时收敛 Δt/ΔR | 已收敛 | （旧文件不变；需重解） |

更坑的是：

1. **眼在手上（EIH）往往「还能用」**，眼在手外（ETH）却差一个数量级 → 像「ETH 公式写反了」。  
2. 验证里改 `T_flange_marker=165/0/40 mm` 几乎不救平移，反而可能更差 → 像「安装量错了」。  
3. 合成单测 `test_handeye_recovers_T_base_cam` 曾能过 → 像「代码没问题」。  

其实是：**原生求解器里有多处实现错误 + 选优逻辑把坏解藏起来 + ETH 轨迹更吃这些错。**

---

## 2. 正确公式（先钉死，避免误判「A/B 写反」）

记：

- \(G_i = T_{\mathrm{base}\leftarrow\mathrm{flange},i}\)
- \(C_i = T_{\mathrm{cam}\leftarrow\mathrm{marker},i}\)
- ETH：\(X = T_{\mathrm{base}\leftarrow\mathrm{cam}}\)，且 \(G_i\,T_{fm} = X\,C_i\)（\(T_{fm}\) 常值会在相对运动中消去）
- EIH：\(X = T_{\mathrm{flange}\leftarrow\mathrm{cam}}\)

**眼在手外（与 OpenCV ETH 喂 \(\mathrm{inv}(G)\) 时一致）：**

\[
A_{ij} = G_j\,G_i^{-1},\quad
B_{ij} = C_j\,C_i^{-1},\quad
A X = X B
\]

**眼在手上：**

\[
A_{ij} = G_i^{-1}\,G_j,\quad
B_{ij} = C_i\,C_j^{-1},\quad
A X = X B
\]

验证画图（ETH）：

\[
T_{\mathrm{cam}\leftarrow\mathrm{marker}}^{\mathrm{pred}}
= T_{\mathrm{cam}\leftarrow\mathrm{base}}\,G\,T_{fm}
= X^{-1}\,G\,T_{fm}
\]

本仓库 ETH 的 A/B **配对方向是对的**；大误差来自下面三处实现，而不是把 ETH/EIH 公式写反。

代码：`src/rs_calib/domain/handeye.py`、`src/rs_calib/domain/verify_geom.py`。

---

## 3. 大坑一：Park 旋转外积写反

### 3.1 该怎么写

由 \(R_A R_X = R_X R_B\)，旋转轴满足：

\[
R_X\,b = a
\]

其中 \(a,b\) 为相对运动旋转向量（单位化后）。Park–Martin 用

\[
M = \sum_i a_i b_i^{\mathsf{T}}
\]

再对 \(M\) 做极分解 / SVD（Procrustes）得 \(R_X\)。

OpenCV `calibrateHandEyePark` 写的是 \(M \mathrel{+}= b a^{\mathsf{T}}\)，但随后用带 \(M^{\mathsf{T}}\) 的公式，**等价于**在 \(\sum a b^{\mathsf{T}}\) 上求 \(R\)。

### 3.2 我们错在哪

旧原生 Park：

```text
M += outer(b, a)   # Σ b aᵀ
Rx = U @ Vt        # 直接对 M 做 SVD  ← 少了 OpenCV 里的 Mᵀ 处理
```

这会得到接近 **\(R_X^{\mathsf{T}}\)** 的旋转（或完全离谱的分支），平移再在错误 \(R\) 上最小二乘 → 整份 \(X\) 废掉。

### 3.3 合成证据

同一批无噪声 ETH 样本：

| 求解器 | 相对真值 |
|--------|----------|
| 坏 Park | ~100 mm / 几十度 |
| 修好 Park | ~0 |

### 3.4 修复

```text
M += outer(a, b)   # Σ a bᵀ，Rx @ b ≈ a
Rx = orthonormalize(U @ Vt)
```

单测：`tests/test_handeye_synthetic.py::test_handeye_park_recovers_T_base_cam`。

---

## 4. 大坑二：Tsai 用了错误的旋转参数（rotvec）

### 4.1 OpenCV / 论文用的 \(P\)

Tsai–Lenz（及 OpenCV `calibrateHandEyeTsai`）用的是 **修正 Rodrigues / 四元数向量部**：

\[
P = 2\,q_{xyz}
\quad(q_w \ge 0)
\]

不是 SciPy 的旋转向量 \(\theta\cdot\mathrm{axis}\)。

线性式大致为：

\[
[P_g + P_c]_{\times}\,P'_{cg} = P_c - P_g
\]

再

\[
P_{cg} = \frac{2 P'_{cg}}{\sqrt{1+|P'_{cg}|^2}}
\]

由 \(q_{xyz}=P_{cg}/2\) 还原旋转矩阵。

OpenCV 还会丢掉 \(|P|\) 过小或过大的运动（约 \(0.3\sim 1.7\)，对应转角大致十几度～一百多度）。

### 4.2 我们错在哪

旧实现：

1. 用 `_rotvec`（\(\theta\cdot\mathrm{axis}\)）当 \(P\) 填进 Tsai 方程；  
2. 还原时又用 `Rotation.from_rotvec(nx1 * scale)`，和 OpenCV「四元数向量部」不是同一空间。

在 **运动充分的合成数据** 上，旧 Tsai 有时仍能「凑合」过单测；  
在 **ETH 真实慢速/小姿态** 轨迹上，旧 Tsai 绝对误差可到 **~45 mm**，而修好后同数据 **~4 mm**。

### 4.3 修复

- `_tsai_p(R) = 2 * quat_xyz`（\(q_w\ge 0\)）  
- 还原：`Pcg = 2*nx1/sqrt(1+|nx1|^2)`，再 `from_quat([Pcg/2, qw])`  
- 过滤过小/过大 \(|P|`（不足时再放宽）

单测：`test_handeye_tsai_recovers_T_base_cam`。

---

## 5. 大坑三：相对运动只用「相邻帧」，OpenCV 用「全组合」

### 5.1 差异

| | 旧原生 | OpenCV `calibrateHandEye*` |
|--|--------|------------------------------|
| 相对运动 | 仅 \(i\to i+1\) | 所有 \(j>i\) 的位姿对 |
| 小步轨迹 | 每对转角往往只有几度 | 远距位姿对提供大转角 |

ETH 实时采集常是「密采样 + 慢挪」：相邻相对转角很小，方程病态；  
**全组合**才能掏出大基线相对运动。

### 5.2 真实 session 对照（修复 Park/Tsai 后）

`session_20261022_193810` 上（估最优 \(T_{fm}\) 后的绝对误差中位数）：

| 配对方式 | Park | Tsai（修好后） | Andreff |
|----------|------|----------------|---------|
| 仅相邻 | ~3.9 mm | （旧 Tsai 曾 ~45 mm） | 经常崩到百毫米 |
| 全组合 | ~3.6 mm | ~4.3 mm | 有时可变好，仍不稳 |

结论：

- 全组合是 OpenCV 对齐的正确做法，也让 Tsai 在真实 ETH 上回到可用。  
- Andreff 对配对/噪声更敏感，**不再默认参与跨求解器残差赛马**（仅当用户显式选 Andreff/Daniilidis 时才跑）。

### 5.3 修复

`_motion_pairs_axxb` 改为全组合 \((i,j)\)；样本过多时均匀抽到最多 64 帧再组对，避免实时 \(O(n^2)\) 卡死。  
相对转角过滤默认提高到约 **5°**（过少时再降到 1.5° / 全保留）。

---

## 6. 为什么「选优」反而坑了 ETH，却放过了 EIH

原生路径在无 OpenCV API 时会跑多个求解器，按 **AX=XB 矩阵残差** 选最小者。

旧世界里：

1. **坏 Park** 残差很大 → 被丢掉（好事）  
2. 常落到 **旧 Tsai** 或更脆的回退  
3. ETH 慢轨迹 + 相邻帧 → 旧 Tsai 绝对误差巨大，但残差未必「看起来灾难」  
4. 方法名仍写 `TSAI_NATIVE_...`，**看不出真正胜出的是谁**（曾用残差赛马却不标注 winner）

EIH 为何体感好一些：

- 旧逻辑同样丢掉坏 Park，多半用 Tsai；  
- 腕部相机标定板固定、臂姿态变化往往更大，相邻帧相对运动更「够用」；  
- 旧 Tsai 的 rotvec 错误在 EIH 上常表现为数毫米级，而不是 ETH 那种几十毫米。

合成对照（EIH）：旧 Tsai 在充分运动下可到 ~0.1 mm；坏 Park 仍然很差。  
所以：**不是 ETH 公式独错，是同一套坏实现被 ETH 轨迹放大。**

修复后方法名会带上真实胜者，例如：

```text
TSAI_NATIVE_NO_CV_HANDEYE(PARK)
```

`metrics` 含 `native_winner`、`axxb_residual`。

---

## 7. 和 `T_flange_marker` / 验证 UI 的关系（别再误判）

| 量 | 影响求解？ | 影响验证画图？ |
|----|------------|----------------|
| \(T_{\mathrm{base}\leftarrow\mathrm{cam}}\) / \(T_{\mathrm{flange}\leftarrow\mathrm{cam}}\) | 是 | 是 |
| \(T_{fm}=T_{\mathrm{flange}\leftarrow\mathrm{marker}}\) | **否**（常值会消去） | **是**（绝对轴） |
| `tcp_xyz` | 否 | 仅可选 TCP 轴 |

ETH 验证：

\[
A_{\mathrm{pred}} = T_{\mathrm{cam}\leftarrow\mathrm{base}}\,T_{\mathrm{base}\leftarrow\mathrm{flange}}\,T_{fm}
\]

`T_fm` 未知时旧逻辑当单位阵 → 轴画在法兰原点，和板心差一个安装偏置（本夹具约 **Z≈158 mm + Rx≈90°**）。  
那只解释「常值偏」；**解释不了相对运动预测也差几十毫米**——那是 \(X\) 错。

验证窗口可改 \(T_{fm}\)（mm + RPY）；默认值来自该 ETH 夹具重解隐含安装。  
配置：`configs/default.yaml` → `aruco.T_flange_marker`。

---

## 8. 怎么确认自己踩过 / 是否已修好

### 8.1 环境是否被迫走原生

```python
import cv2
print(cv2.__version__)
print("calibrateHandEye", hasattr(cv2, "calibrateHandEye"))
```

`False` → 100% 原生路径，本坑相关。

### 8.2 看旧 extrinsics

若 `method` 为 `TSAI_NATIVE_NO_CV_HANDEYE`（无 `(PARK)`），且验证绝对误差很大，而实时曾显示收敛 → 高度可疑。

### 8.3 用同一 session 重解（推荐）

对 `session_20261022_193810` 一类数据，用当前代码离线/实时重解后：

- 绝对误差（配合合理 \(T_{fm}\)）应降到 **数毫米** 量级；  
- `method` 常出现 `(PARK)`；  
- 隐含 \(T_{fm}\) 平移大约在法兰系 **[2, -43, 158] mm**，RPY 大约 **[91, 2, -2]°**（夹具相关，换工装需重测）。

**重要：磁盘上的旧 `extrinsics.json` 不会自动变好，必须重新求解并覆盖。**

### 8.4 回归测试

```bash
python -m pytest tests/test_handeye_synthetic.py -q
```

应覆盖 Park / Tsai / EIH / ETH 合成恢复。

---

## 9. 解决方案汇总（本仓库已做）

1. **Park**：`M += a bᵀ`，再 SVD 正交化。  
2. **Tsai**：OpenCV 风格 \(P=2q_{xyz}\) + 四元数还原 + \(|P|\) 过滤。  
3. **相对运动**：全组合 \((i,j)\)，样本封顶抽样；转角过滤加严。  
4. **选优**：默认只在 Park / Tsai（及用户点名的主方法）间比残差；Andreff 不默认参赛。  
5. **透明**：`method` 标注 `native_winner`；验证可编 \(T_{fm}\)。  
6. **文档**：本文 + OpenCV5 坑 + Δt/ΔR 说明交叉引用。

核心文件：`src/rs_calib/domain/handeye.py`。

---

## 10. 实操建议（避免再踩）

1. OpenCV 5 无 HandEye：接受 `NATIVE`，或改用确认带 API 的 OpenCV 4.x。  
2. ETH 采集：故意加大姿态变化，不要只靠平移扫点。  
3. 收敛 ≠ 合格：必须看验证双色轴 / 板误差，不要只看 Δt/ΔR。  
4. 换夹具后更新 \(T_{fm}\)（只影响画图与绝对误差解读）。  
5. 修求解器后 **重标 / 重解**，不要拿旧 `extrinsics.json` 评判新代码。

---

## 11. 一句话

> **OpenCV 5 逼出原生 HandEye 后，Park 外积反了、Tsai 当成 rotvec、相对运动只用邻帧；**  
> **坏 Park 被残差丢掉，ETH 落到更差的旧 Tsai 上，收敛却绝对误差 ~45 mm。**  
> **修：外积、\(P\) 向量、全组合；重解外参。EIH 伤得轻是因为当年多半走 Tsai 且运动更够，不是 ETH 公式写反。**

太坑了，特此立碑。
