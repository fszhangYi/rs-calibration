# 踩坑记录：OpenCV 5 无 `calibrateHandEye` → ΔR 反复 ≈180°

> 状态：**已修**（原生 Tsai/Park/Andreff + 相对上一解姿态对齐）  
> 发现会话：`session_20261021_142820` 一带的实时眼在手上标定  
> 环境示例：Windows · `opencv-python` **5.0.0** · 本机 `hasattr(cv2, "calibrateHandEye") == False`

---

## 现象（有多坑）

实时标定（尤其 **眼在手上**）过程中：

- 界面 **ΔR 多次顶在 170°～180°**
- 同时 Δt 也乱跳（几十～上百 mm）
- 偶发短暂「收敛」，随后又炸
- 方法名长期显示 **`TSAI_FALLBACK_SVD`**，容易误以为在跑 OpenCV 官方 TSAI

第一反应会觉得是「阈值太严 / 板太小 / 运动不够」。  
**其实是求解链路静默降级了，还在用一个很脆的 SVD 回退。**

---

## 根因（坑在哪）

### 1. OpenCV 5.0 轮子「半残」

部分 `opencv-python` **5.0.0** 环境里：

| 符号 | 有没有 |
|------|--------|
| `cv2.CALIB_HAND_EYE_TSAI` 等枚举 | **有** |
| `cv2.calibrateHandEye` | **没有** |

旧代码只做：

```text
if hasattr(cv2, "calibrateHandEye"): 用官方
else: TSAI_FALLBACK_SVD   ← 永远走这里
```

枚举在、API 不在 → **100% 静默走 fallback**，日志里还不像报错。

### 2. 旧 fallback（Kronecker SVD）在实时场景极不稳

实时阶段样本少、相对运动弱时，AX=XB 的 SVD 零空间很大，解会在 **两个几乎反向的旋转分支** 间跳。

相邻两次外参一比：

\[
\Delta R = \angle(R_{k-1}^{\mathsf{T}} R_k) \approx 180^\circ
\]

这不是板真的翻了半圈，是 **求解器在换分支**。  
对 `session_20261021_142820` 复盘：接受解相邻 ΔR **约 44% > 170°**。

### 3. 为什么特别坑

1. **没有显式失败**：UI 仍在「正常求解」  
2. **指标看起来像物理问题**：ΔR=180 很像姿态差很大  
3. **OpenCV 版本表象正常**：`import cv2` 成功，还有 HandEye 常量  
4. **眼在手上更敏感**：主矩阵是 `T_flange_cam`，分支一抖，ΔR 直接打满  

---

## 怎么确认自己踩了

在运行环境执行：

```python
import cv2
print(cv2.__version__)
print("calibrateHandEye", hasattr(cv2, "calibrateHandEye"))
print("CALIB_HAND_EYE_TSAI", hasattr(cv2, "CALIB_HAND_EYE_TSAI"))
```

若出现：

```text
5.0.0
calibrateHandEye False
CALIB_HAND_EYE_TSAI True
```

就是这个坑。

或看实时方法名：

- 旧坑：`TSAI_FALLBACK_SVD` 且 ΔR 常 ≈180  
- 修复后无官方 API：`TSAI_NATIVE_NO_CV_HANDEYE`（或同类 `*_NATIVE*`）

---

## 修复做法（本仓库）

1. **探测** `calibrateHandEye`（含可能的子模块路径）；没有则不用假装走 OpenCV。  
2. **原生求解**：Tsai / Park / Andreff，按 AX=XB 残差选优；过滤过小相对转动。  
3. **姿态对齐**：新解相对上一解 / 锁解，尝试抑制 180° 伪分支，避免界面假尖峰。  
4. **方法名诚实**：`*_NATIVE_NO_CV_HANDEYE` 一眼能看出没走官方 HandEye。

相关代码：

- `src/rs_calib/domain/handeye.py`
- `src/rs_calib/app_services/live_calib_service.py`（`align_to`）
- Δt/ΔR 定义见：`docs/实时标定-Δt与ΔR说明.md`

**续篇（更坑）：** 走上原生路径之后，Park 外积 / Tsai \(P\) 向量 / 仅相邻帧相对运动又把 **ETH 绝对误差** 拉到几十毫米——见  
`docs/踩坑记录-原生HandEye三大实现错误与ETH大误差.md`。

---

## 环境侧建议（可选）

- 需要官方实现：改用 **OpenCV 4.x**（确认 `hasattr(cv2, "calibrateHandEye")` 为 True）  
- 继续用 OpenCV 5：依赖本仓库原生求解即可，但请认方法名里的 `NATIVE`  

`pyproject.toml` 仍写 `opencv-python>=4.8`，**并不保证 5.0 带 HandEye**——以运行时探测为准。

---

## 一句话

> **OpenCV 5 有 HandEye 枚举、没 HandEye 函数 → 静默 SVD → ΔR 假 180。**  
> 修：原生求解 + 对齐 + 方法名说人话。太坑了，特此立碑。
