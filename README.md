# 前轴双轮边电机 TCS 牵引力控制器

> ⚠️ **重要声明**
>
> 本项目**完全由 Claude（AI）生成**，代码未经正式测试与仿真验证，参数未经过实车标定。当前仅通过了 MATLAB 环境下的基本单元测试（9 项），尚未在任何实际硬件平台（如 dSPACE、NI、嵌入式 ECU）上运行过。
>
> - **滑移率计算**中的低速保护阈值 (0.5 m/s)、PI 增益参数 (Kp=800, Ki=200) 均为理论初值，实际车辆需根据轮胎特性、路面条件重新标定。
> - **死区阈值** (±2%)、积分抗饱和上下限等参数同样为经验估算，不保证在实际工况下的稳定性与鲁棒性。
> - **Simulink 模型**未进行 MIL/SIL/HIL 验证，离散求解器步长 (5 ms) 是否满足实时性要求需在实际目标硬件上确认。
> - 🚧 **本项目仅供学习交流与方案参考，请勿直接用于实车控制。** 如有大佬发现 Bug 或设计缺陷，欢迎提 Issue / PR 指正，十分感谢！
>
> 如果你要把它用在真实赛车上：**请先做仿真验证 → 台架测试 → 实车标定，一步一步来。**

[![MATLAB](https://img.shields.io/badge/MATLAB-R2026a-blue)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-R2026a-orange)](https://www.mathworks.com/products/simulink.html)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

基于 PI 控制的前轴双独立轮边电机牵引力控制系统 (Traction Control System)，适用于 FSAE 方程式赛车 / 四驱混合动力车辆。

---

## 🚗 车辆架构

```
         ┌──────────────────────────────────┐
         │          前轴 (Front Axle)        │
         │   ┌──────────┐   ┌──────────┐    │
         │   │ 左轮边电机 │   │ 右轮边电机 │    │
         │   │  wfl, T_L │   │  wfr, T_R │    │
         │   └─────┬────┘   └─────┬────┘    │
         │         │ TCS 独立控制  │          │
         └─────────┼──────────────┼──────────┘
                   │              │
              ┌────▼──────────────▼────┐
              │      TCS Controller    │
              │  滑移率 → PI → 减扭    │
              └────────────────────────┘
                   │
         ┌─────────┴──────────────────┐
         │      后轴 (Rear Axle)       │
         │       发动机驱动            │
         └────────────────────────────┘
```

- **后轴**: 传统发动机驱动
- **前轴**: 左右独立轮边电机驱动，各自受 TCS 独立控制
- **控制目标**: 防止前轮起步/加速打滑，最大化驱动附着力

---

## 🎯 控制策略

### 滑移率计算

```
lambda = (R_wheel × ω - vx) / max(vx, 0.5)
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `R_wheel` | 0.25 m | 车轮滚动半径 |
| `vx_min` | 0.5 m/s | **低速保护** — 防止低速时分母趋零导致滑移率发散 |
| `lambda_target` | 10% | 目标滑移率（FSAE 典型值） |

### PI 控制器

```
error = lambda - lambda_target
PI_out = Kp × error + Ki × ∫error·dt
T_corr = −PI_out    （仅当 error > 0 时生效）
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `Kp` | 800 | 比例增益 [Nm/slip] |
| `Ki` | 200 | 积分增益 [Nm/(slip·s)] |
| `IntegralMax` | 500 | 积分抗饱和上限 |
| `IntegralMin` | 0 | 积分下限（仅减扭） |
| `Tcorr_min` | −500 Nm | 最大减扭量 |
| `Tcorr_max` | 0 Nm | **仅减扭**，不允许增扭 |

### 死区 (Deadband)

```
if |error| < 2%  →  误差置零，PI 不动作
```
防止滑移率在目标值附近微小波动时引起 PI 持续振荡。

### 控制周期

- **200 Hz** (dt = 5 ms)，满足 FSAE 快速响应要求
- 单步执行耗时 < 0.2 ms（MATLAB 环境实测）

---

## 📁 文件结构

```
.
├── README.md                       ← 本文件
├── TCS_Controller.m                ← 核心 TCS 算法函数
├── TCS_FrontAxle_Model.slx         ← Simulink 模型（含完整子系统）
├── build_TCS_Model.m               ← 模型构建脚本（可复现）
├── test_TCS_Controller.m           ← 单元测试（9项）
└── .gitignore
```

### 文件说明

| 文件 | 类型 | 功能 |
|------|------|------|
| `TCS_Controller.m` | MATLAB Function | 独立可调用的 TCS 算法，支持 Simulink MATLAB Function 块集成 |
| `TCS_FrontAxle_Model.slx` | Simulink Model | 完整仿真模型，TCS_Subsystem 用基础模块搭建，可视化 |
| `build_TCS_Model.m` | Build Script | 从零程序化构建 Simulink 模型（可重复执行） |
| `test_TCS_Controller.m` | Test Suite | 9 项单元测试，覆盖全部功能边界 |

---

## 🔌 接口定义

### 输入信号 (9路)

| 端口 | 信号 | 单位 | 说明 |
|------|------|------|------|
| 1 | `vx` | m/s | 车辆纵向速度 |
| 2 | `wfl` | rad/s | 左前轮转速 |
| 3 | `wfr` | rad/s | 右前轮转速 |
| 4 | `Tcmd_L` | Nm | 左电机扭矩指令 |
| 5 | `Tcmd_R` | Nm | 右电机扭矩指令 |
| 6 | `Mz_req` | Nm | 🔮 DYC 横摆力矩需求（预留） |
| 7 | `DeltaT_req` | Nm | 🔮 电子差速扭矩差（预留） |
| 8 | `Enable_DYC` | bool | 🔮 DYC 使能标志（预留） |
| 9 | `Enable_EDiff` | bool | 🔮 E-Diff 使能标志（预留） |

### 输出信号 (3路)

| 端口 | 信号 | 单位 | 说明 |
|------|------|------|------|
| 1 | `Tcorr_L` | Nm | 左电机扭矩修正量 (≤ 0) |
| 2 | `Tcorr_R` | Nm | 右电机扭矩修正量 (≤ 0) |
| 3 | `TCS_State` | enum | TCS 介入状态 |

### TCS_State 编码

| 值 | 状态 | 含义 |
|----|------|------|
| 0 | Normal | 无 TCS 介入 |
| 1 | Left Only | 仅左轮 TCS 减扭 |
| 2 | Right Only | 仅右轮 TCS 减扭 |
| 3 | Both | 双轮 TCS 同时减扭 |

---

## 🧪 测试结果

```
========================================
  TCS控制器单元测试 — 全部通过 (9/9)
========================================

 TC1  正常行驶无介入 ......... ✓  vx=10m/s, Tcorr=0
 TC2  大滑移率减扭 ........... ✓  20%→-82Nm
 TC3  死区振荡抑制 ........... ✓  11%→不触发(偏差<2%)
 TC4  左右轮独立控制 ......... ✓  左-121.5Nm | 右=0
 TC5  低速滑移率保护 ......... ✓  vx=0.2m/s 不发散
 TC6  仅减扭约束 ............. ✓  λ=-5%→Tcorr=0
 TC7  FSAE快速响应 .......... ✓  单步0.2ms (200Hz可行)
 TC8  积分抗饱和 ............. ✓  200步后-50Nm (≥-500)
 TC9  起步工况 ............... ✓  vx 0.1→5m/s 正常
========================================
```

### 时域仿真

| 工况 | 左轮λ | 右轮λ | Tcorr_L | Tcorr_R | State |
|------|-------|-------|---------|---------|-------|
| 左轮打滑 | 20% | 8% | **−138 Nm** | 0 Nm | 1 |
| 双轮打滑 | 30% | 25% | **−276 Nm** | **−207 Nm** | 3 |
| 正常行驶 | 0% | 0% | 0 Nm | 0 Nm | 0 |

---

## 🚀 快速开始

### 1. 打开 Simulink 模型

```matlab
open_system('TCS_FrontAxle_Model')
% 双击 TCS_Subsystem 查看内部控制框图
```

### 2. 运行仿真

在 Simulink 中点击 **Run**，或执行：

```matlab
sim('TCS_FrontAxle_Model')
```

### 3. 运行单元测试

```matlab
run('test_TCS_Controller.m')
```

### 4. 独立调用 TCS 算法

```matlab
[Tcorr_L, Tcorr_R, TCS_State] = TCS_Controller(...
    vx, wfl, wfr, Tcmd_L, Tcmd_R, ...
    Mz_req, DeltaT_req, Enable_DYC, Enable_EDiff, ...
    R_wheel, dt);
```

### 5. 重建模型（可选）

```matlab
run('build_TCS_Model.m')   % 从零重建 Simulink 模型
```

---

## 🔮 扩展规划

当前版本预留了以下接口，后续可扩展：

| 功能 | 接口 | 描述 |
|------|------|------|
| **DYC** (Direct Yaw Control) | `Mz_req`, `Enable_DYC` | 横摆力矩分配至前轴左右轮 |
| **Torque Vectoring / E-Diff** | `DeltaT_req`, `Enable_EDiff` | 电子差速扭矩矢量控制 |

扩展时 TCS 修正优先级最高：**先执行 TCS 减扭 → 再叠加 DYC/E-Diff 扭矩分配**。

---

## 📐 Simulink 子系统内部架构

```
                        TCS_Subsystem
┌──────────────────────────────────────────────────────────────┐
│                                                              │
│  vx ──┬── [Max(vx,0.5)] ────────────────────┐                │
│       │                                      │               │
│  wfl ── [×R] ─→ [Σ:Rω−vx] ─→ [÷] ─→ λ_L ──┤               │
│                                          │   │               │
│  wfr ── [×R] ─→ [Σ:Rω−vx] ─→ [÷] ─→ λ_R  │               │
│                                                              │
│  λ_L ─→ [Σ:λ−target] ─→ [Deadband ±2%] ─→ [PI:Kp+Ki∫]      │
│                              │                    │          │
│                              ▼                    ▼          │
│                         if |e|<2%:0      ─[×−1]─→ [Sat]→Tcorr_L
│                         else:e                         │      │
│                                               error>0?→减扭  │
│                                                              │
│  λ_R ─→ ...  (右轮对称结构) ──────────────→ Tcorr_R          │
│                                                              │
│  Tcorr_L < −0.01 ──→ [×1] ─┐                                 │
│  Tcorr_R < −0.01 ──→ [×2] ─┼── [Σ] ──→ TCS_State            │
│                              └────────────────               │
└──────────────────────────────────────────────────────────────┘
```

---

## 📄 License

MIT License — 详见 [LICENSE](LICENSE)
