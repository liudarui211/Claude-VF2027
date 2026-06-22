function [Tcorr_L, Tcorr_R, TCS_State] = TCS_Controller(vx, wfl, wfr, Tcmd_L, Tcmd_R, ...
    Mz_req, DeltaT_req, Enable_DYC, Enable_EDiff, R_wheel, dt)
% TCS_CONTROLLER 前轴双轮边电机牵引力控制器 (Traction Control System)
%
% 功能描述:
%   - 左右轮独立滑移率计算与PI控制
%   - 仅允许减扭（TCS不增扭）
%   - 低速滑移率保护 (max(vx, 0.5))
%   - 滑移率死区避免振荡
%   - FSAE快速响应设计
%
% 输入参数:
%   vx         - 车辆纵向速度 [m/s]
%   wfl        - 左前轮转速 [rad/s]
%   wfr        - 右前轮转速 [rad/s]
%   Tcmd_L     - 左电机扭矩指令 [Nm] (驾驶员/VCU请求)
%   Tcmd_R     - 右电机扭矩指令 [Nm] (驾驶员/VCU请求)
%   Mz_req     - DYC横摆力矩需求 [Nm] (预留接口)
%   DeltaT_req - 电子差速扭矩差 [Nm] (预留接口)
%   Enable_DYC - DYC使能标志 (预留接口)
%   Enable_EDiff- 电子差速使能标志 (预留接口)
%   R_wheel    - 车轮滚动半径 [m]
%   dt         - 控制周期 [s] (FSAE典型值: 0.005~0.010s)
%
% 输出参数:
%   Tcorr_L    - 左电机扭矩修正量 [Nm] (≤0, 仅减扭)
%   Tcorr_R    - 右电机扭矩修正量 [Nm] (≤0, 仅减扭)
%   TCS_State  - TCS状态字 [0=正常, 1=左轮介入, 2=右轮介入, 3=双轮介入]
%
% 设计参数 (FSAE调校):
%   - 目标滑移率: 10%
%   - PI参数: Kp=800, Ki=200 (可在线标定)
%   - 死区: ±2% 滑移率
%   - 低速保护阈值: 0.5 m/s
%
% 预留扩展:
%   - DYC (Direct Yaw-moment Control): 横摆力矩分配
%   - Torque Vectoring / E-Diff: 电子差速扭矩矢量控制
%
% 版本: v1.0
% 日期: 2026-06-22

%#codegen

%% ==================== 持久变量定义 (PI积分项) ====================
persistent integral_L integral_R
if isempty(integral_L)
    integral_L = 0;
    integral_R = 0;
end

%% ==================== TCS控制参数 ====================
% --- 目标滑移率 ---
lambda_target = 0.10;           % 10% 目标滑移率

% --- PI控制器参数 (FSAE快速响应标定) ---
Kp = 800;                       % 比例增益 [Nm / slip_ratio]
Ki = 200;                       % 积分增益 [Nm / (slip_ratio * s)]

% --- 死区参数 ---
deadband = 0.02;                % ±2% 滑移率死区

% --- 低速保护 ---
vx_min = 0.5;                   % 最低速度阈值 [m/s]

% --- 积分抗饱和限幅 ---
integral_max = 500;             % 最大积分累积 [Nm]
integral_min = 0;               % 最小积分累积 [Nm] (仅减扭，积分≥0)

% --- 扭矩修正限幅 ---
Tcorr_max = 0;                  % TCS仅减扭，修正量≤0
Tcorr_min = -500;               % 最大减扭量 [Nm]

%% ==================== 滑移率计算 ====================
% 低速保护分母: max(vx, vx_min)
vx_prot = max(vx, vx_min);

% 左前轮滑移率
lambda_L = (R_wheel * wfl - vx) / vx_prot;

% 右前轮滑移率
lambda_R = (R_wheel * wfr - vx) / vx_prot;

%% ==================== 滑移率误差计算 ====================
error_L = lambda_L - lambda_target;
error_R = lambda_R - lambda_target;

%% ==================== 死区处理 ====================
% 滑移率在死区内时，误差置零以抑制PI振荡
if abs(error_L) < deadband
    error_L_active = 0;
else
    error_L_active = error_L;
end

if abs(error_R) < deadband
    error_R_active = 0;
else
    error_R_active = error_R;
end

%% ==================== PI控制 ====================
% --- 左轮PI ---
P_term_L = Kp * error_L_active;
integral_L = integral_L + Ki * error_L_active * dt;

% 积分抗饱和
integral_L = max(integral_min, min(integral_max, integral_L));

% 当滑移率回到死区内时，缓慢泄放积分项 (避免积分饱和导致的超调)
if abs(error_L) < deadband
    integral_L = integral_L * 0.95;  % 积分泄漏
end

I_term_L = integral_L;
PI_out_L = P_term_L + I_term_L;

% --- 右轮PI ---
P_term_R = Kp * error_R_active;
integral_R = integral_R + Ki * error_R_active * dt;

% 积分抗饱和
integral_R = max(integral_min, min(integral_max, integral_R));

% 积分泄漏
if abs(error_R) < deadband
    integral_R = integral_R * 0.95;
end

I_term_R = integral_R;
PI_out_R = P_term_R + I_term_R;

%% ==================== TCS扭矩修正 (仅减扭) ====================
% TCS仅允许减小扭矩 (Tcorr ≤ 0)
% PI输出为正表示需要减少扭矩 (lambda > target)
% PI输出为负表示滑移率不足，但TCS不增扭，钳位为0

% 当滑移率不超标时(error <= 0)，不需要减扭
if error_L_active <= 0
    Tcorr_L_raw = 0;
else
    Tcorr_L_raw = -PI_out_L;  % 转换为负修正量
end

if error_R_active <= 0
    Tcorr_R_raw = 0;
else
    Tcorr_R_raw = -PI_out_R;
end

% 限幅 (确保仅减扭且不超过最大减扭量)
Tcorr_L = max(Tcorr_min, min(Tcorr_max, Tcorr_L_raw));
Tcorr_R = max(Tcorr_min, min(Tcorr_max, Tcorr_R_raw));

%% ==================== TCS状态判定 ====================
% TCS_State 编码:
%   0 = 正常工作 (无TCS介入)
%   1 = 仅左轮TCS介入
%   2 = 仅右轮TCS介入
%   3 = 双轮TCS介入

TCS_State = uint8(0);
if Tcorr_L < -0.01  % 左轮有减扭动作 (阈值避免浮点抖动)
    TCS_State = TCS_State + 1;
end
if Tcorr_R < -0.01  % 右轮有减扭动作
    TCS_State = TCS_State + 2;
end

%% ==================== 预留接口 ====================
% 以下参数在后续扩展DYC/Torque Vectoring时使用:
%   Mz_req      - 横摆力矩需求，用于DYC扭矩分配
%   DeltaT_req  - 电子差速扭矩差，用于E-Diff控制
%   Enable_DYC  - DYC使能标志
%   Enable_EDiff - 电子差速使能标志
%
% 扩展方案:
%   当 Enable_DYC == 1 时，将Mz_req分配至左右轮 (前轴)
%   当 Enable_EDiff == 1 时，将DeltaT_req叠加至左右轮扭矩
%   TCS修正优先级最高，先执行TCS减扭，再进行DYC/E-Diff分配

end
