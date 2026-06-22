%% build_TCS_Model.m
% 程序化构建前轴双轮边电机TCS控制器Simulink模型
% 使用基础Simulink模块实现全部控制逻辑
% 架构: 滑移率计算 → PI控制(左右独立) → 扭矩限幅 → 状态判定
%
% 版本: v1.0 | 日期: 2026-06-22

clear; clc;

model_name = 'TCS_FrontAxle_Model';

% 关闭已打开的模型
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

%% ==================== 创建新模型 ====================
new_system(model_name, 'Model');
open_system(model_name);

% 模型级参数配置
set_param(model_name, 'StopTime', '5');
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.005');  % 200Hz FSAE

%% ==================== 模型工作区参数 ====================
mdlWks = get_param(model_name, 'ModelWorkspace');
mdlWks.assignin('R_wheel', 0.25);
mdlWks.assignin('lambda_target', 0.10);
mdlWks.assignin('Kp', 800);
mdlWks.assignin('Ki', 200);
mdlWks.assignin('deadband', 0.02);
mdlWks.assignin('vx_min', 0.5);
mdlWks.assignin('Tcorr_min_val', -500);
mdlWks.assignin('IntegralMax', 500);
mdlWks.assignin('IntegralMin', 0);

%% ==================== 构建 TCS Subsystem ====================
tcs = [model_name, '/TCS_Subsystem'];
add_block('simulink/Ports & Subsystems/Subsystem', tcs);
set_param(tcs, 'Position', [400, 50, 700, 600]);
set_param(tcs, 'BackgroundColor', 'cyan');

Simulink.SubSystem.deleteContents(tcs);

%% ---- 输入端口 ----
in_names = {'vx','wfl','wfr','Tcmd_L','Tcmd_R', ...
            'Mz_req','DeltaT_req','Enable_DYC','Enable_EDiff'};
for i = 1:9
    bh = add_block('simulink/Sources/In1', [tcs, '/', in_names{i}]);
    set_param(bh, 'Port', num2str(i));
    set_param(bh, 'Position', [30, 30+(i-1)*55, 60, 44+(i-1)*55]);
end

%% ---- 低速保护: max(vx, vx_min) ----
add_block('simulink/Math Operations/MinMax', [tcs, '/Max_Protect']);
set_param([tcs, '/Max_Protect'], 'Function', 'max');
set_param([tcs, '/Max_Protect'], 'Inputs', '2');
set_param([tcs, '/Max_Protect'], 'Position', [120, 30, 170, 60]);

add_block('simulink/Sources/Constant', [tcs, '/vx_min_Const']);
set_param([tcs, '/vx_min_Const'], 'Value', 'vx_min');
set_param([tcs, '/vx_min_Const'], 'Position', [120, 80, 170, 110]);

%% ---- 左轮滑移率计算 ----
% Gain: R_wheel * wfl
add_block('simulink/Math Operations/Gain', [tcs, '/L_Gain_R']);
set_param([tcs, '/L_Gain_R'], 'Gain', 'R_wheel');
set_param([tcs, '/L_Gain_R'], 'Position', [120, 140, 175, 175]);

% Sum: R*wfl - vx
add_block('simulink/Math Operations/Sum', [tcs, '/L_Sum1']);
set_param([tcs, '/L_Sum1'], 'Inputs', '+-');
set_param([tcs, '/L_Sum1'], 'Position', [230, 140, 265, 175]);

% Divide: lambda_L
add_block('simulink/Math Operations/Divide', [tcs, '/L_Div']);
set_param([tcs, '/L_Div'], 'Position', [320, 140, 355, 175]);

%% ---- 右轮滑移率计算 ----
add_block('simulink/Math Operations/Gain', [tcs, '/R_Gain_R']);
set_param([tcs, '/R_Gain_R'], 'Gain', 'R_wheel');
set_param([tcs, '/R_Gain_R'], 'Position', [120, 220, 175, 255]);

add_block('simulink/Math Operations/Sum', [tcs, '/R_Sum1']);
set_param([tcs, '/R_Sum1'], 'Inputs', '+-');
set_param([tcs, '/R_Sum1'], 'Position', [230, 220, 265, 255]);

add_block('simulink/Math Operations/Divide', [tcs, '/R_Div']);
set_param([tcs, '/R_Div'], 'Position', [320, 220, 355, 255]);

%% ---- 左轮误差 ----
add_block('simulink/Sources/Constant', [tcs, '/L_Target']);
set_param([tcs, '/L_Target'], 'Value', 'lambda_target');
set_param([tcs, '/L_Target'], 'Position', [390, 110, 430, 145]);

add_block('simulink/Math Operations/Sum', [tcs, '/L_Error']);
set_param([tcs, '/L_Error'], 'Inputs', '+-');
set_param([tcs, '/L_Error'], 'Position', [480, 140, 515, 175]);

%% ---- 右轮误差 ----
add_block('simulink/Sources/Constant', [tcs, '/R_Target']);
set_param([tcs, '/R_Target'], 'Value', 'lambda_target');
set_param([tcs, '/R_Target'], 'Position', [390, 190, 430, 225]);

add_block('simulink/Math Operations/Sum', [tcs, '/R_Error']);
set_param([tcs, '/R_Error'], 'Inputs', '+-');
set_param([tcs, '/R_Error'], 'Position', [480, 220, 515, 255]);

%% ---- 左轮死区 ----
add_block('simulink/Math Operations/Abs', [tcs, '/L_Abs']);
set_param([tcs, '/L_Abs'], 'Position', [540, 105, 575, 135]);

add_block('simulink/Sources/Constant', [tcs, '/L_DB_Const']);
set_param([tcs, '/L_DB_Const'], 'Value', 'deadband');
set_param([tcs, '/L_DB_Const'], 'Position', [540, 70, 575, 95]);

% Compare: if |error| < deadband → true (在死区内)
add_block('simulink/Logic and Bit Operations/Relational Operator', [tcs, '/L_RelOp']);
set_param([tcs, '/L_RelOp'], 'Operator', '<');
set_param([tcs, '/L_RelOp'], 'Position', [600, 80, 630, 150]);

% Switch: 死区内→0, 死区外→error
add_block('simulink/Signal Routing/Switch', [tcs, '/L_Switch']);
set_param([tcs, '/L_Switch'], 'Position', [660, 140, 695, 200]);

add_block('simulink/Sources/Constant', [tcs, '/L_Zero']);
set_param([tcs, '/L_Zero'], 'Value', '0');
set_param([tcs, '/L_Zero'], 'Position', [630, 210, 665, 235]);

%% ---- 右轮死区 ----
add_block('simulink/Math Operations/Abs', [tcs, '/R_Abs']);
set_param([tcs, '/R_Abs'], 'Position', [540, 185, 575, 215]);

add_block('simulink/Logic and Bit Operations/Relational Operator', [tcs, '/R_RelOp']);
set_param([tcs, '/R_RelOp'], 'Operator', '<');
set_param([tcs, '/R_RelOp'], 'Position', [600, 160, 630, 235]);

add_block('simulink/Signal Routing/Switch', [tcs, '/R_Switch']);
set_param([tcs, '/R_Switch'], 'Position', [660, 220, 695, 280]);

add_block('simulink/Sources/Constant', [tcs, '/R_Zero']);
set_param([tcs, '/R_Zero'], 'Value', '0');
set_param([tcs, '/R_Zero'], 'Position', [630, 290, 665, 315]);

%% ---- 左轮PI ----
add_block('simulink/Math Operations/Gain', [tcs, '/L_Kp']);
set_param([tcs, '/L_Kp'], 'Gain', 'Kp');
set_param([tcs, '/L_Kp'], 'Position', [740, 100, 785, 140]);

add_block('simulink/Math Operations/Gain', [tcs, '/L_Ki']);
set_param([tcs, '/L_Ki'], 'Gain', 'Ki');
set_param([tcs, '/L_Ki'], 'Position', [740, 155, 785, 195]);

add_block('simulink/Continuous/Integrator', [tcs, '/L_Integrator']);
set_param([tcs, '/L_Integrator'], 'UpperSaturationLimit', 'IntegralMax');
set_param([tcs, '/L_Integrator'], 'LowerSaturationLimit', 'IntegralMin');
set_param([tcs, '/L_Integrator'], 'Position', [830, 155, 875, 195]);

add_block('simulink/Math Operations/Sum', [tcs, '/L_PI_Sum']);
set_param([tcs, '/L_PI_Sum'], 'Inputs', '++');
set_param([tcs, '/L_PI_Sum'], 'Position', [920, 105, 955, 185]);

%% ---- 右轮PI ----
add_block('simulink/Math Operations/Gain', [tcs, '/R_Kp']);
set_param([tcs, '/R_Kp'], 'Gain', 'Kp');
set_param([tcs, '/R_Kp'], 'Position', [740, 250, 785, 285]);

add_block('simulink/Math Operations/Gain', [tcs, '/R_Ki']);
set_param([tcs, '/R_Ki'], 'Gain', 'Ki');
set_param([tcs, '/R_Ki'], 'Position', [740, 300, 785, 340]);

add_block('simulink/Continuous/Integrator', [tcs, '/R_Integrator']);
set_param([tcs, '/R_Integrator'], 'UpperSaturationLimit', 'IntegralMax');
set_param([tcs, '/R_Integrator'], 'LowerSaturationLimit', 'IntegralMin');
set_param([tcs, '/R_Integrator'], 'Position', [830, 300, 875, 340]);

add_block('simulink/Math Operations/Sum', [tcs, '/R_PI_Sum']);
set_param([tcs, '/R_PI_Sum'], 'Inputs', '++');
set_param([tcs, '/R_PI_Sum'], 'Position', [920, 250, 955, 335]);

%% ---- 左轮扭矩修正 (仅减扭) ----
add_block('simulink/Math Operations/Gain', [tcs, '/L_Negate']);
set_param([tcs, '/L_Negate'], 'Gain', '-1');
set_param([tcs, '/L_Negate'], 'Position', [990, 105, 1030, 145]);

add_block('simulink/Signal Routing/Switch', [tcs, '/L_TorqueSw']);
set_param([tcs, '/L_TorqueSw'], 'Position', [1060, 105, 1095, 165]);

add_block('simulink/Sources/Constant', [tcs, '/L_Tzero']);
set_param([tcs, '/L_Tzero'], 'Value', '0');
set_param([tcs, '/L_Tzero'], 'Position', [1030, 175, 1065, 200]);

add_block('simulink/Discontinuities/Saturation', [tcs, '/L_Sat']);
set_param([tcs, '/L_Sat'], 'UpperLimit', '0');
set_param([tcs, '/L_Sat'], 'LowerLimit', 'Tcorr_min_val');
set_param([tcs, '/L_Sat'], 'Position', [1120, 115, 1145, 155]);

%% ---- 右轮扭矩修正 (仅减扭) ----
add_block('simulink/Math Operations/Gain', [tcs, '/R_Negate']);
set_param([tcs, '/R_Negate'], 'Gain', '-1');
set_param([tcs, '/R_Negate'], 'Position', [990, 250, 1030, 290]);

add_block('simulink/Signal Routing/Switch', [tcs, '/R_TorqueSw']);
set_param([tcs, '/R_TorqueSw'], 'Position', [1060, 250, 1095, 310]);

add_block('simulink/Sources/Constant', [tcs, '/R_Tzero']);
set_param([tcs, '/R_Tzero'], 'Value', '0');
set_param([tcs, '/R_Tzero'], 'Position', [1030, 320, 1065, 345]);

add_block('simulink/Discontinuities/Saturation', [tcs, '/R_Sat']);
set_param([tcs, '/R_Sat'], 'UpperLimit', '0');
set_param([tcs, '/R_Sat'], 'LowerLimit', 'Tcorr_min_val');
set_param([tcs, '/R_Sat'], 'Position', [1120, 260, 1145, 300]);

%% ---- TCS状态判定 ----
add_block('simulink/Logic and Bit Operations/Relational Operator', [tcs, '/L_Detect']);
set_param([tcs, '/L_Detect'], 'Operator', '<');
set_param([tcs, '/L_Detect'], 'Position', [1180, 70, 1210, 100]);

add_block('simulink/Sources/Constant', [tcs, '/L_Thresh']);
set_param([tcs, '/L_Thresh'], 'Value', '-0.01');
set_param([tcs, '/L_Thresh'], 'Position', [1150, 35, 1185, 60]);

add_block('simulink/Logic and Bit Operations/Relational Operator', [tcs, '/R_Detect']);
set_param([tcs, '/R_Detect'], 'Operator', '<');
set_param([tcs, '/R_Detect'], 'Position', [1180, 115, 1210, 145]);

add_block('simulink/Sources/Constant', [tcs, '/R_Thresh']);
set_param([tcs, '/R_Thresh'], 'Value', '-0.01');
set_param([tcs, '/R_Thresh'], 'Position', [1150, 155, 1185, 180]);

add_block('simulink/Math Operations/Gain', [tcs, '/L_Gain1']);
set_param([tcs, '/L_Gain1'], 'Gain', '1');
set_param([tcs, '/L_Gain1'], 'Position', [1240, 70, 1270, 100]);

add_block('simulink/Math Operations/Gain', [tcs, '/R_Gain2']);
set_param([tcs, '/R_Gain2'], 'Gain', '2');
set_param([tcs, '/R_Gain2'], 'Position', [1240, 115, 1270, 145]);

add_block('simulink/Math Operations/Sum', [tcs, '/State_Sum']);
set_param([tcs, '/State_Sum'], 'Inputs', '++');
set_param([tcs, '/State_Sum'], 'Position', [1300, 80, 1330, 130]);

%% ---- 输出端口 ----
out_names = {'Tcorr_L','Tcorr_R','TCS_State'};
for i = 1:3
    bh = add_block('simulink/Sinks/Out1', [tcs, '/', out_names{i}]);
    set_param(bh, 'Port', num2str(i));
    set_param(bh, 'Position', [1370, 100+(i-1)*80, 1400, 114+(i-1)*80]);
end

%% ==================== TCS Subsystem 连线 ====================
% 简写
PH = @(blk) get_param([tcs, '/', blk], 'PortHandles');
IN = @(n) PH(in_names{n});

% --- vx → Max_Protect, L_Sum1(-), R_Sum1(-) ---
add_line(tcs, IN(1).Outport(1), PH('Max_Protect').Inport(1), 'autorouting', 'on');
add_line(tcs, IN(1).Outport(1), PH('L_Sum1').Inport(2), 'autorouting', 'on');
add_line(tcs, IN(1).Outport(1), PH('R_Sum1').Inport(2), 'autorouting', 'on');

% vx_min → Max_Protect
add_line(tcs, PH('vx_min_Const').Outport(1), PH('Max_Protect').Inport(2), 'autorouting', 'on');

% --- 左轮信号链 ---
% wfl → L_Gain_R → L_Sum1(+) → L_Div(u1)
add_line(tcs, IN(2).Outport(1), PH('L_Gain_R').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Gain_R').Outport(1), PH('L_Sum1').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Sum1').Outport(1), PH('L_Div').Inport(1), 'autorouting', 'on');
% vx_prot → L_Div(u2)
add_line(tcs, PH('Max_Protect').Outport(1), PH('L_Div').Inport(2), 'autorouting', 'on');
% lambda_L → L_Error(+)
add_line(tcs, PH('L_Div').Outport(1), PH('L_Error').Inport(1), 'autorouting', 'on');
% lambda_target → L_Error(-)
add_line(tcs, PH('L_Target').Outport(1), PH('L_Error').Inport(2), 'autorouting', 'on');

% --- 左轮死区 ---
% error → L_Abs, L_Switch(u1)
add_line(tcs, PH('L_Error').Outport(1), PH('L_Abs').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Error').Outport(1), PH('L_Switch').Inport(1), 'autorouting', 'on');
% |error| → L_RelOp(u1), deadband → L_RelOp(u2)
add_line(tcs, PH('L_Abs').Outport(1), PH('L_RelOp').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_DB_Const').Outport(1), PH('L_RelOp').Inport(2), 'autorouting', 'on');
% RelOp → L_Switch(u2), Zero → L_Switch(u3)
add_line(tcs, PH('L_RelOp').Outport(1), PH('L_Switch').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('L_Zero').Outport(1), PH('L_Switch').Inport(3), 'autorouting', 'on');

% --- 左轮PI ---
add_line(tcs, PH('L_Switch').Outport(1), PH('L_Kp').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Switch').Outport(1), PH('L_Ki').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Ki').Outport(1), PH('L_Integrator').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Kp').Outport(1), PH('L_PI_Sum').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Integrator').Outport(1), PH('L_PI_Sum').Inport(2), 'autorouting', 'on');

% --- 左轮扭矩修正 ---
add_line(tcs, PH('L_PI_Sum').Outport(1), PH('L_Negate').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Negate').Outport(1), PH('L_TorqueSw').Inport(1), 'autorouting', 'on');
% error作为控制: error>0时才减扭
add_line(tcs, PH('L_Error').Outport(1), PH('L_TorqueSw').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('L_Tzero').Outport(1), PH('L_TorqueSw').Inport(3), 'autorouting', 'on');
add_line(tcs, PH('L_TorqueSw').Outport(1), PH('L_Sat').Inport(1), 'autorouting', 'on');

% --- 右轮信号链 ---
add_line(tcs, IN(3).Outport(1), PH('R_Gain_R').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Gain_R').Outport(1), PH('R_Sum1').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Sum1').Outport(1), PH('R_Div').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('Max_Protect').Outport(1), PH('R_Div').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('R_Div').Outport(1), PH('R_Error').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Target').Outport(1), PH('R_Error').Inport(2), 'autorouting', 'on');

% --- 右轮死区 ---
add_line(tcs, PH('R_Error').Outport(1), PH('R_Abs').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Error').Outport(1), PH('R_Switch').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Abs').Outport(1), PH('R_RelOp').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_DB_Const').Outport(1), PH('R_RelOp').Inport(2), 'autorouting', 'on'); % share deadband
add_line(tcs, PH('R_RelOp').Outport(1), PH('R_Switch').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('R_Zero').Outport(1), PH('R_Switch').Inport(3), 'autorouting', 'on');

% --- 右轮PI ---
add_line(tcs, PH('R_Switch').Outport(1), PH('R_Kp').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Switch').Outport(1), PH('R_Ki').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Ki').Outport(1), PH('R_Integrator').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Kp').Outport(1), PH('R_PI_Sum').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Integrator').Outport(1), PH('R_PI_Sum').Inport(2), 'autorouting', 'on');

% --- 右轮扭矩修正 ---
add_line(tcs, PH('R_PI_Sum').Outport(1), PH('R_Negate').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Negate').Outport(1), PH('R_TorqueSw').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Error').Outport(1), PH('R_TorqueSw').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('R_Tzero').Outport(1), PH('R_TorqueSw').Inport(3), 'autorouting', 'on');
add_line(tcs, PH('R_TorqueSw').Outport(1), PH('R_Sat').Inport(1), 'autorouting', 'on');

% --- TCS State ---
add_line(tcs, PH('L_Sat').Outport(1), PH('L_Detect').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Thresh').Outport(1), PH('L_Detect').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('R_Sat').Outport(1), PH('R_Detect').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Thresh').Outport(1), PH('R_Detect').Inport(2), 'autorouting', 'on');
add_line(tcs, PH('L_Detect').Outport(1), PH('L_Gain1').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Detect').Outport(1), PH('R_Gain2').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('L_Gain1').Outport(1), PH('State_Sum').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Gain2').Outport(1), PH('State_Sum').Inport(2), 'autorouting', 'on');

% --- Output ---
add_line(tcs, PH('L_Sat').Outport(1), PH('Tcorr_L').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('R_Sat').Outport(1), PH('Tcorr_R').Inport(1), 'autorouting', 'on');
add_line(tcs, PH('State_Sum').Outport(1), PH('TCS_State').Inport(1), 'autorouting', 'on');

%% ==================== 布置子系统 ====================
Simulink.BlockDiagram.arrangeSystem(tcs);

%% ==================== 顶层模型 ====================
% 测试信号: vx (ramp 0→15 m/s)
add_block('simulink/Sources/Ramp', [model_name, '/vx_Ramp']);
set_param([model_name, '/vx_Ramp'], 'Position', [50, 80, 150, 120]);
set_param([model_name, '/vx_Ramp'], 'slope', '5');
set_param([model_name, '/vx_Ramp'], 'start', '0.2');
set_param([model_name, '/vx_Ramp'], 'InitialOutput', '0.1');

% wfl: 20% 滑移模拟 (左前轮打滑)
add_block('simulink/Math Operations/Gain', [model_name, '/wfl_20pctSlip']);
set_param([model_name, '/wfl_20pctSlip'], 'Position', [200, 130, 270, 170]);
set_param([model_name, '/wfl_20pctSlip'], 'Gain', '4.80');  % (1+0.20)/0.25

% wfr: 5% 滑移模拟 (右前轮轻微滑移)
add_block('simulink/Math Operations/Gain', [model_name, '/wfr_5pctSlip']);
set_param([model_name, '/wfr_5pctSlip'], 'Position', [200, 190, 270, 230]);
set_param([model_name, '/wfr_5pctSlip'], 'Gain', '4.20');  % (1+0.05)/0.25

% Tcmd 电机扭矩指令
add_block('simulink/Sources/Constant', [model_name, '/Tcmd']);
set_param([model_name, '/Tcmd'], 'Position', [50, 280, 150, 320]);
set_param([model_name, '/Tcmd'], 'Value', '150');

% 预留接口
add_block('simulink/Sources/Constant', [model_name, '/Mz_req']);
set_param([model_name, '/Mz_req'], 'Position', [50, 360, 150, 390]);
set_param([model_name, '/Mz_req'], 'Value', '0');

add_block('simulink/Sources/Constant', [model_name, '/DeltaT_req']);
set_param([model_name, '/DeltaT_req'], 'Position', [50, 410, 150, 440]);
set_param([model_name, '/DeltaT_req'], 'Value', '0');

add_block('simulink/Sources/Constant', [model_name, '/Enable_DYC']);
set_param([model_name, '/Enable_DYC'], 'Position', [50, 460, 150, 490]);
set_param([model_name, '/Enable_DYC'], 'Value', '0');

add_block('simulink/Sources/Constant', [model_name, '/Enable_EDiff']);
set_param([model_name, '/Enable_EDiff'], 'Position', [50, 510, 150, 540]);
set_param([model_name, '/Enable_EDiff'], 'Value', '0');

% Scope: 监控5路信号
add_block('simulink/Sinks/Scope', [model_name, '/Scope']);
set_param([model_name, '/Scope'], 'Position', [700, 60, 880, 240]);
set_param([model_name, '/Scope'], 'NumInputPorts', '5');

% Display
add_block('simulink/Sinks/Display', [model_name, '/Tcorr_L_Disp']);
set_param([model_name, '/Tcorr_L_Disp'], 'Position', [700, 300, 820, 340]);

add_block('simulink/Sinks/Display', [model_name, '/Tcorr_R_Disp']);
set_param([model_name, '/Tcorr_R_Disp'], 'Position', [700, 360, 820, 400]);

add_block('simulink/Sinks/Display', [model_name, '/TCS_State_Disp']);
set_param([model_name, '/TCS_State_Disp'], 'Position', [700, 420, 820, 460]);

% 注: 预留接口 (Mz_req, DeltaT_req, Enable_DYC, Enable_EDiff)
% 已连接至TCS_Subsystem，后续扩展DYC/EDiff时可直接使用
fprintf('注: 预留DYC/E-Diff扩展接口已连接至TCS_Subsystem\n');

%% ==================== 顶层连线 ====================
ph_top = @(blk) get_param([model_name, '/', blk], 'PortHandles');
ph_tcs_in = get_param(tcs, 'PortHandles');

% vx → wfl_sim, wfr_sim, TCS(1)
add_line(model_name, ph_top('vx_Ramp').Outport(1), ph_top('wfl_20pctSlip').Inport(1), 'autorouting', 'on');
add_line(model_name, ph_top('vx_Ramp').Outport(1), ph_top('wfr_5pctSlip').Inport(1), 'autorouting', 'on');
add_line(model_name, ph_top('vx_Ramp').Outport(1), ph_tcs_in.Inport(1), 'autorouting', 'on');

% wfl_sim → TCS(2), wfr_sim → TCS(3)
add_line(model_name, ph_top('wfl_20pctSlip').Outport(1), ph_tcs_in.Inport(2), 'autorouting', 'on');
add_line(model_name, ph_top('wfr_5pctSlip').Outport(1), ph_tcs_in.Inport(3), 'autorouting', 'on');

% Tcmd → TCS(4), TCS(5)
add_line(model_name, ph_top('Tcmd').Outport(1), ph_tcs_in.Inport(4), 'autorouting', 'on');
add_line(model_name, ph_top('Tcmd').Outport(1), ph_tcs_in.Inport(5), 'autorouting', 'on');

% 预留接口 → TCS(6-9)
add_line(model_name, ph_top('Mz_req').Outport(1), ph_tcs_in.Inport(6), 'autorouting', 'on');
add_line(model_name, ph_top('DeltaT_req').Outport(1), ph_tcs_in.Inport(7), 'autorouting', 'on');
add_line(model_name, ph_top('Enable_DYC').Outport(1), ph_tcs_in.Inport(8), 'autorouting', 'on');
add_line(model_name, ph_top('Enable_EDiff').Outport(1), ph_tcs_in.Inport(9), 'autorouting', 'on');

% TCS输出 → Scope/Display
ph_tcs_out = get_param(tcs, 'PortHandles');
ph_scope = ph_top('Scope');

% Scope通道: vx, Tcorr_L, Tcorr_R, 保留2路
add_line(model_name, ph_top('vx_Ramp').Outport(1), ph_scope.Inport(1), 'autorouting', 'on');
add_line(model_name, ph_tcs_out.Outport(1), ph_scope.Inport(2), 'autorouting', 'on');
add_line(model_name, ph_tcs_out.Outport(2), ph_scope.Inport(3), 'autorouting', 'on');

% Display
add_line(model_name, ph_tcs_out.Outport(1), ph_top('Tcorr_L_Disp').Inport(1), 'autorouting', 'on');
add_line(model_name, ph_tcs_out.Outport(2), ph_top('Tcorr_R_Disp').Inport(1), 'autorouting', 'on');
add_line(model_name, ph_tcs_out.Outport(3), ph_top('TCS_State_Disp').Inport(1), 'autorouting', 'on');

%% ==================== 最终整理 ====================
Simulink.BlockDiagram.arrangeSystem(model_name);
save_system(model_name);

fprintf('========================================\n');
fprintf(' TCS Simulink模型构建成功!\n');
fprintf(' 模型: %s.slx\n', model_name);
fprintf('========================================\n');
fprintf('\n模型架构:\n');
fprintf(' [vx_Ramp]─────────────────────────────┐\n');
fprintf('    ├── [wfl_20pctSlip] ───────────────┤\n');
fprintf('    └── [wfr_5pctSlip]  ───────────────┤\n');
fprintf(' [Tcmd=150] ───────────────────────────┤\n');
fprintf(' [Mz_req,DeltaT_req,EnDYC,EnEDiff=0] ──┤\n');
fprintf('                                        v\n');
fprintf('                              ┌─────────────────┐\n');
fprintf('                              │  TCS_Subsystem  │\n');
fprintf('                              │  ┌─────────────┐│\n');
fprintf('                              │  │ Slip Calc   ││\n');
fprintf('                              │  │ PI Control  ││\n');
fprintf('                              │  │ Torque Limit││\n');
fprintf('                              │  │ State Logic ││\n');
fprintf('                              │  └─────────────┘│\n');
fprintf('                              └──────┬────┬─────┘\n');
fprintf('                                     v    v\n');
fprintf('                              [Tcorr_L] [Tcorr_R]\n');
fprintf('                                     [TCS_State]\n');
fprintf('\n控制参数:\n');
fprintf('  目标滑移率: 10%% | Kp=800 | Ki=200\n');
fprintf('  死区: ±2%% | 低速保护: max(vx,0.5)\n');
fprintf('  仅减扭: [%d, 0] Nm | 积分限幅: [0, %d]\n', -500, 500);
fprintf('  控制周期: 5ms (200Hz FSAE)\n');
