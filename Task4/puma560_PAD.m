clear; clc;
%% 参数与模型加载
mdl_puma560;                % 加载 Puma560 模型
robot = p560;               % SerialLink 对象
n = robot.n;                % 6

% 初始关节角（可任选安全范围内姿态）
q0 = zeros(1,n);            % 初始姿态，用于逆解或直接使用
dt = 0.05;                  % 50 ms 采样
% total_time 可根据 PAD 分段决定；这里假设总时长 200s
total_time = 74;
num_steps = ceil(total_time/dt);

% PAD→JVG 映射参数
alphaJ = 1.8;
alphaV = 1.2;
alphaG = 1.3;

% 振荡频率范围 (Hz)
f_min = 0.1;
f_max = 2.0;

% 振幅系数
k_V = 0.2 * ones(1,n);
q_neutral = zeros(1,n);
q_open = [0,  0.3, -0.5,  0.4, 0, 0];
Delta_q_open = q_open - q_neutral;
k_G = 0.5;
offset_time_const = 1.0;
phi0 = 2*pi*rand(1,n);

% ---- 新增：往返起点和终点，以及半程时长 ----
% 在工作空间中选两个合理点 p_start, p_end，注意保持姿态可解
p_start = [0.5,  0.5, 0.5];  
p_end   = [0.5,  0.5, -0.5];
T_half = 4;   % 每半程时长（秒），可根据需要调整
% 计算往返周期
T_period = 2 * T_half;  % 完整往返一次的周期

% 若需要逆解初始关节使末端在 p_start（可选）
% 这里假定初始 q0 已在 p_start，可用 ikine:
T_des0 = transl(p_start);
q = robot.ikine(T_des0, 'mask', [1 1 1 0 0 0]);
if isempty(q)
    q = q0;  % 若逆解失败，就用默认初始姿态
end

% 录视频准备：固定 figure 大小，避免 writeVideo 尺寸不一致
fig = figure('Position', [100,100,840,630]);
video_name = 'expressive_nullspace_wave.avi';
v = VideoWriter(video_name);
v.FrameRate = 1/dt;
open(v);

t = 0;  % 全局时间

%% 主循环
for k = 1:num_steps
    % 当前时间
    t = (k-1)*dt;
    
    % --- 计算当前往返目标 p_des ---
    % 计算在周期内的位置：u = mod(t, T_period)
    u = mod(t, T_period);
    if u <= T_half
        % 前半程：p_start -> p_end
        alpha = u / T_half;  % ∈[0,1]
    else
        % 后半程：p_end -> p_start
        alpha = (u - T_half) / T_half;  % ∈[0,1]
        alpha = 1 - alpha;  % 先到1再回到0
    end
    p_des = (1-alpha)*p_start' + alpha*p_end';  % 3×1 列向量

    % --- 主任务: 末端速度控制 ---
    T_cur = robot.fkine(q);
    pos_cur = T_cur.t;                % 3×1
    v_des_full = (p_des - pos_cur)/dt;  % 3×1
    
    % 控制全部XYZ位置，或只XY平面挥手保持高度：这里用 XYZ
    J6 = robot.jacob0(q);          % 6×6
    Jp = J6(1:3, :);               % 3×6
    qdot_task = pinv(Jp) * v_des_full;  % 6×1

    % --- PAD 状态选取（根据 t 分段）---
    if t < 15
        PAD = [0, 0, 0];
    elseif t < 30
        PAD = [1, 1, 1];
    elseif t < 45
        PAD = [-1, -1, -1];
    elseif t < 60
        PAD = [-1, 1, -1];
    else
        PAD = [-1, 1, 1];
    end
    P = PAD(1); A = PAD(2); D = PAD(3);

    % --- 计算 JVG ---
    uP = (P + 1)/2; uA = (A + 1)/2; uD = (D + 1)/2;
    J_base = 1 - uP;
    Jv = J_base.^alphaJ;
    Vv = uA.^alphaV;
    Gv = uD.^alphaG;

    % --- 次任务 qdot0 设计 ---
    f_cur = f_min + (f_max - f_min) * Jv;
    amp_vec = k_V .* Vv;
    osc = amp_vec .* sin(2*pi*f_cur*t + phi0);

    target_offset = Gv * Delta_q_open;
    current_offset = q - q_neutral;
    offset_rate = (target_offset - current_offset)/offset_time_const;
    offset_rate = offset_rate * k_G;

    qdot0 = offset_rate' + osc';

    % --- 零空间合成 ---
    Nmat = eye(n) - pinv(Jp)*Jp;
    qdot = qdot_task + Nmat * qdot0;

    % 更新 q
    q = q + (qdot' * dt);

    % --- 可视化并标注 PAD ---
    figure(fig);  % 保证在同一 figure
    robot.plot(q);
    str= sprintf('PAD = [%d, %d, %d]\nJVG = [%.2f, %.2f, %.2f]', PAD(1), PAD(2), PAD(3), Jv, Vv, Gv);
    title(str, 'FontSize', 16, 'Color', 'm', 'FontWeight', 'bold');

    drawnow;

    % 录制当前帧
    frame = getframe(fig);
    writeVideo(v, frame);
end

close(v);
fprintf('已保存视频: %s\n', video_name);

