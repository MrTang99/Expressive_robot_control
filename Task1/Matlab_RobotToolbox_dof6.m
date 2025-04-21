%% 6-DOF Robot Joint Space PID Control Simulation using Robotics System Toolbox

clear; clc; close all;

%% (1) 定义Denavit-Hartenberg (DH)参数 (Standard DH for Puma 560-like arm)
disp('步骤 1: 定义 DH 参数...');
% [a, alpha, d, theta]
dhparams = [    0,      pi/2,   0.6718,     0;... % Joint 1
            0.4318,       0,    0.15005,    0;... % Joint 2
            0.0203,   -pi/2,        0,      0;... % Joint 3
                 0,    pi/2,   0.4318,     0;... % Joint 4
                 0,   -pi/2,        0,      0;... % Joint 5
                 0,       0,    0.05625,    0];    % Joint 6
disp('DH 参数定义完成。');
fprintf('\n');

%% (2) 创建机器人模型
disp('步骤 2: 创建机器人模型...');
robot = rigidBodyTree('DataFormat', 'column'); % 初始化，数据格式为列向量 [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

bodies = cell(6, 1);
joints = cell(6, 1);
parentName = robot.BaseName; % 初始化父对象名称为 'base'

for i = 1:6
    bodyName = ['link' num2str(i)];
    jointName = ['jnt' num2str(i)];

    bodies{i} = rigidBody(bodyName); % 创建刚体 [12, 3, 4, 5, 13, 6, 7, 14, 8, 9]
    joints{i} = rigidBodyJoint(jointName, 'revolute'); % 创建旋转关节 [12, 3, 4, 5, 13, 6, 7, 14, 8, 9]

    % 使用 DH 参数设置固定变换 [12, 15, 3, 5, 13, 6, 16, 17, 14, 18, 19, 9]
    setFixedTransform(joints{i}, dhparams(i,:), 'dh');

    bodies{i}.Joint = joints{i}; % 将关节赋给刚体 [12, 3, 4, 5, 13, 6, 7, 14, 8, 9]

    % 将刚体添加到机器人树中 [12, 3, 4, 5, 13, 6, 7, 14, 8, 9, 20]
    addBody(robot, bodies{i}, parentName);

    % 更新下一个迭代的父对象名称
    parentName = bodyName;
end

% (可选) 添加一个固定的末端执行器坐标系
ee_body = rigidBody('end_effector');
ee_joint = rigidBodyJoint('ee_joint', 'fixed');
setFixedTransform(ee_joint, eye(4)); % 假设末端坐标系与最后一个关节坐标系重合
ee_body.Joint = ee_joint;
addBody(robot, ee_body, parentName); % 连接到 'link6'

disp('机器人模型创建完成。');
showdetails(robot); % 显示机器人结构详情 [12, 3, 4, 5, 13, 6, 7, 14, 8, 9]
fprintf('\n');

%% (3) 为连杆设置假设的动力学属性
disp('步骤 3: 分配动力学属性...');
% 这些值是示例性的，实际应用中需要精确值 [3, 5, 21]
masses = [3.7, 8.393, 2.27, 1.37, 1.4, 0.2]; % kg (近似Puma连杆质量)
centers_of_mass = [ 0      -0.0256    0.0159;  % Link 1 CoM [x,y,z] 相对于连杆坐标系
                   0.2159   0         -0.07;   % Link 2 CoM
                   0.01015  0.01      -0.05;   % Link 3 CoM
                   0        0.2       -0.015;  % Link 4 CoM
                   0        0         -0.01;   % Link 5 CoM
                   0        0          0.02];  % Link 6 CoM
inertias = [ 0.1   0.1   0.1   0   0   0;       % Link 1 Inertia [Ixx Iyy Izz Iyz Ixz Ixy]
             0.5   0.5   0.1   0   0   0;       % Link 2 Inertia
             0.05  0.01  0.05  0   0   0;       % Link 3 Inertia
             0.02  0.02  0.005 0   0   0;       % Link 4 Inertia
             0.005 0.001 0.005 0   0   0;       % Link 5 Inertia
             0.001 0.001 0.001 0   0   0];      % Link 6 Inertia

% 确定实际的关节数量 (DoF)
num_joints = 0;
for i = 1:robot.NumBodies
    if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
        num_joints = num_joints + 1;
    end
end
if num_joints ~= 6
    warning('机器人模型自由度不为 6，请检查模型创建过程。');
end

% 分配动力学属性给前 num_joints 个连杆
body_counter = 0;
for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    if ~strcmp(body.Joint.Type, 'fixed')
        body_counter = body_counter + 1;
        if body_counter <= num_joints
            body.Mass = masses(body_counter);
            body.CenterOfMass = centers_of_mass(body_counter, :);
            body.Inertia = inertias(body_counter, :);
        end
    end
end
disp('已为连杆 1-6 分配动力学属性。');
fprintf('\n');

%% (4) 设置DataFormat和Gravity
disp('步骤 4: 设置 DataFormat 和 Gravity...');
robot.DataFormat = 'column'; % 动力学函数需要 'column' 或 'row' [3, 4, 22, 23, 24, 25, 8, 26, 21]
robot.Gravity = [0; 0; -9.81]; % 标准重力沿Z轴负方向 (列向量) [3, 4, 27, 23, 25, 26, 21, 28]
disp(['DataFormat 设置为: ' robot.DataFormat]);
grav_str = sprintf('%.2f, %.2f, %.2f', robot.Gravity);   % 先生成一行字符串
disp(['Gravity 设置为: [' grav_str ']']);              % 然后用 disp 拼接
fprintf('\n');

%% (5) 定义仿真参数
disp('步骤 5: 定义仿真参数...');
dt = 0.001; % 仿真时间步长 (s)
T_final = 5.0; % 总仿真时间 (s)
t_span = 0:dt:T_final;
num_steps = length(t_span);
disp(['仿真时间步长 dt = ' num2str(dt) ' s']);
fprintf('\n');

%% (6) 定义PID控制器增益 (假设值)
disp('步骤 6: 定义 PID 增益...');
Kp_val = [ 10,  20, 10,  5,  3,  1]; % 各关节比例增益
Ki_val = [ 10,  20, 10,  5,  3,  1]; % 各关节积分增益
Kd_val = [ 20,  30, 15, 10,  5,  3]; % 各关节微分增益 [29, 30, 31, 32, 33, 34]

Kp_mat = diag(Kp_val);
Ki_mat = diag(Ki_val);
Kd_mat = diag(Kd_val);
disp('PID 增益矩阵 (Kp_mat, Ki_mat, Kd_mat) 定义完成。');
fprintf('\n');

%% (7) 初始化机器人状态
configs = homeConfiguration(robot);

% 如果它返回 struct array，就取字段；否则直接当向量用
if isstruct(configs)
    q_initial = [configs.JointPosition]';
else
    q_initial = configs(:);  % 保证是列向量
end

q_dot_initial = zeros(num_joints, 1);
q = q_initial;
q_dot = q_dot_initial;
integral_error = zeros(num_joints, 1);

disp('机器人状态 (位置, 速度) 和积分误差已初始化。');
fprintf('\n');

%% (8) 定义期望关节空间轨迹 (Quintic Polynomial)
disp('步骤 8: 生成期望轨迹...');
q_target = [pi/2; pi/4; -pi/3; pi/6; pi/2; pi/3]; % 示例目标关节位置 (rad)

if size(q_target, 1) ~= num_joints
    error('q_target 维度与关节数量不匹配。');
end

waypoints = [q_initial, q_target]; % 起点和终点
timepoints = [0, T_final];


% 定义边界条件 (起点和终点速度、加速度为零)
velocity_boundary_condition = zeros(num_joints, 2);
acceleration_boundary_condition = zeros(num_joints, 2);

% 生成轨迹 [41, 42, 43, 44, 45, 46, 47, 48, 49, 50]
[q_des, qdot_des, qddot_des] = quinticpolytraj(waypoints, timepoints, t_span,...
                                               'VelocityBoundaryCondition', velocity_boundary_condition,...
                                               'AccelerationBoundaryCondition', acceleration_boundary_condition);
disp('期望轨迹 (q_des, qdot_des, qddot_des) 已生成。');
fprintf('\n');

%% (9) 构建仿真循环
disp('步骤 9: 开始仿真循环...');
% 预分配历史数据数组以提高效率
q_history = zeros(num_joints, num_steps);
q_dot_history = zeros(num_joints, num_steps);
torque_history = zeros(num_joints, num_steps);
q_des_history_for_plot = zeros(num_joints, num_steps); % 用于绘制期望位置

q_history(:, 1) = q;
q_dot_history(:, 1) = q_dot;
q_des_history_for_plot(:, :) = q_des; % 存储整个期望轨迹

% 可视化设置
figure('Name', 'Robot Simulation - 6 DOF Joint Space PID');
ax = show(robot, q_initial, 'PreservePlot', false, 'Frames', 'on', 'Visuals', 'on'); % [1, 15, 3, 13, 51, 52, 53, 37, 54, 55, 56]
title(ax, sprintf('Simulation Time: %.3f s', 0));
axis(ax, [-1 1 -1 1 -0.5 1.5]); % 根据机器人尺寸调整坐标轴范围
hold(ax, 'on'); % 保持绘图以便更新
visualization_rate = 50; % 每 N 步更新一次可视化 (可调整)

for k = 1:num_steps - 1
    % (a) 获取当前实际状态和期望状态
    q_current = q_history(:, k);
    q_dot_current = q_dot_history(:, k);
    q_d = q_des(:, k);
    qdot_d = qdot_des(:, k);
    qddot_d = qddot_des(:, k);

    % (b) 计算PID误差
    error = q_d - q_current;
    error_dot = qdot_d - q_dot_current;

    % (c) 更新积分误差
    integral_error = integral_error + error * dt;
    % 可在此处添加抗饱和逻辑

    % (d) 计算PID反馈力矩
    torque_PID = Kp_mat * error + Ki_mat * integral_error + Kd_mat * error_dot;

    % (e) 计算前馈力矩 (基于期望运动) [4, 57, 58, 23, 38, 29, 59, 60, 21, 28, 61, 62, 48, 63, 64, 65, 66]
    torque_ff = inverseDynamics(robot, q_d, qdot_d, qddot_d);

    % (f) 计算总关节力矩 [58]
    total_torque = torque_PID + torque_ff;
    % 可在此处添加摩擦模型

    % (g) 计算实际关节加速度 (基于实际状态和总力矩) [12, 4, 67, 57, 25, 68, 69, 26, 21, 28, 61, 62, 48, 63, 64, 65, 66, 70, 71]
    q_ddot = forwardDynamics(robot, q_current, q_dot_current, total_torque);

    % (h) 更新实际状态 (欧拉积分)
    q_dot_next = q_dot_current + q_ddot * dt;
    q_next = q_current + q_dot_current * dt; % 简单欧拉法

    % (i) 记录历史数据
    q_history(:, k+1) = q_next;
    q_dot_history(:, k+1) = q_dot_next;
    torque_history(:, k) = total_torque; % 记录施加于第k步的力矩

    % 更新状态变量以用于下一次迭代
    q = q_next;
    q_dot = q_dot_next;

    % (j) 定期更新可视化
    if mod(k, visualization_rate) == 0
        show(robot, q_current, 'Parent', ax, 'PreservePlot', false, 'Frames', 'on', 'Visuals', 'on'); % [1, 15, 3, 13, 51, 52, 53, 37, 54, 55, 56]
        title(ax, sprintf('Simulation Time: %.3f s', t_span(k+1)));
        drawnow; % 强制更新图形
    end
end
% 记录最后一步的力矩
torque_history(:, num_steps) = total_torque;
disp('仿真循环结束。');
hold(ax, 'off');
fprintf('\n');

%% (10) 绘制仿真结果
disp('步骤 10: 绘制仿真结果...');
figure('Name', 'Simulation Results - 6 DOF Joint Space PID');

% 绘制关节位置 (实际 vs. 期望)
for i = 1:num_joints
    subplot(3, num_joints, i);
    plot(t_span, q_history(i, :), 'b-', t_span, q_des_history_for_plot(i, :), 'r--');
    grid on;
    title(['Joint ' num2str(i) ' Position']);
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    if i == 1
        legend('Actual', 'Desired', 'Location', 'best');
    end
    ylim('padded'); % 调整y轴范围
end

% 绘制关节速度 (实际)
for i = 1:num_joints
    subplot(3, num_joints, i + num_joints);
    plot(t_span, q_dot_history(i, :), 'b-');
    grid on;
    title(['Joint ' num2str(i) ' Velocity']);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    ylim('padded'); % 调整y轴范围
end

% 绘制关节力矩 (施加的总力矩)
for i = 1:num_joints
    subplot(3, num_joints, i + 2*num_joints);
    plot(t_span, torque_history(i, :), 'b-');
    grid on;
    title(['Joint ' num2str(i) ' Torque']);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    ylim('padded'); % 调整y轴范围
end

sgtitle('6-DOF Robot Joint Space PID Control Simulation Results'); % 图形总标题
disp('绘图完成。');