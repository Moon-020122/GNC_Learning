%% 1. 初始化
clc; clear; close all;

%% 2. 读取数据
% 确保这里的路径指向你最新的 csv 文件
filepath = 'E:\DockerDesktopWSL\GNC_Learning\01_\inverted_pendulum\data\run.csv';
data = readtable(filepath);

% 自动提取变量 (根据你的 C++ header: t,x,xdot,theta,thetadot,u,theta_ref,x_hat,theta_hat)
t         = data.t;           % 时间
x         = data.x;           % 真实位置 (传感器/仿真真值)
x_dot     = data.xdot;        % 真实速度
theta     = data.theta;       % 真实角度
th_dot    = data.thetadot;    % 真实角速度
u         = data.u;           % 控制量
theta_ref = data.theta_ref;   % 参考角度
x_hat     = data.x_hat;       % 【新增】观测器估计位置
theta_hat = data.theta_hat;   % 【新增】观测器估计角度

%% 3. 绘图 (3行2列布局)
figure('Name', 'Observer & Control Performance', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- (1) 位置对比: 真实 x vs 估计 x_hat ---
subplot(3, 2, 1);
hold on;
plot(t, x, 'b', 'LineWidth', 1.5);         % 真实值 (蓝色实线)
plot(t, x_hat, 'r--', 'LineWidth', 1.5);   % 估计值 (红色虚线)
yline(0, '--k', 'Target');                 % 0位参考线
hold off;
grid on; 
ylabel('Position (m)'); 
title('Position: True(x) vs Observer(\hat{x})');
legend('True x', 'Est \hat{x}', 'Location', 'best');

% --- (2) 角度对比: 真实 theta vs 估计 theta_hat vs 参考 ref ---
subplot(3, 2, 2);
hold on;
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.5);       % 真实角度 (蓝色实线)
plot(t, rad2deg(theta_hat), 'r--', 'LineWidth', 1.5); % 估计角度 (红色虚线)
plot(t, rad2deg(theta_ref), 'g-.', 'LineWidth', 1.2); % 参考角度 (绿色点划线)
hold off;
grid on; 
ylabel('Angle (deg)'); 
title('Angle: True(\theta) vs Obs(\hat{\theta}) vs Ref');
legend('True \theta', 'Est \hat{\theta}', 'Ref', 'Location', 'best');

% --- (3) 速度 x_dot (你没log速度的估计值，所以只画真值) ---
subplot(3, 2, 3);
plot(t, x_dot, 'b', 'LineWidth', 1.5);
grid on; ylabel('Velocity (m/s)'); title('True Velocity \dot{x}');

% --- (4) 角速度 theta_dot ---
subplot(3, 2, 4);
plot(t, rad2deg(th_dot), 'm', 'LineWidth', 1.5);
grid on; ylabel('Rate (deg/s)'); title('True Angular Rate \dot{\theta}');

% --- (5) 控制量 u ---
subplot(3, 2, [5, 6]); 
plot(t, u, 'k', 'LineWidth', 1.2);
grid on; 
xlabel('Time (s)'); ylabel('Control Input (V or N)'); 
title('Control Effort u');

% 绘制饱和线 (假设你的电机最大电压是 10V 或 24V，这里画两条红线示意)
yline(10, '--r', 'Limit (+)');
yline(-10, '--r', 'Limit (-)');
legend('Control u', 'Saturation Limit');

%% 4. (可选) 单独画一个观测误差图
% 这一步对于调试观测器极点非常有用，看误差收敛得快不快
figure('Name', 'Observer Error', 'Color', 'w', 'Position', [150, 150, 800, 600]);
subplot(2,1,1);
plot(t, x - x_hat, 'k', 'LineWidth', 1.5);
grid on; title('Position Estimation Error (x - \hat{x})'); ylabel('Error (m)');

subplot(2,1,2);
plot(t, rad2deg(theta - theta_hat), 'k', 'LineWidth', 1.5);
grid on; title('Angle Estimation Error (\theta - \hat{\theta})'); ylabel('Error (deg)');
xlabel('Time (s)');