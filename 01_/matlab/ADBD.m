%% 1. 定义物理参数 (必须与C++里的一致)
M = 0.5;  
m = 0.2;  
l = 0.5;  
g = 9.8;
Ts = 0.01;  % 控制周期 (即 C++ 里的 dt_control)

%% 2. 构建连续矩阵 A, B
% 状态向量: x = [位置; 速度; 角度; 角速度]
A = [0, 1, 0, 0;
     0, 0, -m*g/M, 0;
     0, 0, 0, 1;
     0, 0, (M+m)*g/(M*l), 0];
B = [0; 
     1/M; 
     0; 
     -1/(M*l)];
C = [1, 0, 0, 0;  
     0, 0, 1, 0];
D = [0;0];

%% 3. 离散化 (Magic Step)
% 使用 'zoh' (零阶保持器) 方法
sys_continuous = ss(A, B, C, D);
sys_discrete = c2d(sys_continuous, Ts, 'zoh');
Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;

%% 4. Luenberger 观测器设计 (极点配置法 - 旧方案)
observer_poles = [0.8, 0.81, 0.82, 0.83]; 
L_luenberger = place(Ad', Cd', observer_poles)'; 

%% 5. [新增] 卡尔曼滤波器设计 (最优估计法 - 新方案)
% 核心思想：不用你去猜极点，而是让你填“信任度”

% G: 过程噪声驱动矩阵 (假设噪声独立作用于每个状态)
G = eye(4); 

% --- 调参区域 (这是卡尔曼的灵魂) ---
% Q: 过程噪声协方差 (Process Noise Covariance)
%    含义：你的物理模型有多不准？或者外界风吹草动有多大？
%    数值越大 -> 越不信模型 -> K越大 -> 动态响应快，但噪音大
Q = diag([1e-1, 1e-1, 1e-1, 1e-1]); 

% R: 测量噪声协方差 (Measurement Noise Covariance)
%    含义：你的传感器有多烂？(可以通过静止采集数据算方差得到)
%    数值越大 -> 越不信传感器 -> K越小 -> 滤波更平滑，但有滞后
%    R(1,1)是位置噪声，R(2,2)是角度噪声
R = diag([1e-4, 1e-4]); 
% ---------------------------------

% 使用 MATLAB 的 dlqe (Discrete Linear Quadratic Estimator) 函数求解
% 它会自动求解代数黎卡提方程 (DARE)
[K_kalman, P_ss, E_kalman] = dlqe(Ad, G, Cd, Q, R);

%% 6. 输出结果供 C++ 使用
clc;
fprintf('------------------------------------------------\n');
disp('Ad Matrix (4x4):');
disp(Ad);
fprintf('------------------------------------------------\n');
disp('Bd Matrix (4x1):');
disp(Bd);
fprintf('------------------------------------------------\n');
disp('=== 方案A: Luenberger Gain L (4x2) ==='); 
disp(L_luenberger);
fprintf('------------------------------------------------\n');
disp('=== 方案B: Kalman Gain K (4x2) [推荐] ===');
disp(K_kalman);
fprintf('------------------------------------------------\n');