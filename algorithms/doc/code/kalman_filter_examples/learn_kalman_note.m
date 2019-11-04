                                                                    %%
% 
% 卡尔曼滤波器学习笔记（一）中的示例代码
% http://blog.csdn.net/lizilpl/article/details/45268471
% 
% 程序描述了如下一个系统：
%    房间温度为24度
%    房间内连续两个时刻温度差值的标准差为 0.02 度
%    温度计的测量值误差的标准差为 0.5 度
%    对温度的初始估计值为 23.5 度，误差的方差为 1
%    对整个系统的控制量为 0
%    现在需要利用 Kalman Filter 来估计房间的实时温度 

%% 
clear;
clc;

% 初始化参数
n_iter = 100;       % 计算连续 n_iter 个时刻
sz = [n_iter, 1];   % 构造 n_iter x 1 维度的矩阵
x = 24;             % 温度的真实值
Q = 4e-4;           % 对温度预测值的方差
R = 0.25;           % 测量方差，反应温度计的测量精度
T_start = 23.5;     % 温度初始估计值
delta_start = 1;    % 温度初始估计方差
% z 是温度计的测量结果，在真实值的基础上加上了方差为 0.25 的高斯噪声。
z = x + sqrt(R)*randn(sz); 

% 初始化数组
state_kalman = zeros(sz); 
% 对温度的估计值。即在 k 时刻，结合温度计当前测量值与 k-1 时刻预测值，得到的最终估计值
variance_kalman = zeros(sz);  		% 估计值的方差
state_pre = zeros(sz); 				% 对温度的预测
variance_pre = zeros(sz);    		% 预测值的方差
K = zeros(sz);         				% 卡尔曼增益
state_kalman(1) = T_start;   		% 温度估计值初始化
variance_kalman(1) = delta_start;	% 估计值方差初始化

%
%开始迭代计算
for k = 2:n_iter
	% 用上一时刻的最优估计值来作为对当前时刻的温度的预测
	state_pre(k) = state_kalman(k-1);
	% 预测的方差为上一时刻温度最优估计值的方差与高斯噪声方差之和
	variance_pre(k) = variance_kalman(k-1) + Q;

	% 计算卡尔曼增益
	K(k) = variance_pre(k)/( variance_pre(k)+R ); 
	%
	% 结合当前时刻温度计的测量值，对上一时刻的预测进行校正，得到校正后的最优估计。
	% 由于是直接测量，故 C 为 1.
	state_kalman(k) = state_pre(k) + K(k) * (z(k) - state_pre(k)); 
	%计算最终估计值的方差用于下一次迭代
	variance_kalman(k) = (1 - K(k)) * variance_pre(k); 
end

%% 绘图相关 ...
FontSize = 9;
LineWidth = 1;
figure();
plot(z,'k+'); % 画出温度计的测量值
hold on;
plot(state_kalman,'b-','LineWidth', LineWidth) % 画出最优估计值
hold on;
plot(x * ones(sz),'g-','LineWidth', LineWidth); % 画出真实值
legend({'温度测量值', 'Kalman估计值', '真实值'}, 'FontSize',FontSize);

xl = xlabel('时间(分钟)');
yl = ylabel('温度');
set(xl,'fontsize', FontSize);
set(yl,'fontsize', FontSize);
hold off;
set(gca,'FontSize', FontSize);

figure();
valid_iter = [2:n_iter]; % variance_pre not valid at step 1
% 画出最优估计值的方差
plot(valid_iter, variance_kalman([valid_iter]),'LineWidth',LineWidth); 
legend({'Kalman 估计的误差估计'}, 'FontSize',FontSize);
xl=xlabel('时间(分钟)');
yl=ylabel('℃^2');
set(xl,'fontsize',FontSize);
set(yl,'fontsize',FontSize);
set(gca,'FontSize',FontSize);