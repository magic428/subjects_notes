 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %start from the very beginning,and to create greatness
 %@author: LinChuangwei 
 %@E-mail：979951191@qq.com
 %@brief：卡尔曼滤波的小测试程序
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function lcw_kalman_plot
% clear all;
clc;
close all;
%打开一个文本文件，使用时将数据保存在一个*.txt文件里面
[fname, fpath] = uigetfile(...
    {'*.txt', '*.*'}, ...
    'Pick a file');

z = load(fullfile(fpath, fname));%加载所有的原始数据，matlab是从1开始的,这个值也就是测量值
lcw_length = length(z);%求长度

%分配内存
xlcw_ = zeros(1, lcw_length);%x的先验估计
xlcw = zeros(1, lcw_length);%x的后验估计
residual = zeros(1, lcw_length);%滤波残余
p_ = ones(1, lcw_length);%先验均方差
p = ones(1, lcw_length);%后验均方差
k = zeros(1, lcw_length);%卡尔曼增益
%几个初始化   
%状态模型: x(t) = a x(t-1) + w(t-1)
%测量模型: z(t) = h x(t) + v(t)
%初步猜测，这两个数是猜测的，对于卡尔曼滤波算法来说，这两个值是否准确并不十分重要
xlcw(1) = 40;%需要调
p(1) = 5e2;%需要调
A = 1;
H = 1;%h是激励倍数，实际工作中h可能会随着时间而变化，在这里假定为常标量,但是我们可以改
Q = 10e-4;%5e2;%这两个值的初始化就很关键了   噪声协方差  %需要调
R = 5e2;%10e-4;%需要调

%卡尔曼滤波
%进入循环过程。下面nsim为总循环次数
for t = 2 : lcw_length,
%预测
xlcw_(t) = A * xlcw(t-1);%x的先验估计由上一个时间点的后验估计值和输入信息给出，这里没有输入，b为0
residual(t) = z(t) - H * xlcw_(t);%z(t)的实际测量值和预测值之间的差为滤波过程的残余(革新)
p_(t) = A * A * p(t-1) + Q;%计算先验均方差
%校准
k(t) = H * p_(t)/(H * H * p_(t) + R);%k(t)为卡尔曼增益或卡尔曼混合系数
p(t) = p_(t) * (1 - H * k(t));%计算后验均方差
xlcw(t) = xlcw_(t) + k(t) * residual(t);%利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
end

%下面开始绘图
t = 1: lcw_length;
%画出状态和状态估计
figure;
h1 = plot(t, xlcw_, 'b');
hold on
h2 = plot(t, xlcw, 'g');
h3 = plot(t, z, 'r');
xlabel('By LinChuangwei');
hold off
legend([h1(1) h2(1) h3(1)], '先验估计', '后验估计', '测量值');%显示格式
title('先验估计, 后验估计和测量值的比较');
ylabel('状态');
axis(gca,[0,lcw_length,30,70]);%规定画图的范围

%画协方差
figure;
h1 = plot(t, p_, 'b');
hold on;
h2 = plot(t, p, 'g');
hold off
legend([h1(1) h2(1)], '先验估计', '后验估计');
title('先验估计和后验估计的协方差');
ylabel('协方差');
xlabel('By LinChuangwei');
%axis(gca,[0,lcw_length,0,500]);%规定画图的范围

%卡尔曼增益
figure;
h1 = plot(t, k, 'b');
legend([h1(1)], '卡尔曼增益');
title('卡尔曼增益');
ylabel('卡尔曼增益 k');
xlabel('By LinChuangwei');
