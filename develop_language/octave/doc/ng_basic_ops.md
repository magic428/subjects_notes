# Octave 基本操作   


## 变量   

a = pi
disp(sprintf('2 decimals % 0.2f', a))
disp(sprintf('6 decimals % 0.6f', a))

format long
format short


## 向量  

```python
A = [1 2; 3 4; 5 6]
size(A)
size(A, 1)  % 第一维的大小
size(A, 2)

v = 1:0.1:2 
v = 1:6 
length(v)

ones(2,3)
C = 2*ones(2, 3)
w = ones(1, 3)
w = zeros(1, 3)
I = eye(4)

% 均匀分布(0, 1) 之间的随机数
w = rand(1, 3)
w = rand(3, 4)

% 高斯分布(0, 1) 之间的随机数
w = randn(1, 3)
w = 6 - sqrt(10) * randn(10000, 1);
hist(w)
hist(w, 50)
```

## 加载数据   

```
pwd
cd data

load featuresX.dat
load pricesY.dat

load('featuresX.dat')
load('pricesY.dat')

who
whos

clear featuresX

v = pricesY(1:10)
save hello.mat v;

clear
load hello.mat   % 压缩保存变量为二进制形式
save hello.txt v -ascii   % 保存为字符形式

A = [1 2; 3 4; 5 6]
A(3, 2)         % 第三行第二列的元素, 从 1 开始, 并非 0
A(2, :)         % 第二行的所有元素
A(:, 2)         % 第二列的所有元素
A([1 3], :)     % 第 1, 3 行的所有元素
A(:, 2) = [10; 11; 12]
A = [A, [100; 101; 102]]  % 添加 1 列
A(:)            % flaten, 展开为 1 行

A = [1 2; 3 4]
B = [11 12; 13 14]
C = [A B]
C = [A; B]
```

## 常用矩阵运算   


```
A = [1 2; 3 4; 5 6]
B = [11 12; 13 14; 15 16]
C = [1 1; 2 2]

A*C
A.*B
A.^2
1./A
log(A)
exp(A)
abs(A)
-A
A + 1

A'   % 转置
max(A)
[val, ind] = max(A)   % 得到矩阵 A 每一列的最大值   

A < 3
[r, c] = find(A < 3)
A = magic(3)   % 实际中并不常用

sum(a)
prod(a)
floor(a)
ceil(a)
max(A, B)  % 返回两个矩阵中对应位置处较大的值
max(A, [], 1)   % 返回矩阵每列中的最大值
max(A, [], 2)   % 返回矩阵每行中的最大值

% 单纯的获取矩阵中的最大值
max(max(A))
max(A(:))


sum(A, 1)
sum(A, 2)
sum(sum(A.*eye(3)))
sum(sum(A.*flipud(eye(3))))


pinv(A)
inv(A)
I = A*pinv(A)
```

## 数据可视化   

```
t = [0:0.01:0.98]
y1 = sin(2*pi*4*t);
figure(1);
plot(t, y1);

hold on;
y2 = cos(2*pi*4*t);
plot(t, y2, 'r');

xlabel('time');
ylabel('value');
legend('sin', 'cos');
title('my plot');
print -djpg 'myplot.jpg'  % print -dpng 'myplot.png'
close

subplot(1,2,1); % 1x2 的 grid
plot(t, y1);
subplot(1,2,2); % 1x2 的 grid
plot(t, y2);
axis([0.5 1 -1 1])

clf;
imagesc(A)
imagesc(A), colorbar
imagesc(A), colorbar, colormap gray;

a = 1, b = 2, c = 3
a = 1; b = 2; c = 3
```

## 控制语句  

```
for i = 1:10,
    v(i) = i^2;
end
```

```
i = 1;
while true,
    i = i+1
    if i <=5,
        v(i) = 100;
        break;
    end
end
```

```
if v(1) == 1,
    disp('the value is one');
elseif v(1) == 2,
    disp('the value is two')
else,
    disp('the value is three')
end
```

.m 函数文件  

```
function y = squareThisNumber(x)
y = x^2;
```

```bash
% 一定要在 .m 文件所在的路径下
squareThisNumber(5)
```

addpath("/home/magic")


函数返回多个值  

```
function [y1, y2] = squareAndCubeThisNumber(x)

y1 = x^2
y2 = x^3
```


