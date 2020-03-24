# 梯度下降法    

自动驾驶中预测方向是一个连续变量问题, 因此属于回归问题.    

**Notations**:    
- $ m $: 样本的个数;    
- $ x $: 输入变量 / 特征;   
- $ n $: 输入变量的个数 / 特征的个数;    
- $ y $: 输出变量 / 目标变量;   
- ($ x, y $): 训练样本;   
- ($ x^{(i)}, y^{(i)} $): 第 i 个训练样本, 即表示训练样本的第 i 行;   v

## 机器学习的流程    

```flow
st=>start: Training Set
learning_algorithm=>subroutine: Learning Algorithm
hyperthis=>operation: Hyperthis
e=>end: predict 
st->learning_algorithm
learning_algorithm->hyperthis(right)->e
```

## 线性回归问题   
1. 函数假设    
$$ h_\theta(x^{(i)}) = \sum_{j=0}^{n} \theta_j x_j^{(i)} = \theta ^ T x^{(i)} $$   
其中,  $ n $ 是特征的个数.    
2. cost 函数      
找到 $ \theta $ 使得下面的函数值最小:   
$$ J(\theta) = \frac{1}{2} \sum_{i = 1}^{m} (h_\theta(x^{(i)}) - y^{(i)})^2 $$    
3. 优化方法     
梯度下降法: 站在上坡上, 环顾四周之后, 决定我怎么样才能快速到达山脚下. 梯度下降法是沿着下降最快的方向走一步, 而这个方向就是梯度的方向.      
梯度下降法的特点是它最后一定会停下.    
缺点是: 梯度下降法最后达到的最小值依赖于初始点的位置, 也就是说它可能陷入局部最优而不是全局最优.     
**算法实现**:    
(1) 从某一个 $ \theta $ 值( 比如 $ \theta = \vec{0} $ )开始, 不断更新 $ \theta $ 的值来使 cost 函数最小化.   
(2) 使用梯度下降法更新 $ \theta $ 的值: $$ \theta := \theta_i - \alpha \dfrac{\partial}{\partial\theta_i}J(\theta) $$     
4. 一个样本的情形    
梯度 $ \dfrac{\partial}{\partial\theta_i}J(\theta) $ 推导过程为:    
$= \dfrac{\partial}{\partial\theta_i}(\dfrac{1}{2}(h_\theta(x) - y)^2) $
$= \dfrac{1}{2} * 2 *(h_\theta(x) - y) * \dfrac{\partial}{\partial\theta_i}(h_\theta(x) - y) $    
$= (h_\theta(x) - y) * \dfrac{\partial}{\partial\theta_i}(\sum_{i=0}^{n} \theta_i x_i - y) $
$= (h_\theta(x) - y) * \dfrac{\partial}{\partial\theta_i}(\theta_0 x_0 + \theta_1 x_1 + \theta_i x_i + ... + \theta_m x_m - y) $
$= (h_\theta(x) - y) * x_i $
5. 多个样本的情形    
梯度为:   
$$ \dfrac{\partial}{\partial\theta_i}J(\theta) = \sum_{j = 1}^{m}(h_\theta(x^{(j)}) - y^{(j)}) * x_i^{(j)} $$   
6. 是否收敛的判断指标     
(1) 比较参数在两次梯度下降之间的变化程度;    
(2) 比较目标函数在迭代中是否变得更小;     
7. batch gradient descent 和 stochastic gradient descent (SGD)    
当数据集很大的时候, batch gradient descent 算法的效率很低. SGD 则是随机选取一个样本计算梯度.       
8. 线性回归最后得到的结果    
是 $$ \theta ^ T x $$    

## Normal Equation    

$$ \theta = (X^TX)^{-1}X^Ty $$   

$$ \theta = \prod_{i = 0}^{m} (X^TX)^{-1}X^Ty $$   
 
 